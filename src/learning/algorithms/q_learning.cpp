// Copyright (C) 2021-2021 Adrian Wöltche
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see https://www.gnu.org/licenses/.

#include "q_learning.hpp"

#include <chrono>
#include <unordered_map>

#include <boost/functional/hash.hpp>

#include <environment/environments/single.hpp>
#include <environment/environments/single_performance.hpp>
#include <geometry/util.hpp>
#include <io/csv_exporter.hpp>
#include <matching/types.hpp>

namespace map_matching_2::learning {

    template<typename Environment>
    q_learning<Environment>::q_learning(Environment &environment, const learning::settings &settings)
            : _environment{environment}, _settings{settings} {
        std::uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
        _random.seed(seed);
        _environment.intelligent_action = true;
    }

    template<typename Environment>
    std::vector<std::pair<std::size_t, std::size_t>> q_learning<Environment>::operator()() {
        const auto action_reward_comparator = [](const auto &a, const auto &b) {
            return a.second < b.second;
        };

        std::size_t episode = 1;

        std::unordered_map<state_type, std::unordered_map<action_type, double>, boost::hash<state_type>> Q;
        std::vector<reward_type> rewards;
        std::vector<std::pair<std::size_t, std::size_t>> policy;

//        io::csv_exporter csv{"score.csv"};
//        csv << std::vector{"score", "best"};

        _environment.init();

        state_type state;
        double score;
        double best_score = -std::numeric_limits<double>::infinity();
        double best_policy_score;
        std::size_t no_improvement = 0;
        std::size_t improvement_stop = 0;
        std::for_each(_environment.candidates().cbegin(), _environment.candidates().cend(),
                      [&improvement_stop](const auto &candidate) {
                          improvement_stop += candidate.edges.size();
                      });
        bool done;

        while (episode <= _settings.episodes) {
            state = _environment.reset();
            score = 0.0;
            std::size_t step = 0;
            std::size_t max_steps = _environment.candidates().size() * 2;
            state_type prev_state_1, prev_state_2;
            bool skipped = false;
            done = false;

            while (not done and step < max_steps) {
                bool skip_loop = (step >= 1 and prev_state_1 == state) or
                                 (step >= 2 and prev_state_2 == state);

                bool no_action = true;
                action_type action;
                if (not skip_loop and _epsilon(_random) > _settings.epsilon) {
                    // with the probabilty of (1 - epsilon) take the best action in our Q-table
                    const auto state_it = Q.find(state);
                    if (state_it != Q.cend()) {
                        const auto &action_map = (*state_it).second;
                        const auto action_it = std::max_element(
                                action_map.cbegin(), action_map.cend(), action_reward_comparator);
                        if (action_it != action_map.cend()) {
                            action = (*action_it).first;
                            no_action = false;
                        }
                    }
                }
                if (no_action) {
                    // else take a random action
                    const auto actions = _environment.actions();

                    if (not _environment.intelligent_action or skipped or skip_loop) {
                        std::uniform_int_distribution<std::size_t> action_dist{0, actions.size() - 1};
                        action = actions[action_dist(_random)];
                    } else {
                        std::exponential_distribution action_dist{0.5};
                        action = actions[std::min(actions.size() - 1, (std::size_t) action_dist(_random))];
                    }
                }

                // step environment
                prev_state_2 = prev_state_1;
                prev_state_1 = _environment.state;
                const auto result = _environment.step(action);
                const auto &new_state = std::get<0>(result);
                const auto new_action = std::get<1>(result);
                const auto reward = std::get<2>(result);
                done = std::get<3>(result);

                if constexpr(Environment::performance) {
                    skipped = _environment.action2internal(new_action) < 0;
                } else {
                    skipped = new_action < 0;
                }

                // update Q-table
                double current_Q = 0;
                auto state_it = Q.find(state);
                if (state_it != Q.cend()) {
                    const auto &action_map = (*state_it).second;
                    const auto action_it = action_map.find(action);
                    if (action_it != action_map.cend()) {
                        current_Q = (*action_it).second;
                    }
                }
                double max_next_Q = 0;
                const auto next_state_it = Q.find(new_state);
                if (next_state_it != Q.cend()) {
                    const auto &next_action_map = (*next_state_it).second;
                    const auto next_action_it = std::max_element(
                            next_action_map.cbegin(), next_action_map.cend(), action_reward_comparator);
                    if (next_action_it != next_action_map.cend()) {
                        max_next_Q = (*next_action_it).second;
                    }
                }

                const double new_Q = current_Q +
                                     _settings.learning_rate * (reward + _settings.discount * max_next_Q) -
                                     current_Q;

                if (state_it == Q.cend()) {
                    const auto state_insert = Q.emplace(state, std::unordered_map<action_type, double>{});
                    state_it = state_insert.first;
                }
                auto &action_map = (*state_it).second;
                action_map.insert_or_assign(action, new_Q);

                // advance state
                score += reward;
                state = new_state;

                step++;
            }

            // mark best score and reset no_improvement counter
            if (score > best_score) {
                best_score = score;
                no_improvement = 0;
            } else {
                no_improvement++;
            }

            //            csv << std::vector{std::to_string(score), std::to_string(best_score)};

            if (no_improvement >= improvement_stop) {
                best_policy_score = best_score;

                // extract policy
                policy.clear();
                rewards.clear();
                state = _environment.reset();
                score = 0.0;
                step = 0;
                done = false;

                while (not done and step < max_steps) {
                    const auto state_it = Q.find(state);
                    if (state_it == Q.cend()) {
                        // reached a point we never visited
                        score = -std::numeric_limits<double>::infinity();
                        no_improvement = 0;
                        break;
                    }

                    const auto &action_map = (*state_it).second;
                    const auto action_it = std::max_element(
                            action_map.cbegin(), action_map.cend(), action_reward_comparator);
                    if (action_it == action_map.cend()) {
                        // reached a point we never visited
                        score = -std::numeric_limits<double>::infinity();
                        no_improvement = 0;
                        break;
                    }
                    action_type action = (*action_it).first;

                    // step environment
                    const auto current_state = _environment.state;
                    const auto result = _environment.step(action);
                    const auto &new_state = std::get<0>(result);
                    const auto new_action = std::get<1>(result);
                    const auto reward = std::get<2>(result);
                    done = std::get<3>(result);

                    rewards.emplace_back(reward);

                    std::size_t candidate;
                    std::int64_t edge;
                    if constexpr(Environment::performance) {
                        candidate = _environment.state2internal(current_state).back();
                        edge = _environment.action2internal(new_action);
                    } else {
                        candidate = current_state.back();
                        edge = new_action;
                    }
                    if (edge >= 0) {
                        policy.emplace_back(std::pair{candidate, edge});
                    } else if (not policy.empty()) {
                        policy.emplace_back(policy.back());
                    }

                    score += reward;
                    state = new_state;

                    step++;
                }

                //                    std::cout << " Policy Score: " << score << std::endl;

                if (score > best_policy_score) {
                    best_score = score;
                    no_improvement = 0;
                } else if (no_improvement >= improvement_stop) {
                    //                        std::cout << "Stop after episode " << episode << std::endl;
                    break;
                }
            }

            //                std::cout << "Episode: " << episode << ", score: " << std::setprecision(20) << score << std::endl;
            episode++;
        }

        return policy;
    }

    template
    class q_learning<environment::single<matching::types_geographic::matcher_static>>;

    template
    class q_learning<environment::single<matching::types_cartesian::matcher_static>>;

    template
    class q_learning<environment::single_performance<matching::types_geographic::matcher_static>>;

    template
    class q_learning<environment::single_performance<matching::types_cartesian::matcher_static>>;

}
