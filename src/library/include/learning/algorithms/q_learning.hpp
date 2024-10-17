// Copyright (C) 2021-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_LEARNING_ALGORITHMS_Q_LEARNING_HPP
#define MAP_MATCHING_2_LEARNING_ALGORITHMS_Q_LEARNING_HPP

#include <string>
#include <vector>
#include <random>

#include "matching/settings.hpp"

namespace map_matching_2::learning {

    template<typename Environment>
    class q_learning {

    public:
        using environment_type = Environment;

        using state_type = typename environment_type::state_type;
        using action_type = typename environment_type::action_type;
        using reward_type = typename environment_type::reward_type;

        const std::string name = "q_learning";

        explicit q_learning(environment_type &environment, const matching::settings &settings = {})
            : _environment{environment}, _settings{settings} {
            std::uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
            _random.seed(seed);
            _environment.intelligent_action = true;
        }

        [[nodiscard]] environment_type &environment() {
            return _environment;
        }

        std::vector<std::pair<std::size_t, std::size_t>> operator()() {
            assert(std::is_unsigned_v<state_type>);
            assert(std::is_unsigned_v<action_type>);
            std::vector<std::pair<std::size_t, std::size_t>> policy;

            // io::csv_exporter csv{"score.csv"};
            // csv << std::vector{"score", "best"};

            _environment.init();

            if (_environment.abort()) {
                return policy;
            }

            std::vector<std::vector<double>> Q;
            std::vector<reward_type> rewards;

            std::size_t episode = 1;

            state_type state;
            double score;
            double best_score = -std::numeric_limits<double>::infinity();
            double best_policy_score;
            std::size_t no_improvement = 0;
            std::size_t improvement_stop = _environment.edges() * _settings.early_stop_factor;
            bool done;

            while (episode <= _settings.episodes) {
                state = _environment.reset();
                score = 0.0;
                std::size_t step = 0;
                std::size_t max_steps = _environment.candidates().size() * 2;
                done = false;

                while (not done and step < max_steps) {
                    while (state >= Q.size()) {
                        Q.resize(Q.empty() ? 16 : Q.size() * 2);
                    }

                    bool no_action = true;
                    action_type action;
                    if (_epsilon(_random) > _settings.epsilon) {
                        // with the probabilty of (1 - epsilon) take the best action in our Q-table
                        const auto &actions = Q[state];
                        const auto action_it = std::max_element(actions.cbegin(), actions.cend());
                        if (action_it != actions.cend() and *action_it > -std::numeric_limits<double>::infinity()) {
                            action = std::distance(actions.cbegin(), action_it);
                            no_action = false;
                        }
                    }
                    if (no_action) {
                        // else take a random action
                        const auto actions = _environment.actions();
                        if (Q[state].empty()) {
                            std::vector<double> Q_actions(actions.size() + _environment.special_actions,
                                    -std::numeric_limits<double>::infinity());
                            Q[state] = std::move(Q_actions);
                        }

                        if (not _environment.intelligent_action) {
                            std::uniform_int_distribution<std::size_t> action_dist{0, actions.size() - 1};
                            action = actions[action_dist(_random)];
                        } else {
                            std::exponential_distribution action_dist{0.5};
                            action = actions[std::min(actions.size() - 1, (std::size_t) action_dist(_random))];
                        }
                    }

                    // step environment
                    const auto result = _environment.step(action);
                    const auto &new_state = std::get<0>(result);
                    const auto new_action = std::get<1>(result);
                    const auto reward = std::get<2>(result);
                    done = std::get<3>(result);

                    // fix Q-table if action is larger
                    while (action >= Q[state].size()) {
                        Q[state].emplace_back(-std::numeric_limits<double>::infinity());
                    }

                    // update Q-table
                    double current_Q = Q[state][action];
                    if (current_Q == -std::numeric_limits<double>::infinity()) {
                        current_Q = 0;
                    }
                    double max_next_Q = 0;
                    if (new_state < Q.size()) {
                        const auto &next_actions = Q[new_state];
                        const auto next_action_it = std::max_element(next_actions.cbegin(), next_actions.cend());
                        if (next_action_it != next_actions.cend() and
                            *next_action_it > -std::numeric_limits<double>::infinity()) {
                            max_next_Q = *next_action_it;
                        }
                    }

                    Q[state][action] = current_Q +
                            _settings.learning_rate * (reward + _settings.discount * max_next_Q) -
                            current_Q;

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

                // csv << std::vector{std::to_string(score), std::to_string(best_score)};

                // early stop when max-time reached
                bool fixed_time_stop = false;
                if (_environment.abort()) {
                    fixed_time_stop = true;
                }

                if (no_improvement >= improvement_stop or fixed_time_stop) {
                    best_policy_score = best_score;

                    // extract policy
                    rewards.clear();
                    policy.clear();
                    state = _environment.reset();
                    score = 0.0;
                    step = 0;
                    done = false;

                    while (not done and step < max_steps) {
                        if (state >= Q.size()) {
                            // reached a point we never visited
                            score = -std::numeric_limits<double>::infinity();
                            no_improvement = 0;
                            break;
                        }

                        const auto &actions = Q[state];
                        const auto action_it = std::max_element(actions.cbegin(), actions.cend());
                        if (action_it == actions.cend() or *action_it == -std::numeric_limits<double>::infinity()) {
                            // reached a point we never visited
                            score = -std::numeric_limits<double>::infinity();
                            no_improvement = 0;
                            break;
                        }
                        action_type action = std::distance(actions.cbegin(), action_it);

                        // step environment
                        const auto current_state = _environment.state;
                        const auto result = _environment.step(action);
                        const auto &new_state = std::get<0>(result);
                        const auto new_action = std::get<1>(result);
                        const auto reward = std::get<2>(result);
                        done = std::get<3>(result);

                        rewards.emplace_back(reward);

                        std::size_t candidate = _environment.state2internal(current_state).back();
                        std::int64_t edge = _environment.action2internal(new_action);
                        if (edge >= 0) {
                            policy.emplace_back(std::pair{candidate, edge});
                        } else if (not policy.empty()) {
                            policy.emplace_back(policy.back());
                        }

                        score += reward;
                        state = new_state;

                        step++;
                    }

                    // std::cout << " Policy Score: " << score << std::endl;

                    if (score > best_policy_score) {
                        best_score = score;
                        no_improvement = 0;
                    } else if (no_improvement >= improvement_stop or fixed_time_stop) {
                        // std::cout << "Stop after episode " << episode << std::endl;
                        break;
                    }
                }

                // std::cout << "Episode: " << episode << ", score: " << std::setprecision(20) << score << std::endl;
                episode++;
            }

            return policy;
        }

    private:
        environment_type &_environment;
        const matching::settings &_settings;

        std::default_random_engine _random;
        std::uniform_real_distribution<double> _epsilon{0.0, 1.0};

    };

}

#endif //MAP_MATCHING_2_LEARNING_ALGORITHMS_Q_LEARNING_HPP
