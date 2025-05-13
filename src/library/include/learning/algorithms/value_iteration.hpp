// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_LEARNING_ALGORITHMS_VALUE_ITERATION_HPP
#define MAP_MATCHING_2_LEARNING_ALGORITHMS_VALUE_ITERATION_HPP

#include <string>
#include <vector>
#include <deque>

#include <boost/unordered/unordered_flat_set.hpp>

#include "matching/settings.hpp"

namespace map_matching_2::learning {

    template<typename Environment>
    class value_iteration {

    public:
        using environment_type = Environment;

        using state_type = typename environment_type::state_type;
        using action_type = typename environment_type::action_type;
        using reward_type = typename environment_type::reward_type;

        const std::string name = "value_iteration";

        explicit value_iteration(environment_type &environment, const matching::settings &settings = {})
            : _environment{environment}, _settings{settings} {}

        [[nodiscard]] environment_type &environment() {
            return _environment;
        }

        std::size_t generate_states(std::vector<state_type> &states) {
            auto state = _environment.reset();
            state_type highest_state = state;

            boost::unordered_flat_set<state_type> states_visited;
            std::deque<state_type> states_to_visit;

            states_to_visit.emplace_back(state);
            states_visited.emplace(state);
            states.emplace_back(state);

            while (not states_to_visit.empty()) {
                state = states_to_visit.front();
                states_to_visit.pop_front();

                _environment.state = state;

                const auto actions = _environment.actions();

                for (auto action : actions) {
                    auto result = _environment.step(action);
                    auto new_state = std::get<0>(result);
                    auto done = std::get<3>(result);

                    if (not done and not states_visited.contains(new_state)) {
                        states_to_visit.emplace_back(new_state);
                        states.emplace_back(new_state);
                        states_visited.emplace(new_state);
                    }

                    if (new_state > highest_state) {
                        highest_state = new_state;
                    }

                    _environment.state = state;

                    if (_environment.abort()) {
                        return highest_state;
                    }
                }

                // if (states.size() >= 1000 and states.size() < 10000 and
                //     states.size() % 1000 == 0 or states.size() % 10000 == 0) {
                //     std::cout << "States: " << states.size() << ", to visit: " << states_to_visit.size() << std::endl;
                // }
            }

            // std::cout << "Final States: " << states.size() << std::endl;
            return highest_state;
        }

        std::vector<std::pair<std::size_t, std::size_t>> operator()() {
            assert(std::is_unsigned_v<state_type>);
            assert(std::is_unsigned_v<action_type>);

            bool trained = false;

            _environment.init();

            std::vector<std::pair<std::size_t, std::size_t>> policy;

            if (_environment.abort()) {
                return policy;
            }

            std::vector<state_type> states;
            std::vector<double> V;

            std::size_t highest_state = generate_states(states);

            if (_environment.abort()) {
                return policy;
            }

            V.reserve(highest_state + 1);
            while (V.size() < V.capacity()) {
                V.emplace_back(0.0);
            }

            double delta = std::numeric_limits<double>::infinity();
            double value = 0.0, old_value = 0.0;
            std::size_t episode = 0;

            while (delta > _settings.threshold) {
                old_value = value;
                value = 0.0;

                for (auto it = states.crbegin(); it != states.crend(); ++it) {
                    const auto &state = *it;
                    _environment.state = state;

                    const auto actions = _environment.actions();

                    if (not actions.empty()) {
                        double max_action_value = std::numeric_limits<double>::lowest();

                        for (const auto action : actions) {
                            const auto result = _environment.step(action);
                            const auto &new_state = std::get<0>(result);
                            const auto reward = std::get<2>(result);
                            const auto done = std::get<3>(result);

                            double state_value = reward;
                            if (not done) {
                                state_value += _settings.discount * V[new_state];
                            }

                            if (state_value > max_action_value) {
                                max_action_value = state_value;
                            }

                            _environment.state = state;
                        }

                        V[state] = max_action_value;
                        trained = true;

                        value += max_action_value;
                    }

                    if (_environment.abort()) {
                        return policy;
                    }
                }

                const double new_delta = std::fabs(value - old_value);
                if (new_delta < delta) {
                    delta = new_delta;
                }

                episode++;

                // std::cout << "Episode: " << episode << ", value: " << std::setprecision(20) << value << std::endl;
            }

            auto state = _environment.reset();
            bool finished = false;
            double score = 0.0;

            std::vector<reward_type> rewards;
            policy.clear();

            if (trained) {
                while (not finished) {
                    const auto actions = _environment.actions();

                    if (not actions.empty()) {
                        action_type max_action{};
                        double max_action_value = std::numeric_limits<double>::lowest();

                        for (const auto action : actions) {
                            const auto result = _environment.step(action);
                            const auto &new_state = std::get<0>(result);
                            const auto reward = std::get<2>(result);
                            const auto done = std::get<3>(result);

                            double state_value = reward;
                            if (not done) {
                                state_value += _settings.discount * V[new_state];
                            }

                            if (state_value > max_action_value) {
                                max_action_value = state_value;
                                max_action = action;
                            }

                            _environment.state = state;
                        }

                        const auto current_state = _environment.state;
                        const auto result = _environment.step(max_action);
                        const auto &new_state = std::get<0>(result);
                        const auto new_action = std::get<1>(result);
                        const auto reward = std::get<2>(result);
                        finished = std::get<3>(result);

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
                    } else {
                        // cannot continue if there are no more actions
                        finished = true;
                    }
                }
            }

            // std::cout << " Score: " << score << std::endl;

            return policy;
        }

    private:
        environment_type &_environment;
        const matching::settings &_settings;

    };

}

#endif //MAP_MATCHING_2_LEARNING_ALGORITHMS_VALUE_ITERATION_HPP
