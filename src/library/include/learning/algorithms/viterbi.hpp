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

#ifndef MAP_MATCHING_2_LEARNING_ALGORITHMS_VITERBI_HPP
#define MAP_MATCHING_2_LEARNING_ALGORITHMS_VITERBI_HPP

#include <string>
#include <vector>
#include <cmath>

#include "matching/settings.hpp"

namespace map_matching_2::learning {

    template<typename Environment>
    class viterbi {

    public:
        using environment_type = Environment;

        using action_type = typename environment_type::action_type;

        const std::string name = "viterbi";

        explicit viterbi(environment_type &environment, const matching::settings &settings = {})
            : _environment{environment}, _settings{settings} {}

        [[nodiscard]] environment_type &environment() {
            return _environment;
        }

        std::vector<std::pair<std::size_t, std::size_t>> operator()() {
            const auto &observations = _environment.observations();
            const auto &transitions = _environment.transitions();

            std::vector<action_type> selections;
            if (not observations.empty()) {
                // initialize T and K
                std::vector<std::vector<double>> T;
                T.reserve(observations.size());
                std::vector<std::vector<action_type>> K;
                K.reserve(observations.size());
                for (std::size_t t = 0; t < observations.size(); ++t) {
                    std::vector<double> probabilities(observations[t].size(), std::log(0.0));
                    T.emplace_back(std::move(probabilities));
                    std::vector<action_type> paths(
                            observations[t].size(), _environment.no_action);
                    K.emplace_back(std::move(paths));
                }

                // initial probabilities
                bool initialized = false;
                for (std::size_t g = 0; g < observations[0].size(); ++g) {
                    T[0][g] = transitions[0][g][0] + observations[0][g];
                    initialized = true;
                }

                // calculate path probabilities
                for (std::size_t t = 1; t < observations.size(); ++t) {
                    bool initializing = false;
                    for (std::size_t g = 0; g < observations[t].size(); ++g) {
                        double max_prob = -std::numeric_limits<double>::infinity();
                        action_type k = _environment.no_action;
                        for (std::size_t s = 0; s < observations[t - 1].size(); ++s) {
                            double prob = T[t - 1][s] + transitions[t][g][s] + observations[t][g];
                            if (prob > max_prob) {
                                max_prob = prob;
                                k = s;
                            }
                        }

                        K[t][g] = k;
                        if (k >= 0) {
                            T[t][g] = T[t - 1][k] + transitions[t][g][k] + observations[t][g];
                        } else if (not initialized) {
                            // late initialization because of skips
                            T[t][g] = std::log(1.0 / transitions[t].size()) + observations[t][g];
                            initializing = true;
                        }
                    }
                    if (initializing) {
                        initialized = true;
                    }
                }

                // prepare policy
                selections.assign(observations.size(), _environment.no_action);

                // find optimal last state
                double max_prob = -std::numeric_limits<double>::infinity();
                action_type k = _environment.no_action;
                for (std::size_t g = 0; g < observations[observations.size() - 1].size(); ++g) {
                    double prob = T[T.size() - 1][g];
                    if (prob > max_prob) {
                        k = g;
                        max_prob = prob;
                    }
                }
                selections[selections.size() - 1] = k;

                // backtrack path and set policy
                for (std::size_t t = observations.size() - 1; t > 0; --t) {
                    k = selections[t];
                    if (k >= 0) {
                        selections[t - 1] = K[t][k];
                    }
                }
            }

            std::vector<std::pair<std::size_t, std::size_t>> policy;
            policy.reserve(selections.size());
            for (std::size_t candidate = 0; candidate < observations.size(); ++candidate) {
                std::int64_t edge = selections[candidate];
                if (edge >= 0) {
                    policy.emplace_back(std::pair{candidate, edge});
                }
            }

            return policy;
        }

    private:
        environment_type &_environment;
        const matching::settings &_settings;

    };

}

#endif //MAP_MATCHING_2_LEARNING_ALGORITHMS_VITERBI_HPP
