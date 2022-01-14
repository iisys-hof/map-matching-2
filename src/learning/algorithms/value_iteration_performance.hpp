// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_VALUE_ITERATION_PERFORMANCE_HPP
#define MAP_MATCHING_2_VALUE_ITERATION_PERFORMANCE_HPP

#include <string>
#include <vector>

#include "../settings.hpp"

namespace map_matching_2::learning {

    template<typename Environment>
    class value_iteration_performance {

    public:
        using environment_type = Environment;

        using state_type = typename Environment::state_type;
        using action_type = typename Environment::action_type;
        using reward_type = typename Environment::reward_type;

        static constexpr bool performance = true;
        const std::string name = "value_iteration";

        value_iteration_performance(
                Environment &environment, const learning::settings &settings = learning::settings{});

        [[nodiscard]] auto &environment() {
            return _environment;
        }

        std::size_t generate_states(std::vector<state_type> &states);

        std::vector<std::pair<std::size_t, std::size_t>> operator()();

    private:
        Environment &_environment;
        const learning::settings &_settings;

    };

}

#endif //MAP_MATCHING_2_VALUE_ITERATION_PERFORMANCE_HPP
