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

#ifndef MAP_MATCHING_2_SINGLE_PERFORMANCE_HPP
#define MAP_MATCHING_2_SINGLE_PERFORMANCE_HPP

#include "single.hpp"

namespace map_matching_2::environment {

    template<typename Matcher>
    class single_performance : public single<Matcher> {

    public:
        using base_type = single<Matcher>;
        using matcher_type = Matcher;
        using track_type = typename base_type::track_type;

        using state_type = std::size_t;
        using action_type = std::size_t;
        using state_internal = typename base_type::state_type;
        using action_internal = typename base_type::action_type;
        using reward_type = typename base_type::reward_type;

        using reward_tuple = std::tuple<state_type, action_type, reward_type, bool>;

        static constexpr bool performance = true;
        const std::string name = "mdp";

        state_type state;

        explicit single_performance(matcher_type &matcher,
                                    const matching::settings &match_settings = matching::settings{});

        ~single_performance() = default;

        auto states() const {
            std::size_t states = 0;
            for (const auto &state_actions: _state_cache) {
                states += state_actions.size();
            }
            return states;
        }

        void init();

        std::vector<action_type> actions();

        state_type reset();

        reward_tuple step(action_type action);

        [[nodiscard]] const state_internal &state2internal(const state_type state) const;

        [[nodiscard]] action_internal action2internal(const action_type action) const;

    protected:
        [[nodiscard]] std::vector<action_type> _actions(const state_type &state);

        void _intelligent_actions(std::vector<action_type> &actions);

        [[nodiscard]] state_type _reset(const state_type &state);

        [[nodiscard]] reward_tuple _step(state_type &state, action_type action);

    private:
        std::vector<state_internal> _states;
        std::unordered_map<state_internal, state_type, boost::hash<state_internal>> _state_map;
        std::vector<std::vector<std::pair<bool, reward_tuple>>> _state_cache;

        [[nodiscard]] state_type _internal2state(const state_internal &state);

        [[nodiscard]] action_type _internal2action(const action_internal &action) const;

    };

}

#endif //MAP_MATCHING_2_SINGLE_PERFORMANCE_HPP
