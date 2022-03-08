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

#ifndef MAP_MATCHING_2_SINGLE_HPP
#define MAP_MATCHING_2_SINGLE_HPP

#include <cstdint>
#include <unordered_map>

#include <boost/functional/hash.hpp>

#include <matching/settings.hpp>
#include <matching/detector.hpp>

namespace map_matching_2::environment {

    template<typename Matcher>
    class single {

    public:
        using matcher_type = Matcher;
        using track_type = typename matcher_type::track_type;

        using state_type = std::size_t;
        using action_type = std::size_t;
        using state_internal = std::vector<std::int64_t>;
        using action_internal = std::int64_t;
        using reward_type = double;

        using reward_tuple = std::tuple<state_type, action_type, reward_type, bool>;

        const std::string name = "mdp";

        std::size_t state_size = 3;
        bool intelligent_action = false;

        const std::size_t special_actions = 2;
        const std::int64_t no_action = -1;
        const std::int64_t skip = -2;

        state_type state;

        explicit single(matcher_type &matcher, const matching::settings &match_settings = matching::settings{});

        ~single() = default;

        void set(const track_type &track);

        const auto &track() const {
            return _track;
        }

        const auto &candidates() const {
            return _candidates;
        }

        auto states() const {
            std::size_t states = 0;
            for (const auto &state_actions: _state_cache) {
                states += state_actions.size();
            }
            return states;
        }

        auto edges() const {
            std::size_t edges = 0;
            for (const auto &candidate: _candidates) {
                edges += candidate.edges.size();
            }
            return edges;
        }

        void init();

        std::vector<action_type> actions();

        state_type reset();

        reward_tuple step(action_type action);

        template<typename Result>
        [[nodiscard]] Result result(const std::vector<std::pair<std::size_t, std::size_t>> &policy) const {
            return Result{_matcher.candidates_route(
                    _candidates, policy, _match_settings.export_edges, _match_settings.join_merges)};
        }

        void resize_candidates(const std::vector<std::size_t> &positions, std::size_t round,
                               bool adaptive_resize = true);

        [[nodiscard]] const state_internal &state2internal(const state_type state) const;

        [[nodiscard]] action_internal action2internal(const action_type action) const;

    private:
        void _intelligent_actions(std::vector<action_type> &actions);

        [[nodiscard]] reward_type
        _reward(const state_internal &state, state_internal &new_state, action_internal &action);

        [[nodiscard]] bool _done(const state_internal &done_state) const;

        [[nodiscard]] state_internal _new_state(const state_internal &state, const action_internal &action) const;

        [[nodiscard]] state_internal _skip_state(state_internal skippable_state);

        [[nodiscard]] state_internal _done_state(state_internal done_state);

        [[nodiscard]] reward_type _fail_reward(std::size_t current_position) const;

        [[nodiscard]] state_type _internal2state(const state_internal &state);

        [[nodiscard]] action_type _internal2action(const action_internal &action) const;

        matcher_type &_matcher;
        const matching::settings &_match_settings;

        track_type _track;

        std::vector<typename matcher_type::candidate_type> _candidates;

        std::vector<state_internal> _states;
        std::unordered_map<state_internal, state_type, boost::hash<state_internal>> _state_map;
        std::vector<std::vector<std::pair<bool, reward_tuple>>> _state_cache;

    };

}

#endif //MAP_MATCHING_2_SINGLE_HPP
