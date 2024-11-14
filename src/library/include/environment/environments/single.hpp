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

#ifndef MAP_MATCHING_2_ENVIRONMENT_ENVIRONMENTS_SINGLE_HPP
#define MAP_MATCHING_2_ENVIRONMENT_ENVIRONMENTS_SINGLE_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <tuple>

#include <boost/unordered/unordered_flat_map.hpp>

#include "types/matching/algorithm/algorithms.hpp"

#include "geometry/algorithm/distance.hpp"
#include "geometry/algorithm/direction.hpp"
#include "geometry/algorithm/simplify.hpp"
#include "geometry/algorithm/median_merge.hpp"

#include "matching/traits/network.hpp"
#include "matching/settings.hpp"

#include "util/time_helper.hpp"

namespace map_matching_2::environment {

    template<typename Network>
    class single {

        using state_internal = std::vector<std::int64_t>;
        using action_internal = std::int64_t;

    public:
        using network_type = Network;
        using algorithms_type = matching::algorithms<network_type>;
        using track_type = typename matching::network_traits<network_type>::track_type;
        using route_type = typename matching::network_traits<network_type>::route_type;
        using candidate_type = typename matching::network_traits<network_type>::candidate_type;
        using lazy_rich_line_type = typename matching::network_traits<network_type>::lazy_rich_line_type;
        using length_type = typename matching::network_traits<network_type>::length_type;
        using angle_type = typename matching::network_traits<network_type>::angle_type;

        using state_type = std::size_t;
        using action_type = std::size_t;
        using reward_type = double;

        using reward_tuple = std::tuple<state_type, action_type, reward_type, bool>;

        const std::string name = "mdp";

        std::size_t state_size = 3;
        bool intelligent_action = false;

        const std::size_t special_actions = 2;
        const action_internal no_action = -1;
        const action_internal skip = -2;

        state_type state{};

        constexpr single(const network_type &network, algorithms_type &algorithms,
                const matching::settings &settings = {})
            : _network{network}, _settings{settings}, _algorithms{algorithms} {}

        constexpr single(const single &other) = delete;

        constexpr single(single &&other) noexcept
            : _network{other._network}, _settings{other._settings}, _algorithms{other._algorithms},
            _track{std::move(other._track)}, _candidates{std::move(other._candidates)},
            _states{std::move(other._states)},
            _state_map{std::move(other._state_map)},
            _state_cache{std::move(other._state_cache)} {
            other._states.clear();
            other._state_map.clear();
            other._state_cache.clear();
        }

        constexpr single &operator=(const single &other) = delete;

        constexpr single &operator=(single &&other) noexcept = delete;

        constexpr ~single() = default;

        void set(track_type track) {
            _track = std::move(track);

            if (_settings.filter_duplicates) {
                const auto defects = _algorithms.detector.detect(track.rich_line, _settings.filter_duplicates);
                _algorithms.detector.remove_defects(track.rich_line, defects);

                if (abort()) {
                    return;
                }
            }

            if (_settings.simplify_track) {
                geometry::simplify_rich_line(_track.rich_line, false, _settings.simplify_track_distance_tolerance);

                if (abort()) {
                    return;
                }
            }

            if (_settings.median_merge) {
                _track.rich_line = geometry::median_merge(_track.rich_line.line(),
                        _settings.median_merge_distance_tolerance, _settings.adaptive_median_merge, &_network);
            }
        }

        [[nodiscard]] const auto &track() const {
            return _track;
        }

        [[nodiscard]] const auto &candidates() const {
            return _candidates;
        }

        [[nodiscard]] std::size_t states() const {
            std::size_t states = 0;
            for (const auto &state_actions : _state_cache) {
                states += state_actions.size();
            }
            return states;
        }

        [[nodiscard]] std::size_t edges() const {
            std::size_t edges = 0;
            for (const auto &candidate : _candidates) {
                edges += candidate.edges.size();
            }
            return edges;
        }

        void init() {
            _states.clear();
            _state_map.clear();
            _state_cache.clear();

            // state-size in settings is only the points, add the actions which is points - 1
            state_size = _settings.state_size + (_settings.state_size - 1);

            _candidates = _algorithms.searcher.candidate_search(_track, _settings);
        }

        std::vector<action_type> actions() {
            const state_internal &state_int = state2internal(state);
            if (not state_int.empty()) {
                const auto next_position = state_int.back();

                std::size_t action_size = 0;
                if (next_position < _candidates.size()) {
                    action_size = _candidates[next_position].edges.size();
                }

                if (action_size > 0) {
                    std::vector<action_type> actions(action_size);
                    std::iota(actions.begin(), actions.end(), special_actions);
                    if (intelligent_action) {
                        _intelligent_actions(actions);
                    }
                    return actions;
                }
            }

            return std::vector<action_type>{_internal2action(no_action)};
        }

        state_type reset() {
            state = _internal2state(state_internal{0});
            return state;
        }

        reward_tuple step(action_type action) {
            if (state < _state_cache.size()) [[likely]] {
                const auto &state_action_cache = _state_cache[state];
                if (action < state_action_cache.size()) [[likely]] {
                    const auto &result_pair = state_action_cache[action];
                    if (result_pair.first) [[likely]] {
                        const auto &result = result_pair.second;
                        state = std::get<0>(result);
                        return result;
                    }
                }
            }

            const state_internal &state_int = state2internal(state);
            action_internal action_int = action2internal(action);

            state_internal new_state = _new_state(state_int, action_int);
            reward_type reward = _reward(state_int, new_state, action_int);

            state_type next_state = _internal2state(new_state);
            action_type next_action = _internal2action(action_int);

            bool done = _done(new_state);
            reward_tuple result{next_state, next_action, reward, done};
            auto &state_actions = _state_cache[state];
            while (action >= state_actions.size()) {
                state_actions.emplace_back(false, reward_tuple{});
            }
            state_actions[action] = std::pair{true, result};
            state = next_state;
            return result;
        }

        [[nodiscard]] route_type result(
                const std::vector<std::pair<std::size_t, std::size_t>> &policy) const {
            return _algorithms.router.candidates_route(
                    _candidates, policy, _settings.within_edge_turns, _settings.join_merges,
                    _settings.routing_max_distance_factor);
        }

        [[nodiscard]] const state_internal &state2internal(state_type state) const {
            return _states[state];
        }

        [[nodiscard]] action_internal action2internal(action_type action) const {
            if (action == 0) {
                return no_action;
            } else if (action == 1) {
                return skip;
            } else {
                return action - special_actions;
            }
        }

        [[nodiscard]] bool abort() {
            return _algorithms.time_helper.update_and_has_reached();
        }

    private:
        using states_type = std::vector<state_internal>;
        using state_map_type = boost::unordered_flat_map<state_internal, state_type>;
        using state_cache_value_type = std::vector<std::pair<bool, reward_tuple>>;
        using state_cache_type = std::vector<state_cache_value_type>;

        void _intelligent_actions(std::vector<action_type> &actions) {
            std::vector<std::pair<action_type, reward_type>> rewards;
            rewards.reserve(actions.size());

            state_type _state = state;
            for (const action_type action : actions) {
                const auto result = step(action);
                const auto reward = std::get<2>(result);
                rewards.emplace_back(std::pair{action, reward});
                state = _state;
            }

            // reverse sort, so comparator is >= instead of <
            const auto rewards_comparator = [](const auto &left, const auto &right) {
                return left.second > right.second;
            };

            std::sort(rewards.begin(), rewards.end(), rewards_comparator);

            for (std::size_t i = 0; i < actions.size(); ++i) {
                actions[i] = rewards[i].first;
            }
        }

        [[nodiscard]] reward_type
        _reward(const state_internal &state, state_internal &new_state, action_internal &action) {
            reward_type reward{0.0};

            if (not state.empty() and action >= 0) {
                const auto next_position = state.back();

                const auto &next_candidate = _candidates[next_position];
                const auto &next_edge = next_candidate.edges[action];

                if (state.size() < 3) {
                    if (next_position == 0) {
                        // first position should be near
                        reward -= _settings.mdp_distance_factor * (next_edge.distance * 3);
                    } else {
                        // reset after skip is allowed to be further away
                        reward -= _settings.mdp_distance_factor * (next_edge.distance * 0.1);
                    }
                } else {
                    const auto current_position = state[state.size() - 3];
                    const auto current_action = state[state.size() - 2];

                    if (current_action >= 0) {
                        const auto &current_candidate = _candidates[current_position];
                        const auto &current_edge = current_candidate.edges[current_action];

                        const auto route = _algorithms.router.candidate_route(
                                _candidates, current_position, current_action, next_position, action,
                                _settings.within_edge_turns, _settings.routing_max_distance_factor);

                        if (route.is_invalid()) {
                            // no route found from current to next position, fail and skip next position
                            reward -= _fail_reward(current_position);
                            if (_settings.skip_errors) {
                                new_state = _skip_state(state);
                                action = skip;
                            }
                        } else {
                            bool adjacent = current_candidate.index + 1 == next_candidate.index;
                            bool next_equal = (adjacent and current_candidate.next_equal) or
                            (not adjacent and geometry::equals_points(
                                    current_candidate.point(),
                                    next_candidate.point()));
                            if (next_equal and not route.has_length()) {
                                // no distance between candidates and empty route, we stay here
                                reward -= 0.0;
                            } else {
                                // calculate reward
                                reward -= _settings.mdp_distance_factor *
                                        (next_edge.distance + current_edge.distance);

                                if (next_candidate.index + 1 >= _candidates.size()) {
                                    // last candidate, extra distance reward for improving that it stops near goal
                                    reward -= _settings.mdp_distance_factor * (next_edge.distance * 2);
                                }

                                length_type next_distance = geometry::default_float_type<length_type>::v0;
                                if (adjacent) {
                                    if (next_candidate.track->rich_line.has_length(next_candidate.index - 1)) {
                                        next_distance =
                                                next_candidate.track->rich_line.length(next_candidate.index - 1);
                                    }
                                } else {
                                    next_distance = geometry::point_distance(
                                            current_candidate.point(), next_candidate.point());
                                }

                                length_type route_length = geometry::default_float_type<length_type>::v0;
                                if (route.has_length()) {
                                    route_length = route.length();
                                }

                                const auto route_diff = std::fabs(next_distance - route_length);

                                if (route_length > 0.0 and next_distance > 0.0) {
                                    const auto route_factor = route_length / next_distance;
                                    // when route is longer than next distance, it might be bad,
                                    // if it is shorter, it might be good
                                    reward -= _settings.mdp_length_factor * (route_diff * route_factor);

                                    if (_settings.skip_errors and route_factor > 10.0) {
                                        // when route is more than 10 times longer than next distance, skip
                                        new_state = _skip_state(state);
                                        action = skip;
                                    }
                                } else {
                                    reward -= _settings.mdp_length_factor * route_diff;
                                }

                                if (route.has_azimuth()) {
                                    const angle_type route_azimuth = route.azimuth();
                                    angle_type next_azimuth = route_azimuth;
                                    if (adjacent) {
                                        if (next_candidate.track->rich_line.has_azimuth(next_candidate.index - 1)) {
                                            next_azimuth =
                                                    next_candidate.track->rich_line.azimuth(next_candidate.index - 1);
                                        }
                                    } else {
                                        if (not geometry::equals_points(
                                                current_candidate.point(), next_candidate.point())) {
                                            next_azimuth = geometry::azimuth_deg(
                                                    current_candidate.point(), next_candidate.point());
                                        }
                                    }

                                    const auto azimuth_diff = std::fabs(geometry::angle_diff(
                                            next_azimuth, route_azimuth));
                                    reward -= _settings.mdp_azimuth_factor * azimuth_diff;

                                    // calculate turn
                                    if (current_candidate.index >= 1 and next_candidate.index >= 2 and
                                        current_candidate.track->rich_line.has_azimuth(current_candidate.index - 1)) {
                                        const angle_type current_azimuth =
                                                current_candidate.track->rich_line.azimuth(current_candidate.index - 1);

                                        angle_type next_turn = std::numeric_limits<angle_type>::quiet_NaN();
                                        if (adjacent) {
                                            if (next_candidate.track->rich_line.has_direction(
                                                    next_candidate.index - 2)) {
                                                next_turn = next_candidate.track->rich_line.direction(
                                                        next_candidate.index - 2);
                                            }
                                        } else {
                                            next_turn = geometry::direction_deg(current_azimuth, next_azimuth);
                                        }

                                        if (not std::isnan(next_turn)) {
                                            const auto route_turn = geometry::direction_deg(
                                                    current_azimuth, route_azimuth);
                                            auto turn_diff = std::fabs(geometry::angle_diff(next_turn, route_turn));
                                            reward -= _settings.mdp_turn_factor * turn_diff;
                                        }
                                    }
                                }

                                if (route.has_directions()) {
                                    reward -= _settings.mdp_direction_factor * route.absolute_directions();
                                }
                            }

                            // when state_size is 5, calculate additional rewards
                            if (state.size() > 3) {
                                const auto prev_position = state[state.size() - 5];
                                const auto prev_action = state[state.size() - 4];

                                if (prev_action >= 0) {
                                    const auto &prev_candidate = _candidates[prev_position];
                                    const auto &prev_edge = current_candidate.edges[prev_action];

                                    const auto prev_route = _algorithms.router.candidate_route(
                                            _candidates, prev_position, prev_action, current_position, current_action,
                                            _settings.within_edge_turns,
                                            _settings.routing_max_distance_factor);

                                    if (not prev_route.is_invalid()) {
                                        // only continue if previous route is valid, should generally be the case

                                        if (not prev_route.attaches(route)) {
                                            // when routes do not attach, fail to done state
                                            reward -= _fail_reward(current_position);
                                            new_state = _done_state(state);
                                        } else {
                                            // routes attach, calculate additional metrics
                                            if (prev_route.has_azimuth() and route.has_azimuth()) {
                                                // calculate direction difference
                                                bool prev_adjacent =
                                                        prev_candidate.index + 1 == current_candidate.index;

                                                const angle_type route_direction = geometry::angle_diff(
                                                        prev_route.azimuth(), route.azimuth());
                                                angle_type candidates_direction =
                                                        std::numeric_limits<angle_type>::quiet_NaN();
                                                if (prev_adjacent and adjacent) {
                                                    if (next_candidate.track->rich_line.has_direction(
                                                            next_candidate.index - 2)) {
                                                        candidates_direction =
                                                                next_candidate.track->rich_line.direction(
                                                                        next_candidate.index - 2);
                                                    }
                                                } else {
                                                    if (not geometry::equals_points(
                                                                prev_candidate.point(), current_candidate.point()) and
                                                        not geometry::equals_points(
                                                                current_candidate.point(), next_candidate.point()))
                                                        candidates_direction = geometry::direction_deg(
                                                                prev_candidate.point(),
                                                                current_candidate.point(),
                                                                next_candidate.point());
                                                }

                                                if (not std::isnan(candidates_direction)) {
                                                    reward -= _settings.mdp_turn_factor * std::fabs(
                                                            geometry::angle_diff(candidates_direction,
                                                                    route_direction));
                                                }
                                            }

                                            if (prev_route.has_length() and route.has_length()) {
                                                // calculate zig-zag segments
                                                const auto return_line = prev_route.
                                                        template extract_return_line<lazy_rich_line_type>(route);
                                                if (return_line.has_length()) {
                                                    reward -= _settings.mdp_length_factor
                                                            * (return_line.length() * 2);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return reward;
        }

        [[nodiscard]] bool _done(const state_internal &done_state) const {
            return done_state.back() >= _candidates.size();
        }

        [[nodiscard]] state_internal _new_state(const state_internal &state, const action_internal &action) {
            state_internal new_state;
            new_state.reserve(state.size() + 2 > state_size ? state.size() : state.size() + 2);
            new_state.insert(new_state.end(),
                    state.size() >= state_size ? state.begin() + 2 : state.begin(), state.end());
            new_state.emplace_back(action);
            new_state.emplace_back(state.back() + 1);
            return new_state;
        }

        [[nodiscard]] static state_internal _skip_state(state_internal skippable_state) {
            if (skippable_state.size() >= 3) {
                skippable_state.erase(skippable_state.begin(), std::next(skippable_state.begin(), 2));
            } else if (not skippable_state.empty()) {
                skippable_state.back()++;
            }
            return skippable_state;
        }

        [[nodiscard]] state_internal _done_state(state_internal done_state) {
            if (!done_state.empty()) {
                done_state.back() = _candidates.size();
            }
            return done_state;
        }

        [[nodiscard]] reward_type _fail_reward(const std::size_t current_position) const {
            reward_type remaining_length = geometry::default_float_type<reward_type>::v0;
            for (std::size_t i = current_position; i + 1 < _candidates.size(); ++i) {
                if (_candidates[i].track->rich_line.has_length(_candidates[i].index)) {
                    remaining_length += _candidates[i].track->rich_line.length(_candidates[i].index);
                }
            }
            return remaining_length;
        }

        [[nodiscard]] state_type _internal2state(const state_internal &state) {
            auto search = _state_map.find(state);
            if (search != _state_map.cend()) {
                return search->second;
            } else {
                state_type index = _states.size();
                _state_map.emplace(state, index);
                _states.emplace_back(state);

                std::size_t action_size = 1; // for no-action
                if (not state.empty()) {
                    const auto next_position = state.back();
                    if (next_position < this->_candidates.size()) {
                        action_size = this->_candidates[next_position].edges.size() + this->special_actions;
                    }
                }

                _state_cache.emplace_back(state_cache_value_type(action_size));
                return index;
            }
        }

        [[nodiscard]] action_type _internal2action(const action_internal &action) const {
            if (action == this->no_action) {
                return 0;
            } else if (action == this->skip) {
                return 1;
            } else {
                return action + this->special_actions;
            }
        }

        const network_type &_network;
        const matching::settings &_settings;
        matching::algorithms<network_type> &_algorithms;

        track_type _track;
        std::vector<candidate_type> _candidates;

        states_type _states{};
        state_map_type _state_map{};
        state_cache_type _state_cache{};

    };

}

#endif //MAP_MATCHING_2_ENVIRONMENT_ENVIRONMENTS_SINGLE_HPP
