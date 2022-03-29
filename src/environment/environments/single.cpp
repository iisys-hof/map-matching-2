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

#include "single.hpp"

#include <numeric>

#include <geometry/util.hpp>
#include <matching/types.hpp>

namespace map_matching_2::environment {

    template<typename Matcher>
    single<Matcher>::single(Matcher &matcher, const matching::settings &match_settings)
            : _matcher{matcher}, _match_settings{match_settings} {}

    template<typename Matcher>
    void single<Matcher>::set(const track_type &track) {
        if (_match_settings.filter_duplicates or _match_settings.filter_defects) {
            auto defects = _matcher.detect(track, _match_settings.filter_duplicates, _match_settings.filter_defects);
            _track = _matcher.remove_defects(track, defects);
        } else {
            _track = track;
        }

        if (_match_settings.simplify) {
            _track.simplify(false, _match_settings.simplify_distance_tolerance);
        }

        if (_match_settings.median_merge) {
            _track.median_merge(_match_settings.median_merge_distance_tolerance, _match_settings.adaptive_median_merge,
                                &_matcher.network);
        }
    }

    template<typename Matcher>
    void single<Matcher>::init() {
        decltype(_states) states;
        _states.swap(states);

        decltype(_state_map) state_map;
        _state_map.swap(state_map);

        decltype(_state_cache) state_cache;
        _state_cache.swap(state_cache);

        // state-size in settings is only the points, add the actions which is points - 1
        state_size = _match_settings.state_size + (_match_settings.state_size - 1);

        if (_match_settings.k_nearest_candidate_search) {
            _candidates = _matcher.candidate_search_nearest(
                    _track, _match_settings.k_nearest, _match_settings.k_nearest_reverse,
                    _match_settings.k_nearest_adjacent, _match_settings.candidate_adoption_siblings,
                    _match_settings.candidate_adoption_nearby, _match_settings.candidate_adoption_reverse);
        } else {
            _candidates = _matcher.candidate_search_buffer(
                    _track, _match_settings.buffer_points, _match_settings.buffer_radius,
                    _match_settings.buffer_upper_radius, _match_settings.buffer_lower_radius,
                    _match_settings.adaptive_radius, _match_settings.candidate_adoption_siblings,
                    _match_settings.candidate_adoption_nearby, _match_settings.candidate_adoption_reverse);
        }
    }

    template<typename Matcher>
    std::vector<typename single<Matcher>::action_type> single<Matcher>::actions() {
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

    template<typename Matcher>
    typename single<Matcher>::state_type single<Matcher>::reset() {
        state = _internal2state(state_internal{0});
        return state;
    }

    template<typename Matcher>
    typename single<Matcher>::reward_tuple single<Matcher>::step(action_type action) {
        if (state < _state_cache.size()) {
            const auto &state_action_cache = _state_cache[state];
            if (action < state_action_cache.size()) {
                const auto &result_pair = state_action_cache[action];
                if (result_pair.first) {
                    const auto &result = result_pair.second;
                    state = std::get<0>(result);
                    return result;
                }
            }
        }

        state_internal state_int = state2internal(state);
        action_internal action_int = action2internal(action);

        state_internal new_state = _new_state(state_int, action_int);
        reward_type reward = _reward(state_int, new_state, action_int);

        state_type next_state = _internal2state(new_state);
        action_type next_action = _internal2action(action_int);

        bool done = _done(new_state);
        reward_tuple result{next_state, next_action, reward, done};
        auto &state_actions = _state_cache[state];
        while (action >= state_actions.size()) {
            state_actions.emplace_back(std::pair{true, reward_tuple{}});
        }
        state_actions[action] = std::pair{true, result};
        state = next_state;
        return result;
    }

    template<typename Matcher>
    void single<Matcher>::resize_candidates(const std::vector<std::size_t> &positions, const std::size_t round,
                                            const bool adaptive_resize) {
        if (_match_settings.k_nearest_candidate_search) {
            _matcher.resize_candidates_nearest(
                    _track, _candidates, positions, round, _match_settings.k_nearest_reverse,
                    _match_settings.k_nearest_adjacent, _match_settings.candidate_adoption_siblings,
                    _match_settings.candidate_adoption_nearby, _match_settings.candidate_adoption_reverse);
        } else {
            _matcher.resize_candidates_buffer(
                    _track, _candidates, positions, round, _match_settings.buffer_points,
                    _match_settings.buffer_upper_radius, _match_settings.buffer_lower_radius,
                    _match_settings.adaptive_radius, adaptive_resize, _match_settings.candidate_adoption_siblings,
                    _match_settings.candidate_adoption_nearby, _match_settings.candidate_adoption_reverse);
        }
    }

    template<typename Matcher>
    const typename single<Matcher>::state_internal &
    single<Matcher>::state2internal(const state_type state) const {
        return _states[state];
    }

    template<typename Matcher>
    typename single<Matcher>::action_internal
    single<Matcher>::action2internal(const action_type action) const {
        if (action == 0) {
            return no_action;
        } else if (action == 1) {
            return skip;
        } else {
            return action - special_actions;
        }
    }

    template<typename Matcher>
    void single<Matcher>::_intelligent_actions(std::vector<action_type> &actions) {
        std::vector<std::pair<action_type, reward_type>> rewards;
        rewards.reserve(actions.size());

        state_type _state = state;
        for (const action_type action: actions) {
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

    template<typename Matcher>
    typename single<Matcher>::reward_type single<Matcher>::_reward(
            const state_internal &state, state_internal &new_state, action_internal &action) {
        reward_type reward{0.0};

        if (state.size() >= 1 and action >= 0) {
            const auto next_position = state.back();

            const auto &next_candidate = _candidates[next_position];
            const auto &next_edge = next_candidate.edges[action];

            if (state.size() < 3) {
                if (next_position == 0) {
                    // first position should be near
                    reward -= _match_settings.mdp_distance_factor * (next_edge.distance * 3);
                } else {
                    // reset after skip is allowed to be further away
                    reward -= _match_settings.mdp_distance_factor * (next_edge.distance * 0.1);
                }
            } else {
                const auto current_position = state[state.size() - 3];
                const auto current_action = state[state.size() - 2];

                if (current_action >= 0) {
                    const auto &current_candidate = _candidates[current_position];
                    const auto &current_edge = current_candidate.edges[current_action];

                    const auto route = _matcher.candidate_route(
                            _candidates, current_position, current_action, next_position, action,
                            _match_settings.routing_max_distance_factor);

                    if (route.is_invalid) {
                        // no route found from current to next position, fail and skip next position
                        reward -= _fail_reward(current_position);
                        if (_match_settings.skip_errors) {
                            new_state = _skip_state(state);
                            action = skip;
                        }
                    } else {
                        bool adjacent = current_candidate.index + 1 == next_candidate.index;
                        bool next_equal = (adjacent and current_candidate.next_equal) or
                                          (not adjacent and geometry::equals_points(
                                                  current_candidate.measurement->point,
                                                  next_candidate.measurement->point));
                        if (next_equal and not route.has_length) {
                            // no distance between candidates and empty route, we stay here
                            reward -= 0.0;
                        } else {
                            // calculate reward
                            reward -= _match_settings.mdp_distance_factor *
                                      (next_edge.distance + current_edge.distance);

                            if (next_candidate.index + 1 >= _candidates.size()) {
                                // last candidate, extra distance reward for improving that it stops near goal
                                reward -= _match_settings.mdp_distance_factor * (next_edge.distance * 2);
                            }

                            const typename Matcher::length_type next_distance =
                                    adjacent
                                    ? next_candidate.track->rich_segments[next_candidate.index - 1].length
                                    : geometry::point_distance(current_candidate.measurement->point,
                                                               next_candidate.measurement->point);

                            typename Matcher::length_type route_length =
                                    geometry::default_float_type<typename Matcher::length_type>::v0;
                            if (route.has_length) {
                                route_length = route.length;
                            }

                            const auto route_diff = std::fabs(next_distance - route_length);

                            if (route_length > 0.0 and next_distance > 0.0) {
                                const auto route_factor = route_length / next_distance;
                                // when route is longer than next distance, it might be bad,
                                // if it is shorter, it might be good
                                reward -= _match_settings.mdp_length_factor * (route_diff * route_factor);

                                if (_match_settings.skip_errors and route_factor > 10.0) {
                                    // when route is more than 10 times longer than next distance, skip
                                    new_state = _skip_state(state);
                                    action = skip;
                                }
                            } else {
                                reward -= _match_settings.mdp_length_factor * route_diff;
                            }

                            if (route.has_azimuth) {
                                const typename Matcher::coordinate_type next_azimuth =
                                        adjacent
                                        ? next_candidate.track->rich_segments[next_candidate.index - 1].azimuth
                                        : geometry::azimuth_deg(current_candidate.measurement->point,
                                                                next_candidate.measurement->point);

                                const auto azimuth_diff = std::fabs(geometry::angle_diff(next_azimuth, route.azimuth));
                                reward -= _match_settings.mdp_azimuth_factor * azimuth_diff;

                                // calculate turn
                                if (current_candidate.index >= 1 and next_candidate.index >= 2) {
                                    typename Matcher::coordinate_type current_azimuth =
                                            current_candidate.track->rich_segments[current_candidate.index - 1].azimuth;

                                    const typename Matcher::coordinate_type next_turn =
                                            adjacent
                                            ? next_candidate.track->segments_directions[next_candidate.index - 2]
                                            : geometry::direction_deg(current_azimuth, next_azimuth);

                                    const auto route_turn = geometry::direction_deg(current_azimuth, route.azimuth);
                                    auto turn_diff = std::fabs(geometry::angle_diff(next_turn, route_turn));
                                    reward -= _match_settings.mdp_turn_factor * turn_diff;
                                }
                            }

                            if (route.has_directions) {
                                reward -= _match_settings.mdp_direction_factor * route.absolute_directions;
                            }
                        }

                        // when state_size is 5, calculate additional rewards
                        if (state.size() > 3) {
                            const auto prev_position = state[state.size() - 5];
                            const auto prev_action = state[state.size() - 4];

                            if (prev_action >= 0) {
                                const auto &prev_candidate = _candidates[prev_position];
                                const auto &prev_edge = current_candidate.edges[prev_action];

                                const auto prev_route = _matcher.candidate_route(
                                        _candidates, prev_position, prev_action, current_position, current_action,
                                        _match_settings.routing_max_distance_factor);

                                if (not prev_route.is_invalid) {
                                    // only continue if previous route is valid, should generally be the case

                                    if (not prev_route.attaches(route)) {
                                        // when routes do not attach, fail to done state
                                        reward -= _fail_reward(current_position);
                                        new_state = _done_state(state);
                                    } else {
                                        // routes attach, calculate additional metrics
                                        if (prev_route.has_azimuth and route.has_azimuth) {
                                            // calculate direction difference
                                            bool prev_adjacent = prev_candidate.index + 1 == current_candidate.index;

                                            const auto route_direction = geometry::angle_diff(
                                                    prev_route.azimuth, route.azimuth);
                                            const auto candidates_direction =
                                                    prev_adjacent and adjacent
                                                    ? next_candidate.track->segments_directions[
                                                            next_candidate.index - 2]
                                                    : geometry::direction_deg(prev_candidate.measurement->point,
                                                                              current_candidate.measurement->point,
                                                                              next_candidate.measurement->point);

                                            reward -= _match_settings.mdp_turn_factor * std::fabs(
                                                    geometry::angle_diff(candidates_direction, route_direction));
                                        }

                                        if (prev_route.has_length and route.has_length) {
                                            // calculate zig-zag segments
                                            const auto return_line = prev_route.extract_return_line(route);
                                            if (return_line.has_length) {
                                                reward -= _match_settings.mdp_length_factor * (return_line.length * 2);
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

    template<typename Matcher>
    bool single<Matcher>::_done(const state_internal &done_state) const {
        return done_state.back() >= _candidates.size();
    }

    template<typename Matcher>
    typename single<Matcher>::state_internal single<Matcher>::_new_state(
            const state_internal &state, const action_internal &action) const {
        state_internal new_state;
        new_state.reserve(state.size() + 2 > state_size ? state.size() : state.size() + 2);
        new_state.insert(new_state.end(), state.size() >= state_size ? state.begin() + 2 : state.begin(), state.end());
        new_state.emplace_back(action);
        new_state.emplace_back(state.back() + 1);
        return new_state;
    }

    template<typename Matcher>
    typename single<Matcher>::state_internal single<Matcher>::_skip_state(state_internal skippable_state) {
        if (skippable_state.size() >= 3) {
            skippable_state.erase(skippable_state.begin(), std::next(skippable_state.begin(), 2));
        } else if (not skippable_state.empty()) {
            skippable_state.back()++;
        }
        return skippable_state;
    }

    template<typename Matcher>
    typename single<Matcher>::state_internal single<Matcher>::_done_state(state_internal done_state) {
        if (!done_state.empty()) {
            done_state.back() = _candidates.size();
        }
        return done_state;
    }

    template<typename Matcher>
    typename single<Matcher>::reward_type single<Matcher>::_fail_reward(std::size_t current_position) const {
        reward_type remaining_length = geometry::default_float_type<reward_type>::v0;
        for (std::size_t i = current_position; i + 1 < _candidates.size(); ++i) {
            remaining_length += _candidates[i].track->rich_segments[_candidates[i].index].length;
        }
        return remaining_length;
    }

    template<typename Matcher>
    typename single<Matcher>::state_type
    single<Matcher>::_internal2state(const state_internal &state) {
        auto search = _state_map.find(state);
        if (search != _state_map.cend()) {
            return search->second;
        } else {
            state_type index = _states.size();
            _state_map.emplace(state, index);
            _states.emplace_back(state);

            std::size_t action_size = 1; // for no-action
            if (state.size() > 0) {
                const auto next_position = state.back();
                if (next_position < this->_candidates.size()) {
                    action_size = this->_candidates[next_position].edges.size() + this->special_actions;
                }
            }

            std::vector<std::pair<bool, reward_tuple>> results(action_size);
            _state_cache.emplace_back(std::move(results));
            return index;
        }
    }

    template<typename Matcher>
    typename single<Matcher>::action_type single<Matcher>::_internal2action(const action_internal &action) const {
        if (action == this->no_action) {
            return 0;
        } else if (action == this->skip) {
            return 1;
        } else {
            return action + this->special_actions;
        }
    }

    template
    class single<matching::types_geographic::matcher_static>;

    template
    class single<matching::types_cartesian::matcher_static>;

}
