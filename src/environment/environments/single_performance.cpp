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

#include "single_performance.hpp"

#include "single.cpp"

namespace map_matching_2::environment {

    template<typename Matcher>
    single_performance<Matcher>::single_performance(matcher_type &matcher, const matching::settings &match_settings)
            : base_type{matcher, match_settings} {}

    template<typename Matcher>
    void single_performance<Matcher>::init() {
        decltype(_states) states;
        _states.swap(states);

        decltype(_state_map) state_map;
        _state_map.swap(state_map);

        decltype(_state_cache) state_cache;
        _state_cache.swap(state_cache);

        this->_init();
    }

    template<typename Matcher>
    std::vector<typename single_performance<Matcher>::action_type> single_performance<Matcher>::actions() {
        return _actions(this->state);
    }

    template<typename Matcher>
    typename single_performance<Matcher>::state_type single_performance<Matcher>::reset() {
        return _reset(_internal2state(state_internal{0}));
    }

    template<typename Matcher>
    typename single_performance<Matcher>::reward_tuple
    single_performance<Matcher>::step(action_type action) {
        return _step(this->state, action);
    }

    template<typename Matcher>
    const typename single_performance<Matcher>::state_internal &
    single_performance<Matcher>::state2internal(const state_type state) const {
        return _states[state];
    }

    template<typename Matcher>
    typename single_performance<Matcher>::action_internal
    single_performance<Matcher>::action2internal(const action_type action) const {
        if (action == 0) {
            return this->no_action;
        } else if (action == 1) {
            return this->skip;
        } else {
            return action - this->special_actions;
        }
    }

    template<typename Matcher>
    std::vector<typename single_performance<Matcher>::action_type>
    single_performance<Matcher>::_actions(const state_type &state) {
        const state_internal &state_int = state2internal(state);
        if (state_int.size() > 0) {
            const auto next_position = state_int.back();

            std::size_t action_size = 0;
            if (next_position < this->_candidates.size()) {
                action_size = this->_candidates[next_position].edges.size();
            }

            if (action_size > 0) {
                std::vector<action_type> actions(action_size);
                std::iota(actions.begin(), actions.end(), this->special_actions);
                if (this->intelligent_action) {
                    _intelligent_actions(actions);
                }
                return actions;
            }
        }

        return std::vector<action_type>{_internal2action(this->no_action)};
    }

    template<typename Matcher>
    void single_performance<Matcher>::_intelligent_actions(std::vector<action_type> &actions) {
        std::vector<std::pair<action_type, reward_type>> rewards;
        rewards.reserve(actions.size());

        for (const action_type action: actions) {
            state_type _state = this->state;
            const auto result = _step(_state, action);
            const auto reward = std::get<2>(result);
            rewards.emplace_back(std::pair{action, reward});
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
    typename single_performance<Matcher>::state_type single_performance<Matcher>::_reset(const state_type &state) {
        this->state = state;
        return this->state;
    }

    template<typename Matcher>
    typename single_performance<Matcher>::reward_tuple
    single_performance<Matcher>::_step(state_type &state, action_type action) {
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

        state_internal new_state = this->_new_state(state_int, action_int);
        reward_type reward = this->_reward(state_int, new_state, action_int);

        state_type next_state = _internal2state(new_state);
        action_type next_action = _internal2action(action_int);

        bool done = this->_done(new_state);
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
    typename single_performance<Matcher>::state_type
    single_performance<Matcher>::_internal2state(const state_internal &state) {
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
    typename single_performance<Matcher>::action_type single_performance<Matcher>::_internal2action(
            const action_internal &action) const {
        if (action == this->no_action) {
            return 0;
        } else if (action == this->skip) {
            return 1;
        } else {
            return action + this->special_actions;
        }
    }

    template
    class single_performance<matching::types_geographic::matcher_static>;

    template
    class single_performance<matching::types_cartesian::matcher_static>;

}
