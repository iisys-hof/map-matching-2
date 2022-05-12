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

#ifndef MAP_MATCHING_2_HMM_HPP
#define MAP_MATCHING_2_HMM_HPP

#include <cstdint>

#include <matching/settings.hpp>
#include <matching/detector.hpp>

namespace map_matching_2::environment {

    template<typename Matcher>
    class hmm {
    public:
        using matcher_type = Matcher;
        using track_type = typename matcher_type::track_type;

        using action_type = std::int64_t;

        const std::int64_t no_action = -1;

        static constexpr bool performance = false;
        const std::string name = "hmm";

        explicit hmm(matcher_type &matcher, const matching::settings &match_settings);

        void set(const track_type &track);

        const auto &track() const {
            return _track;
        }

        const auto &candidates() const {
            return _candidates;
        }

        auto states() const {
            std::size_t states = 0;
            for (std::int64_t from = -1, to = 0; to < _candidates.size(); ++from, ++to) {
                if (from < 0) {
                    states += _transitions[to].size();
                } else {
                    states += _transitions[to].size() * _transitions[from].size();
                }
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

        const auto &observations() const {
            return _observations;
        }

        const auto &transitions() const {
            return _transitions;
        }

        template<typename Result>
        [[nodiscard]] Result result(const std::vector<std::pair<std::size_t, std::size_t>> &policy) const {
            return Result{_matcher.candidates_route(
                    _candidates, policy, _match_settings.within_edge_turns, _match_settings.export_edges,
                    _match_settings.join_merges, _match_settings.routing_max_distance_factor)};
        }

    private:
        matcher_type &_matcher;
        const matching::settings &_match_settings;

        track_type _track;

        std::vector<typename matcher_type::candidate_type> _candidates;

        std::vector<std::vector<double>> _observations;
        std::vector<std::vector<std::vector<double>>> _transitions;

    };

}

#endif //MAP_MATCHING_2_HMM_HPP
