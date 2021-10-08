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

#ifndef MAP_MATCHING_2_CANDIDATE_HPP
#define MAP_MATCHING_2_CANDIDATE_HPP

#include "candidate_node.hpp"
#include "candidate_edge.hpp"

namespace map_matching_2::matching {

    template<typename Network, typename Track>
    struct candidate {
        using measurement_type = typename Track::measurement_type;
        using candidate_node_type = candidate_node<Network>;
        using candidate_edge_type = candidate_edge<Network>;

        const Track *track;
        std::size_t index;
        const measurement_type *measurement;
        double radius;
        std::size_t number;
        bool next_equal;
        std::vector<candidate_node_type> nodes;
        std::vector<candidate_edge_type> edges;

        candidate(const Track &track, std::size_t index, bool next_equal,
                  std::vector<candidate_node_type> nodes, std::vector<candidate_edge_type> edges);

        ~candidate() = default;

        candidate(const candidate &other) = default;

        candidate(candidate &&other) noexcept = default;

        candidate &operator=(const candidate &other) = default;

        candidate &operator=(candidate &&other) noexcept = default;

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::vector<std::string>> rows() const;

        [[nodiscard]] std::string str() const;

        template<typename NetworkT, typename TrackT>
        friend std::ostream &operator<<(std::ostream &out, const candidate<NetworkT, TrackT> &candidate);

    };

}

#endif //MAP_MATCHING_2_CANDIDATE_HPP
