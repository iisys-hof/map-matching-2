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

#ifndef MAP_MATCHING_2_CANDIDATE_EDGE_HPP
#define MAP_MATCHING_2_CANDIDATE_EDGE_HPP

#include <geometry/network/network.hpp>

namespace map_matching_2::matching {

    template<typename Network>
    struct candidate_edge {

        using distance_type = typename boost::geometry::default_distance_result<typename Network::point_type, typename Network::point_type>::type;
        using edge_descriptor_type = typename boost::graph_traits<typename Network::graph_type>::edge_descriptor;
        using point_type = typename Network::point_type;
        using rich_line_type = typename Network::rich_line_type;

        bool adopted;
        std::size_t round;
        edge_descriptor_type edge_descriptor;
        point_type projection_point;
        distance_type distance;
        rich_line_type from;
        rich_line_type to;

        candidate_edge(bool adopted, std::size_t round, edge_descriptor_type edge_descriptor,
                       point_type projection_point, distance_type distance, rich_line_type from, rich_line_type to);

        ~candidate_edge() = default;

        candidate_edge(const candidate_edge &other) = default;

        candidate_edge(candidate_edge &&other) noexcept = default;

        candidate_edge &operator=(const candidate_edge &other) = default;

        candidate_edge &operator=(candidate_edge &&other) noexcept = default;

        [[nodiscard]] static bool distance_comparator(const candidate_edge &left, const candidate_edge &right);

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::string> row() const;

        [[nodiscard]] std::string str() const;

        template<typename NetworkT>
        friend bool operator==(const candidate_edge<NetworkT> &left, const candidate_edge<NetworkT> &right);

        template<typename NetworkT>
        friend bool operator!=(const candidate_edge<NetworkT> &left, const candidate_edge<NetworkT> &right);

        template<typename NetworkT>
        friend std::ostream &operator<<(std::ostream &out, const candidate_edge<NetworkT> &candidate_edge);

    };

}

#endif //MAP_MATCHING_2_CANDIDATE_EDGE_HPP
