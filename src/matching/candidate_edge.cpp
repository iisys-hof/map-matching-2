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

#include "candidate_edge.hpp"

#include <geometry/network/types.hpp>
#include <geometry/util.hpp>

namespace map_matching_2::matching {

    template<typename Network>
    candidate_edge<Network>::candidate_edge(
            bool adopted, std::size_t round, edge_descriptor_type edge_descriptor, point_type projection_point,
            distance_type distance, rich_line_type from, rich_line_type to)
            : adopted{adopted}, round{round}, edge_descriptor{edge_descriptor},
              projection_point{std::move(projection_point)}, distance{distance}, from{std::move(from)},
              to{std::move(to)} {}

    template<typename Network>
    bool candidate_edge<Network>::distance_comparator(const candidate_edge &left, const candidate_edge &right) {
        return left.round < right.round or (left.round == right.round and left.distance < right.distance);
    }

    template<typename Network>
    std::vector<std::string> candidate_edge<Network>::header() const {
        return {"distance", "projection", "from", "to"};
    }

    template<typename Network>
    std::vector<std::string> candidate_edge<Network>::row() const {
        std::vector<std::string> row;
        row.reserve(4);
        row.emplace_back(std::to_string(distance));
        row.emplace_back(geometry::to_wkt(projection_point));
        row.emplace_back(geometry::to_wkt(from.line));
        row.emplace_back(geometry::to_wkt(to.line));
        return row;
    }

    template<typename Network>
    std::string candidate_edge<Network>::str() const {
        const auto &fields = row();
        std::string str = "CandidateEdge(";
        bool first = true;
        for (const auto &field: fields) {
            if (!first) {
                str.append(",");
            }
            first = false;
            str.append(field);
        }
        str.append(")");
        return str;
    }

    template<typename Network>
    bool operator==(const candidate_edge<Network> &left, const candidate_edge<Network> &right) {
        return left.edge_descriptor == right.edge_descriptor and
               geometry::equals_points(left.projection_point, right.projection_point);
    }

    template<typename Network>
    bool operator!=(const candidate_edge<Network> &left, const candidate_edge<Network> &right) {
        return not(left == right);
    }

    template<typename Network>
    std::ostream &operator<<(std::ostream &out, const candidate_edge<Network> &candidate_edge) {
        out << candidate_edge.str();
        return out;
    }

    template
    class candidate_edge<geometry::network::types_geographic::network_static>;

    template
    class candidate_edge<geometry::network::types_cartesian::network_static>;

    template
    bool operator==(const candidate_edge <geometry::network::types_geographic::network_static> &left,
                    const candidate_edge <geometry::network::types_geographic::network_static> &right);

    template
    bool operator==(const candidate_edge <geometry::network::types_cartesian::network_static> &left,
                    const candidate_edge <geometry::network::types_cartesian::network_static> &right);

    template
    bool operator!=(const candidate_edge <geometry::network::types_geographic::network_static> &left,
                    const candidate_edge <geometry::network::types_geographic::network_static> &right);

    template
    bool operator!=(const candidate_edge <geometry::network::types_cartesian::network_static> &left,
                    const candidate_edge <geometry::network::types_cartesian::network_static> &right);

    template
    std::ostream &operator<<(std::ostream &out,
                             const candidate_edge <geometry::network::types_geographic::network_static> &candidate_edge);

    template
    std::ostream &operator<<(std::ostream &out,
                             const candidate_edge <geometry::network::types_cartesian::network_static> &candidate_edge);

}
