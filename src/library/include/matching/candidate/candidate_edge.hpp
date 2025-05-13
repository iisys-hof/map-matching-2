// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_EDGE_HPP
#define MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_EDGE_HPP

#include <memory>

#include "types/geometry/rich_type/lazy_rich_line.hpp"

#include "geometry/traits.hpp"

namespace map_matching_2::matching {

    template<typename EdgeDescriptor, typename Point>
    struct candidate_edge {

        using edge_descriptor = EdgeDescriptor;
        using point_type = Point;
        using distance_type = typename geometry::data<Point>::distance_type;
        using rich_line_type = geometry::lazy_rich_line_type<point_type>;

        bool adopted;
        edge_descriptor edge_desc;
        point_type projection_point;
        distance_type distance;
        std::shared_ptr<rich_line_type> from;
        std::shared_ptr<rich_line_type> to;

        constexpr candidate_edge(const bool adopted, edge_descriptor edge_desc,
                point_type projection_point, distance_type distance,
                std::shared_ptr<rich_line_type> from, std::shared_ptr<rich_line_type> to)
            : adopted{adopted}, edge_desc{std::move(edge_desc)},
            projection_point{std::move(projection_point)}, distance{distance},
            from{std::move(from)}, to{std::move(to)} {}

        constexpr candidate_edge(const candidate_edge &other) = default;

        constexpr candidate_edge(candidate_edge &&other) noexcept = default;

        constexpr candidate_edge &operator=(const candidate_edge &other) = default;

        constexpr candidate_edge &operator=(candidate_edge &&other) noexcept = default;

        constexpr ~candidate_edge() = default;

        [[nodiscard]] static std::vector<std::string> header() {
            return {"distance", "projection", "from", "to"};
        }

        [[nodiscard]] std::vector<std::string> row() const {
            std::vector<std::string> row;
            row.reserve(4);
            row.emplace_back(std::to_string(distance));
            row.emplace_back(geometry::to_wkt(projection_point));
            const auto &from_line = from->line();
            row.emplace_back(geometry::to_wkt(from_line));
            const auto &to_line = to->line();
            row.emplace_back(geometry::to_wkt(to_line));
            return row;
        }

        [[nodiscard]] std::string str() const {
            const auto &fields = row();
            std::string str = "CandidateEdge(";
            bool first = true;
            for (const auto &field : fields) {
                if (!first) {
                    str.append(",");
                }
                first = false;
                str.append(field);
            }
            str.append(")");
            return str;
        }

        friend constexpr bool operator==(const candidate_edge &left, const candidate_edge &right) {
            return left.edge_desc == right.edge_desc and
                    geometry::equals_points(left.projection_point, right.projection_point);
        }

        friend constexpr bool operator!=(const candidate_edge &left, const candidate_edge &right) {
            return not(left == right);
        }

        friend constexpr bool operator<(const candidate_edge &left, const candidate_edge &right) {
            return left.distance < right.distance;
        }

        friend constexpr bool operator<=(const candidate_edge &left, const candidate_edge &right) {
            return not(right < left);
        }

        friend constexpr bool operator>(const candidate_edge &left, const candidate_edge &right) {
            return right < left;
        }

        friend constexpr bool operator>=(const candidate_edge &left, const candidate_edge &right) {
            return not(left < right);
        }

        friend std::ostream &operator<<(std::ostream &out, const candidate_edge &candidate_edge) {
            return out << candidate_edge.str();
        }

    };

}

#endif //MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_EDGE_HPP
