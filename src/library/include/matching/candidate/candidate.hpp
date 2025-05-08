// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_HPP
#define MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_HPP

#include "types/matching/candidate/candidate_node.hpp"
#include "types/matching/candidate/candidate_edge.hpp"

namespace map_matching_2::matching {

    template<typename VertexDescriptor, typename EdgeDescriptor, typename Track>
    struct candidate {
        using vertex_descriptor = VertexDescriptor;
        using edge_descriptor = EdgeDescriptor;
        using track_type = Track;
        using point_type = typename geometry::models<typename track_type::point_type>::explicit_point_type;
        using distance_type = typename geometry::data<point_type>::distance_type;

        using candidate_node_type = candidate_node<vertex_descriptor, distance_type>;
        using candidate_edge_type = candidate_edge<edge_descriptor, point_type>;

        const track_type *track;
        std::size_t index;
        bool next_equal;
        std::vector<candidate_node_type> nodes;
        std::vector<candidate_edge_type> edges;

        constexpr candidate(const track_type &track, const std::size_t index, const bool next_equal,
                std::vector<candidate_node_type> nodes, std::vector<candidate_edge_type> edges)
            : track{&track}, index{index}, next_equal{next_equal}, nodes{std::move(nodes)}, edges{std::move(edges)} {
            assert(index < track.rich_line.size());
        }

        constexpr candidate(const candidate &other) = default;

        constexpr candidate(candidate &&other) noexcept = default;

        constexpr candidate &operator=(const candidate &other) = default;

        constexpr candidate &operator=(candidate &&other) noexcept = default;

        constexpr ~candidate() = default;

        [[nodiscard]] const point_type &point() const {
            return track->rich_line.at(index);
        }

        [[nodiscard]] static std::vector<std::string> header() {
            return {"index", "projection", "distance", "projection_point", "from", "to"};
        }

        [[nodiscard]] std::vector<std::vector<std::string>> rows() const {
            using segment_type = typename geometry::models<point_type>::segment_type;

            std::vector<std::vector<std::string>> row_list;
            row_list.reserve(edges.size());
            for (std::size_t i = 0; i < edges.size(); ++i) {
                const auto &edge = edges[i];
                std::vector<std::string> row;
                row.reserve(6);
                row.emplace_back(std::to_string(i));
                segment_type projection{point(), edge.projection_point};
                row.emplace_back(geometry::to_wkt(projection));
                std::vector<std::string> row_edge = edge.row();
                row.insert(row.end(), row_edge.begin(), row_edge.end());
                row_list.emplace_back(std::move(row));
            }
            return row_list;
        }

        [[nodiscard]] std::string str() const {
            const auto row_list = rows();
            std::string str;
            for (const auto &fields : row_list) {
                str.append("Candidate(");
                bool first = true;
                for (const auto &field : fields) {
                    if (!first) {
                        str.append(",");
                    }
                    first = false;
                    str.append(field);
                }
                str.append(")\n");
            }
            return str;
        }

        friend std::ostream &operator<<(std::ostream &out, const candidate &candidate) {
            return out << candidate.str();
        }

    };

}

#endif //MAP_MATCHING_2_MATCHING_CANDIDATE_CANDIDATE_HPP
