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
        using point_type = typename Track::point_type;
        using candidate_node_type = candidate_node<Network>;
        using candidate_edge_type = candidate_edge<Network>;

        const Track *track;
        std::size_t index;
        double radius;
        std::size_t number;
        bool next_equal;
        std::vector<candidate_node_type> nodes;
        std::vector<candidate_edge_type> edges;

        candidate(const Track &track, std::size_t index, bool next_equal,
                  std::vector<candidate_node_type> nodes, std::vector<candidate_edge_type> edges)
                : track{&track}, index{index}, next_equal{next_equal}, nodes{std::move(nodes)},
                  edges{std::move(edges)} {
            assert(index < track.size());
        }

        ~candidate() = default;

        candidate(const candidate &other) = default;

        candidate(candidate &&other) noexcept = default;

        candidate &operator=(const candidate &other) = default;

        candidate &operator=(candidate &&other) noexcept = default;

        [[nodiscard]] const point_type &point() const {
            return track->at(index);
        }

        [[nodiscard]] std::vector<std::string> header() const {
            return {"index", "projection", "distance", "projection_point", "from", "to"};
        }

        [[nodiscard]] std::vector<std::vector<std::string>> rows() const {
            std::vector<std::vector<std::string>> row_list;
            row_list.reserve(edges.size());
            for (std::size_t i = 0; i < edges.size(); ++i) {
                const auto &edge = edges[i];
                std::vector<std::string> row;
                row.reserve(6);
                row.emplace_back(std::to_string(i));
                typename Network::edge_type::segment_type projection{point(), edge.projection_point};
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
            for (const auto &fields: row_list) {
                str.append("Candidate(");
                bool first = true;
                for (const auto &field: fields) {
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

#endif //MAP_MATCHING_2_CANDIDATE_HPP
