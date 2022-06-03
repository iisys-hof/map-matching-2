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

#include "candidate.hpp"

#include <geometry/track/types.hpp>
#include <geometry/network/types.hpp>
#include <geometry/util.hpp>

namespace map_matching_2::matching {

    template<typename Network, typename Track>
    candidate<Network, Track>::candidate(const Track &track, std::size_t index, bool next_equal,
                                         std::vector<candidate_node_type> nodes, std::vector<candidate_edge_type> edges)
            : track{&track}, index{index}, next_equal{next_equal}, nodes{std::move(nodes)}, edges{std::move(edges)} {
        assert(index < track.line().size());
    }

    template<typename Network, typename Track>
    const typename candidate<Network, Track>::point_type &candidate<Network, Track>::point() const {
        return track->line()[index];
    }

    template<typename Network, typename Track>
    std::vector<std::string> candidate<Network, Track>::header() const {
        return {"index", "projection", "distance", "projection_point", "from", "to"};
    }

    template<typename Network, typename Track>
    std::vector<std::vector<std::string>> candidate<Network, Track>::rows() const {
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

    template<typename Network, typename Track>
    std::string candidate<Network, Track>::str() const {
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

    template<typename Network, typename Track>
    std::ostream &operator<<(std::ostream &out, const candidate<Network, Track> &candidate) {
        out << candidate.str();
        return out;
    }

    template
    class candidate<geometry::network::types_geographic::network_static, geometry::track::types_geographic::track_type>;

    template
    class candidate<geometry::network::types_cartesian::network_static, geometry::track::types_cartesian::track_type>;

    template
    std::ostream &operator<<(std::ostream &out, const candidate <geometry::network::types_geographic::network_static,
    geometry::track::types_geographic::track_type> &candidate);

    template
    std::ostream &operator<<(std::ostream &out, const candidate <geometry::network::types_cartesian::network_static,
    geometry::track::types_cartesian::track_type> &candidate);

}
