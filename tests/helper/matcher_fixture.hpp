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

#ifndef MAP_MATCHING_2_MATCHER_FIXTURE_HPP
#define MAP_MATCHING_2_MATCHER_FIXTURE_HPP

#include <string>
#include <initializer_list>

#include <matching/types.hpp>

#include "network_fixture.hpp"

using matcher_geographic = map_matching_2::matching::types_geographic;
using matcher_metric = map_matching_2::matching::types_cartesian;

template<typename Types>
struct matcher_fixture {

    typename Types::matcher_static::track_type
    create_track(std::string id,
                 std::initializer_list<typename Types::matcher_static::track_type::measurement_type> measurements) {
        typename Types::matcher_static::track_type::line_type line;
        line.reserve(measurements.size());
        for (const auto &measurement: measurements) {
            line.emplace_back(measurement.point);
        }

        return typename Types::matcher_static::track_type
                {std::move(id), std::move(line),
                 std::vector<typename Types::matcher_static::track_type::measurement_type>
                         {measurements.begin(), measurements.end()}};
    }

    template<typename Network>
    typename Types::matcher_static::network_type prepare_network(Network &network) {
        typename Types::matcher_static::network_type matcher_network;
        network.simplify();
        network.convert(matcher_network);
        matcher_network.rebuild_spatial_indices();
        return matcher_network;
    }

    typename Types::matcher_static create_matcher(typename Types::matcher_static::network_type &network) {
        return typename Types::matcher_static{network, true};
    }

    std::int64_t get_candidate_position(typename Types::matcher_static::network_type &network,
                                        std::vector<typename Types::matcher_static::candidate_type> &candidates,
                                        std::size_t candidate_index,
                                        std::pair<osmium::object_id_type, osmium::object_id_type> node_pair) {
        for (std::size_t i = 0; i < candidates.at(candidate_index).edges.size(); ++i) {
            const auto &candidate_edge = candidates.at(candidate_index).edges.at(i);
            const auto source = boost::source(candidate_edge.edge_descriptor, network.graph);
            const auto target = boost::target(candidate_edge.edge_descriptor, network.graph);

            if (network.graph[source].id == node_pair.first and network.graph[target].id == node_pair.second) {
                return i;
            }
        }

        return -1;
    }

};

#endif //MAP_MATCHING_2_MATCHER_FIXTURE_HPP
