// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TESTS_HELPER_MATCHER_FIXTURE_HPP
#define MAP_MATCHING_2_TESTS_HELPER_MATCHER_FIXTURE_HPP

#include <string>
#include <initializer_list>

#include "types/matching/matcher.hpp"

#include "network_fixture.hpp"

using matcher_geographic = MM2_MATCHER(network_geographic);
using matcher_metric = MM2_MATCHER(network_metric);

template<typename Types>
struct matcher_fixture {

    using matcher_type = Types;

    typename matcher_type::track_type
    create_track(std::string id,
            std::initializer_list<typename matcher_type::track_type::point_type> points) {
        typename matcher_type::track_type::line_type line;
        line.reserve(points.size());
        for (const auto &point : points) {
            line.emplace_back(point);
        }

        return typename matcher_type::track_type{std::move(id), std::move(line)};
    }

    template<typename Network>
    typename matcher_type::network_type prepare_network(Network &network_import) {
        typename matcher_type::network_type network;
        network_import.simplify();
        network.graph_helper().from_adjacency_list(network_import.graph_helper(), network_import.tag_helper());
        network.build_spatial_indices();
        return network;
    }

    std::int64_t get_candidate_position(typename matcher_type::network_type &network,
            auto &candidates, std::size_t candidate_index, std::pair<std::uint64_t, std::uint64_t> node_pair) {
        for (std::int64_t i = 0; i < candidates.at(candidate_index).edges.size(); ++i) {
            const auto &candidate_edge = candidates.at(candidate_index).edges.at(i);
            const auto source = network.graph().source(candidate_edge.edge_desc);
            const auto target = network.graph().target(candidate_edge.edge_desc);

            if (network.vertex(source).id == node_pair.first and network.vertex(target).id == node_pair.second) {
                return i;
            }
        }

        return -1;
    }

};

#endif //MAP_MATCHING_2_TESTS_HELPER_MATCHER_FIXTURE_HPP
