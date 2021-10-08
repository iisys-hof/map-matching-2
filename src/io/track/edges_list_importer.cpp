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

#include "edges_list_importer.hpp"

#include <fstream>
#include <string>

#include <geometry/network/types.hpp>
#include <geometry/track/types.hpp>

namespace map_matching_2::io::track {

    template<typename Network, typename MultiTrack>
    edges_list_importer<Network, MultiTrack>::edges_list_importer(
            std::string filename, const Network &network, std::unordered_map<std::string, MultiTrack> &tracks)
            : importer{std::move(filename)}, _network{network}, _tracks{tracks} {}

    template<typename Network, typename MultiTrack>
    void edges_list_importer<Network, MultiTrack>::read() {
        using rich_line_type = typename MultiTrack::rich_line_type;
        using multi_rich_line_type = typename MultiTrack::multi_rich_line_type;

        std::ifstream route;
        route.open(this->filename());

        std::unordered_map<std::int64_t, std::vector<typename Network::edge_descriptor>> edge_map;
        for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(_network.graph))) {
            const auto &edge = _network.graph[edge_descriptor];
            std::int64_t edge_id = edge.id;
            auto search = edge_map.find(edge_id);
            std::vector<typename Network::edge_descriptor> *edges;
            if (search == edge_map.end()) {
                auto inserted = edge_map.emplace(edge_id, std::vector<typename Network::edge_descriptor>{});
                edges = &inserted.first->second;
            } else {
                edges = &search->second;
            }
            edges->emplace_back(edge_descriptor);
        }

        std::string line;
        std::vector<rich_line_type> edges;
        while (std::getline(route, line)) {
            std::int64_t edge_id = std::stol(line);
            const auto &edge_descriptors = edge_map[edge_id];
            if (edge_descriptors.size() != 1) {
                throw std::runtime_error{"could not find edge with id " + std::to_string(edge_id)};
            }
            const rich_line_type &edge = _network.graph[edge_descriptors.front()];
            edges.emplace_back(edge);
        }

        std::string id{"0"};
        multi_rich_line_type track = multi_rich_line_type::merge({edges.begin(), edges.end()});
        _tracks.emplace(id, MultiTrack{id, std::move(track)});
    }

    template
    class edges_list_importer<geometry::network::types_geographic::network_modifiable,
            geometry::track::types_geographic::multi_track_type>;

    template
    class edges_list_importer<geometry::network::types_cartesian::network_modifiable,
            geometry::track::types_cartesian::multi_track_type>;

}