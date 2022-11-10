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

#ifndef MAP_MATCHING_2_EDGES_LIST_IMPORTER_HPP
#define MAP_MATCHING_2_EDGES_LIST_IMPORTER_HPP

#include <fstream>
#include <string>
#include <vector>

#include <absl/container/flat_hash_map.h>

#include <geometry/network/network.hpp>

#include "../importer.hpp"

namespace map_matching_2::io::track {

    template<typename Network, typename MultiTrack>
    class edges_list_importer : public importer {

    public:
        using measurement_type = typename MultiTrack::measurement_type;
        using point_type = typename MultiTrack::point_type;
        using line_type = typename MultiTrack::line_type;

        edges_list_importer(std::string filename, const Network &network,
                            absl::flat_hash_map<std::string, MultiTrack> &tracks)
                : importer{std::move(filename)}, _network{network}, _tracks{tracks} {}

        void read() override {
            using edge_type = typename Network::edge_type;
            using edge_rich_line_type = typename edge_type::rich_line_type;
            using rich_line_type = typename MultiTrack::rich_line_type;
            using multi_rich_line_type = typename MultiTrack::multi_rich_line_type;

            std::ifstream route;
            route.open(this->filename());

            absl::flat_hash_map<std::int64_t, std::vector<typename Network::edge_descriptor>> edge_map;
            for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(_network.graph()))) {
                const auto &edge = _network.edge(edge_descriptor);
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
                const edge_type &edge = _network.edge(edge_descriptors.front());
                edges.emplace_back(rich_line_type{static_cast<const edge_rich_line_type &>(edge)});
            }

            std::string id{"0"};
            multi_rich_line_type track{
                    multi_rich_line_type::template merge<multi_rich_line_type>({edges.begin(), edges.end()})};
            _tracks.emplace(id, MultiTrack{id, std::move(track)});
        }

    private:
        const Network &_network;
        absl::flat_hash_map<std::string, MultiTrack> &_tracks;

    };

}

#endif //MAP_MATCHING_2_EDGES_LIST_IMPORTER_HPP
