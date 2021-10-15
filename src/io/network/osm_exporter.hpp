// Copyright (C) 2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_OSM_EXPORTER_HPP
#define MAP_MATCHING_2_OSM_EXPORTER_HPP

#include "../exporter.hpp"

#include <utility>
#include <unordered_set>
#include <unordered_map>

#include <boost/functional/hash.hpp>

#include <osmium/io/any_output.hpp>

namespace map_matching_2::io::network {

    template<typename Network>
    class osm_exporter : public exporter {

    private:
        Network &_network;

        using node_type = typename Network::node_type;
        using edge_type = typename Network::edge_type;
        using coordinate_type = typename Network::coordinate_type;

        using point_map_key_type = std::pair<coordinate_type, coordinate_type>;
        using point_map_type = std::unordered_map<point_map_key_type, osmium::object_id_type, boost::hash<point_map_key_type>>;
        using point_set_type = std::unordered_set<point_map_key_type, boost::hash<point_map_key_type>>;

        point_map_type _point_map;

        void add_node(osmium::memory::Buffer &buffer, const node_type &node, point_set_type &points_added);

        void add_point(osmium::memory::Buffer &buffer, const std::pair<point_map_key_type, std::size_t> &point);

        void add_way(osmium::memory::Buffer &buffer, const edge_type &edge, bool is_oneway,
                     osmium::object_id_type index);

    public:
        osm_exporter(std::string filename, Network &network);

        void write();

    };

}

#endif //MAP_MATCHING_2_OSM_EXPORTER_HPP
