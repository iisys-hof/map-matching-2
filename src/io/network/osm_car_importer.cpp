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

#include "osm_car_importer.hpp"

#include <geometry/network/types.hpp>

namespace map_matching_2::io::network {

    template<typename Network>
    osmium::TagsFilter osm_car_importer<Network>::query() {
        osmium::TagsFilter filter{false};
        // https://wiki.openstreetmap.org/wiki/Key:highway
        filter.add_rule(false, "highway", "footway");
        filter.add_rule(false, "highway", "pedestrian");
        filter.add_rule(false, "highway", "steps");
        filter.add_rule(false, "highway", "bridleway");
        filter.add_rule(false, "highway", "construction");
        filter.add_rule(false, "highway", "cycleway");
        filter.add_rule(false, "highway", "path");
        filter.add_rule(false, "highway", "bus_guideway");
        filter.add_rule(false, "highway", "platform");
        filter.add_rule(true, "highway");
        return filter;
    }

    template<typename Network>
    osmium::TagsFilter osm_car_importer<Network>::filter() {
        osmium::TagsFilter filter{true};
        // https://wiki.openstreetmap.org/wiki/Key:access
        filter.add_rule(false, "access", "no");
        filter.add_rule(false, "access", "private");
        return filter;
    }

    template<typename Network>
    osm_car_importer<Network>::osm_car_importer(std::string filename, Network &network)
            : osm_importer<Network>{std::move(filename), network} {}

    template
    class osm_car_importer<geometry::network::types_geographic::network_modifiable>;

    template
    class osm_car_importer<geometry::network::types_cartesian::network_modifiable>;

}
