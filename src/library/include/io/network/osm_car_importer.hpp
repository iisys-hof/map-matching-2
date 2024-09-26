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

#ifndef MAP_MATCHING_2_IO_NETWORK_OSM_CAR_IMPORTER_HPP
#define MAP_MATCHING_2_IO_NETWORK_OSM_CAR_IMPORTER_HPP

#include "types/io/network/osm_importer.hpp"

namespace map_matching_2::io::network {

    template<typename OSMHandler>
    class osm_car_importer : public osm_importer<OSMHandler> {

    public:
        constexpr osm_car_importer(std::string filename, OSMHandler &osm_handler)
            : osm_importer<OSMHandler>{std::move(filename), osm_handler} {}

        constexpr osm_car_importer(std::vector<std::string> filenames, OSMHandler &osm_handler)
            : osm_importer<OSMHandler>{std::move(filenames), osm_handler} {}

    protected:
        osmium::TagsFilter query() override {
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

        osmium::TagsFilter filter() override {
            osmium::TagsFilter filter{true};
            // https://wiki.openstreetmap.org/wiki/Key:access
            // for now, allow all access restricted roads, since tracks might lie on them
            // filter.add_rule(false, "access", "no");
            // filter.add_rule(false, "access", "private");
            return filter;
        }

    };

}

#endif //MAP_MATCHING_2_IO_NETWORK_OSM_CAR_IMPORTER_HPP
