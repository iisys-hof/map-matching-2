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

#ifndef MAP_MATCHING_2_OSM_IMPORTER_HPP
#define MAP_MATCHING_2_OSM_IMPORTER_HPP

#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <osmium/tags/tags_filter.hpp>

#include "../importer.hpp"
#include "osm_handler.hpp"

namespace map_matching_2::io::network {

    template<typename Network>
    class osm_importer : public importer {

    private:
        Network &_network;

    protected:
        virtual osmium::TagsFilter query() {
            osmium::TagsFilter filter{false};
            filter.add_rule(true, "highway");
            return filter;
        }

        virtual osmium::TagsFilter filter() {
            osmium::TagsFilter filter{true};
            return filter;
        }

        virtual osmium::TagsFilter oneway() {
            osmium::TagsFilter filter{false};
            filter.add_rule(true, "oneway", "yes");
            filter.add_rule(true, "oneway", "true");
            filter.add_rule(true, "oneway", "1");
            filter.add_rule(true, "junction", "roundabout");
            filter.add_rule(true, "highway", "motorway");
            return filter;
        }

        virtual osmium::TagsFilter oneway_reverse() {
            osmium::TagsFilter filter{false};
            filter.add_rule(true, "oneway", "-1");
            filter.add_rule(true, "oneway", "reverse");
            return filter;
        }

    public:
        osm_importer(std::string filename, Network &network)
                : importer{std::move(filename)}, _network{network} {}

        void read() override {
            osmium::io::File file{this->filename()};

            //osm_relations_manager osm_relations_manager{_network};
            //osmium::relations::read_relations(file, osm_relations_manager);

            osmium::io::Reader reader{file, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
            osm_handler<Network> osm_handler{_network, query(), filter(), oneway(), oneway_reverse()};

            //osmium::apply(reader, osm_handler, osm_relations_manager.handler());
            osmium::apply(reader, osm_handler);
        }

    };

}

#endif //MAP_MATCHING_2_OSM_IMPORTER_HPP
