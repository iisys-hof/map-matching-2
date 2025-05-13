// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_NETWORK_OSM_IMPORTER_HPP
#define MAP_MATCHING_2_IO_NETWORK_OSM_IMPORTER_HPP

#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <osmium/tags/tags_filter.hpp>

#include "../importer.hpp"

namespace map_matching_2::io::network {

    template<typename OSMHandler>
    class osm_importer : public importer {

    public:
        constexpr osm_importer(std::string filename, OSMHandler &osm_handler)
            : importer{std::move(filename)}, _osm_handler{osm_handler} {}

        constexpr osm_importer(std::vector<std::string> filenames, OSMHandler &osm_handler)
            : importer{std::move(filenames)}, _osm_handler{osm_handler} {}

        void read() override {
            _osm_handler.set_tags_filters(query(), filter(), oneway(), oneway_reverse());

            const auto &filenames = this->filenames();
            for (std::size_t i = 0; i < filenames.size(); ++i) {
                if (not std::filesystem::exists(filenames[i])) {
                    throw std::runtime_error{std::format("file '{}' does not exist", filenames[i])};
                }
                osmium::io::File file{filenames[i]};

                osmium::io::Reader reader{file, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
                osmium::apply(reader, _osm_handler);

                // osm_relations_manager osm_relations_manager{_network};
                // osmium::relations::read_relations(file, osm_relations_manager);
                // osmium::apply(reader, osm_handler, osm_relations_manager.handler());
            }
        }

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

    private:
        OSMHandler &_osm_handler;

    };

}

#endif //MAP_MATCHING_2_IO_NETWORK_OSM_IMPORTER_HPP
