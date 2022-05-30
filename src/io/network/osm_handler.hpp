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

#ifndef MAP_MATCHING_2_OSM_HANDLER_HPP
#define MAP_MATCHING_2_OSM_HANDLER_HPP

#include <osmium/osm.hpp>
#include <osmium/handler.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/tags/tags_filter.hpp>

namespace map_matching_2::io::network {

    template<typename Network>
    class osm_handler : public osmium::handler::Handler {

    private:
        using node_type = typename Network::node_type;
        using vertex_descriptor = typename Network::graph_traits::vertex_descriptor;
        using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, std::size_t>;

        static constexpr std::size_t empty_value = osmium::index::empty_value<std::size_t>();

        struct vertex_type {
            node_type node;
            vertex_descriptor vertex;
            bool added = false;
        };

        Network &_network;

        index_type _node_index;
        std::vector<vertex_type> _vertices;

        osmium::TagsFilter _query, _filter, _oneway, _oneway_reverse;

        [[nodiscard]] bool keep(const osmium::Way &way) const;

        [[nodiscard]] std::tuple<bool, bool> detect_oneway(const osmium::Way &way) const;

        std::vector<std::uint64_t> tags(const osmium::TagList &tag_list);

    public:
        osm_handler(Network &network, osmium::TagsFilter query, osmium::TagsFilter filter, osmium::TagsFilter oneway,
                    osmium::TagsFilter oneway_reverse);

        void node(const osmium::Node &osm_node) noexcept;

        void way(const osmium::Way &osm_way) noexcept;

        void relation(const osmium::Relation &relation) noexcept;

    };

}

#endif //MAP_MATCHING_2_OSM_HANDLER_HPP
