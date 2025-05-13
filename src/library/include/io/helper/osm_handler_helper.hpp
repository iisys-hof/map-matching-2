// Copyright (C) 2023-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_HELPER_OSM_HANDLER_HELPER_HPP
#define MAP_MATCHING_2_IO_HELPER_OSM_HANDLER_HELPER_HPP

#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>

#include "types/geometry/rich_type/rich_line.hpp"
#include "types/geometry/network/node.hpp"
#include "types/geometry/network/edge.hpp"
#include "types/geometry/reprojector.hpp"

#include "geometry/rich_type/traits/rich_line.hpp"
#include "geometry/network/traits/network.hpp"

#include "io/memory_mapped/concepts.hpp"
#include "io/memory_mapped/utils.hpp"

namespace map_matching_2::io::helper {

    template<typename OSMHandlerHelper>
    concept is_memory_osm_handler_helper = io::memory_mapped::is_memory_types<typename
        OSMHandlerHelper::osm_handler_storage_types>;

    template<typename OSMHandlerHelper>
    concept is_mmap_osm_handler_helper = io::memory_mapped::is_mmap_types<typename
        OSMHandlerHelper::osm_handler_storage_types>;

    template<typename OSMHandlerStorage, typename TagHelper>
    class osm_handler_helper {

    public:
        using osm_handler_storage_type = OSMHandlerStorage;
        using tag_helper_type = TagHelper;

        using osm_handler_storage_types = typename osm_handler_storage_type::types;
        using allocator_type = typename osm_handler_storage_type::allocator_type;
        using osm_vertex_type = typename osm_handler_storage_type::osm_vertex_type;
        using vertex_type = typename osm_vertex_type::vertex_type;
        using vertex_container_type = typename osm_handler_storage_type::vertex_container_type;

        constexpr osm_handler_helper(
                osm_handler_storage_type osm_handler_storage = {}, tag_helper_type tag_helper = {})
            : _osm_handler_storage{std::move(osm_handler_storage)}, _tag_helper{std::move(tag_helper)} {}

        constexpr osm_handler_helper(const osm_handler_helper &other) = delete;

        constexpr osm_handler_helper(osm_handler_helper &&other) noexcept
            : _osm_handler_storage(std::move(other._osm_handler_storage)),
            _tag_helper{std::move(other._tag_helper)} {}

        constexpr osm_handler_helper &operator=(const osm_handler_helper &other) = delete;

        constexpr osm_handler_helper &operator=(osm_handler_helper &&other) noexcept {
            if (this != &other) {
                _osm_handler_storage = std::move(other._osm_handler_storage);
                _tag_helper = std::move(other._tag_helper);
            }
            return *this;
        }

        constexpr ~osm_handler_helper() = default;

        [[nodiscard]] constexpr const osm_handler_storage_type &osm_handler_storage() const {
            return _osm_handler_storage;
        }

        [[nodiscard]] constexpr osm_handler_storage_type &osm_handler_storage() {
            return _osm_handler_storage;
        }

        [[nodiscard]] constexpr const tag_helper_type &tag_helper() const {
            return _tag_helper;
        }

        [[nodiscard]] constexpr tag_helper_type &tag_helper() {
            return _tag_helper;
        }

        [[nodiscard]] constexpr const vertex_container_type &vertices() const {
            return osm_handler_storage().vertices();
        }

        [[nodiscard]] constexpr vertex_container_type &vertices() {
            return osm_handler_storage().vertices();
        }

        [[nodiscard]] constexpr const allocator_type &allocator() const {
            return osm_handler_storage().allocator();
        }

        [[nodiscard]] constexpr allocator_type &allocator() {
            return osm_handler_storage().allocator();
        }

        void add_node(const osmium::Node &osm_node, const geometry::point_reprojector_variant &reprojector_variant) {
            using osm_point_type = geometry::point_type<geometry::cs_geographic>;

            const auto _add_node = [&]() {
                const auto id = static_cast<std::uint64_t>(osm_node.id());
                const auto node_location = osm_node.location();
                tag_helper().set_node_tags(id, osm_node.tags());
                if (not _sort_vertices and not vertices().empty() and vertices().back().vertex.id > id) {
                    _sort_vertices = true;
                }
                vertices().emplace_back(osm_vertex_type{
                        id, osm_point_type{node_location.lon(), node_location.lat()}, reprojector_variant
                });
            };

            if constexpr (is_mmap_osm_handler_helper<osm_handler_helper>) {
                io::memory_mapped::retry_alloc([&]() {
                            _add_node();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        }, _critical);
            } else {
                _add_node();
            }
        }

        void check_and_sort_nodes() {
            if (_sort_vertices) {
                std::sort(std::begin(vertices()), std::end(vertices()));
                _sort_vertices = false;
            }
        }

        [[nodiscard]] bool contains_node(const std::uint64_t id) {
            check_and_sort_nodes();
            const osm_vertex_type empty_osm_vertex{id, _empty_point};
            return std::binary_search(std::begin(vertices()), std::end(vertices()), empty_osm_vertex);
        }

        [[nodiscard]] osm_vertex_type &get_node(const std::uint64_t id) {
            check_and_sort_nodes();
            const osm_vertex_type empty_osm_vertex{id, _empty_point};
            auto it = std::lower_bound(std::begin(vertices()), std::end(vertices()), empty_osm_vertex);
            if (it == std::end(vertices()) or *it != empty_osm_vertex) {
                throw std::out_of_range{std::format("osm_vertex {} not found in vertices", id)};
            }
            return *it;
        }

        template<typename GraphHelper>
        void add_vertex(GraphHelper &graph_helper, osm_vertex_type &osm_vertex) {
            if (not osm_vertex.added) {
                osm_vertex.vertex_index = graph_helper.add_vertex_with_index_mapping(osm_vertex.vertex, tag_helper());
                osm_vertex.added = true;
            }
        }

        template<typename GraphHelper>
        void add_edge(GraphHelper &graph_helper,
                const osm_vertex_type &osm_vertex_from, const osm_vertex_type &osm_vertex_to,
                const bool oneway, const bool reverse, const osmium::Way &osm_way) {
            const auto id = static_cast<std::uint64_t>(osm_way.id());
            const osmium::TagList &tag_list = osm_way.tags();

            using point_type = typename GraphHelper::vertex_data_type::point_type;
            using edge_data_type = geometry::network::edge_type<point_type>; // in-memory type for transportation
            using rich_line_type = typename edge_data_type::rich_line_type;
            using line_type = typename geometry::rich_line_traits<rich_line_type>::line_type;

            if (oneway) {
                if (not reverse) {
                    graph_helper.add_edge(
                            osm_vertex_from.vertex_index, osm_vertex_to.vertex_index,
                            edge_data_type{
                                    id, rich_line_type{
                                            line_type{osm_vertex_from.vertex.point, osm_vertex_to.vertex.point}
                                    }
                            }, tag_list);
                } else {
                    graph_helper.add_edge(
                            osm_vertex_to.vertex_index, osm_vertex_from.vertex_index,
                            edge_data_type{
                                    id, rich_line_type{
                                            line_type{osm_vertex_to.vertex.point, osm_vertex_from.vertex.point}
                                    }
                            }, tag_list);
                }
            } else {
                graph_helper.add_edge(
                        osm_vertex_from.vertex_index, osm_vertex_to.vertex_index,
                        edge_data_type{
                                id, rich_line_type{
                                        line_type{osm_vertex_from.vertex.point, osm_vertex_to.vertex.point}
                                }
                        }, tag_list);

                graph_helper.add_edge(
                        osm_vertex_to.vertex_index, osm_vertex_from.vertex_index,
                        edge_data_type{
                                id, rich_line_type{
                                        line_type{osm_vertex_to.vertex.point, osm_vertex_from.vertex.point}
                                }
                        }, tag_list);
            }
        }

    private:
        static constexpr typename osm_vertex_type::vertex_type::point_type _empty_point{};

        osm_handler_storage_type _osm_handler_storage;
        tag_helper_type _tag_helper;
        bool _critical{false}, _sort_vertices{false};

        void handle_bad_alloc(auto &ex) {
            if constexpr (io::memory_mapped::has_grow<osm_handler_storage_type>) {
                std::size_t previous_size = vertices().size();
                osm_handler_storage().grow();
                if (vertices().size() != previous_size) {
                    std::rethrow_exception(std::current_exception());
                }
            } else {
                std::rethrow_exception(std::current_exception());
            }
        }

    };

}

#endif //MAP_MATCHING_2_IO_HELPER_OSM_HANDLER_HELPER_HPP
