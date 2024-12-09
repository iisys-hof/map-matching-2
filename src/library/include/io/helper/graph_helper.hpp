// Copyright (C) 2022-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_HELPER_GRAPH_HELPER_HPP
#define MAP_MATCHING_2_IO_HELPER_GRAPH_HELPER_HPP

#include <format>
#include <exception>

#include <boost/serialization/serialization.hpp>

#include <osmium/tags/taglist.hpp>

#include "graph/concepts.hpp"
#include "graph/traits/graph.hpp"

#include "util/concepts.hpp"

#include "io/memory_mapped/concepts.hpp"
#include "io/memory_mapped/utils.hpp"

namespace map_matching_2::io::helper {

    template<typename GraphHelper>
    concept is_memory_graph_helper = io::memory_mapped::is_memory_types<typename GraphHelper::graph_storage_types>;

    template<typename GraphHelper>
    concept is_mmap_graph_helper = io::memory_mapped::is_mmap_types<typename GraphHelper::graph_storage_types>;

    template<typename GraphStorage, typename TagHelper>
    class graph_helper {

    public:
        using graph_storage_type = GraphStorage;
        using tag_helper_type = TagHelper;
        using graph_storage_types = typename graph_storage_type::types;
        using graph_type = typename graph_storage_type::graph_type;
        using allocator_type = typename graph_storage_type::allocator_type;
        using vertex_data_type = typename graph::graph_traits<graph_type>::vertex_data_type;
        using edge_data_type = typename graph::graph_traits<graph_type>::edge_data_type;
        using vertex_descriptor = typename graph::graph_traits<graph_type>::vertex_descriptor;
        using vertex_size_type = typename graph::graph_traits<graph_type>::vertex_size_type;
        using edge_descriptor = typename graph::graph_traits<graph_type>::edge_descriptor;
        using edge_size_type = typename graph::graph_traits<graph_type>::edge_size_type;
        using rich_line_type = typename edge_data_type::rich_line_type;

        constexpr graph_helper(graph_storage_type graph_storage = {}, tag_helper_type tag_helper = {})
            : _graph_storage{std::move(graph_storage)}, _tag_helper{std::move(tag_helper)} {}

        constexpr graph_helper(const graph_helper &other) = delete;

        constexpr graph_helper(graph_helper &&other) noexcept
            : _graph_storage(std::move(other._graph_storage)), _tag_helper{std::move(other._tag_helper)} {}

        constexpr graph_helper &operator=(const graph_helper &other) = delete;

        constexpr graph_helper &operator=(graph_helper &&other) noexcept {
            if (this != &other) {
                _graph_storage = std::move(other._graph_storage);
                _tag_helper = std::move(other._tag_helper);
            }
            return *this;
        }

        constexpr ~graph_helper() = default;

        [[nodiscard]] constexpr const graph_storage_type &graph_storage() const {
            return _graph_storage;
        }

        [[nodiscard]] constexpr graph_storage_type &graph_storage() {
            return _graph_storage;
        }

        [[nodiscard]] constexpr const tag_helper_type &tag_helper() const {
            return _tag_helper;
        }

        [[nodiscard]] constexpr tag_helper_type &tag_helper() {
            return _tag_helper;
        }

        [[nodiscard]] constexpr const graph_type &graph() const {
            return graph_storage().graph();
        }

        [[nodiscard]] constexpr graph_type &graph() {
            return graph_storage().graph();
        }

        [[nodiscard]] constexpr const allocator_type &allocator() const {
            return graph_storage().allocator();
        }

        [[nodiscard]] constexpr allocator_type &allocator() {
            return graph_storage().allocator();
        }

        template<typename GraphHelperT,
            typename TagHelperT = tag_helper_type>
        void from_adjacency_list(const GraphHelperT &other_graph_helper, const TagHelperT &other_tag_helper) requires
            (graph::is_compressed_sparse_row<graph_type>) {
            const auto &vertex_converter = [&]<typename VertexDataT>(
                    const VertexDataT &vertex_data, const allocator_type &allocator) -> vertex_data_type {
                tag_helper().clone_node_tags(vertex_data.id, other_tag_helper);
                return vertex_data_type{vertex_data.id, vertex_data.point};
            };

            const auto &edge_converter = [&]<typename EdgeDataT>(
                    const EdgeDataT &edge_data, const allocator_type &allocator) -> edge_data_type {
                tag_helper().clone_edge_tags(edge_data.id, other_tag_helper);
                return edge_data_type{edge_data.id, rich_line_type{edge_data.rich_line, allocator}};
            };

            if constexpr (is_mmap_tag_helper<tag_helper_type>) {
                // grow tag storage to storage size of the other tag storage
                const auto grow_size = other_tag_helper.tag_storage().size();
                if constexpr (is_mmap_tag_helper<TagHelperT>) {
                    tag_helper().tag_storage().grow(grow_size);
                } else {
                    tag_helper().tag_storage().grow(2 * grow_size);
                }
            }

            if constexpr (is_mmap_graph_helper<graph_helper>) {
                // grow graph storage to storage size of the other graph storage
                const auto grow_size = other_graph_helper.size();
                if constexpr (is_mmap_graph_helper<GraphHelperT>) {
                    grow(2 * grow_size);
                } else {
                    grow(3 * grow_size);
                }

                io::memory_mapped::retry_alloc([&]() {
                            graph().from_adjacency_list(other_graph_helper.graph(), allocator(),
                                    vertex_converter, edge_converter);
                        }, [&](auto &ex) {
                            drop_handle_bad_alloc(ex);
                        }, graph_storage().max_retries());
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                graph().from_adjacency_list(other_graph_helper.graph(), allocator(),
                        vertex_converter, edge_converter);
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
            }

            // finalize tag storage, will not change anymore
            tag_helper().finalize();
        }

        template<typename Network>
        void simplify(Network &network, bool simplify_network_complete = true) requires
            (graph::is_adjacency_list<graph_type> and std::same_as<graph_helper, typename Network::graph_helper_type>) {
            if (this != &network.graph_helper()) {
                throw std::runtime_error("simplfy needs the network that contains the graph_helper it is called on");
            }

            if constexpr (is_mmap_graph_helper<graph_helper>) {
                // first grow once for having sufficient extra space, better performance than handle_bad_alloc
                grow();

                io::memory_mapped::retry_alloc([&]() {
                            network.simplify(simplify_network_complete);
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                network.simplify(simplify_network_complete);
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        template<typename VertexData = vertex_data_type,
            typename TagHelperT = tag_helper_type>
        vertex_descriptor add_vertex(const VertexData &vertex_data, const TagHelperT &other) requires
            (graph::is_adjacency_list<graph_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            tag_helper().clone_node_tags(vertex_data.id, other);
                            return graph().add_vertex(vertex_data_type{
                                    vertex_data.id, vertex_data.point
                            });
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                tag_helper().clone_node_tags(vertex_data.id, other);
                return graph().add_vertex(vertex_data_type{
                        vertex_data.id, vertex_data.point
                });
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        template<typename VertexData = vertex_data_type>
        vertex_descriptor add_vertex(const VertexData &vertex_data, const osmium::TagList &tag_list) requires
            (graph::is_adjacency_list<graph_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            tag_helper().set_node_tags(vertex_data.id, tag_list);
                            return graph().add_vertex(vertex_data_type{
                                    vertex_data.id, vertex_data.point
                            });
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                tag_helper().set_node_tags(vertex_data.id, tag_list);
                return graph().add_vertex(vertex_data_type{
                        vertex_data.id, vertex_data.point
                });
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        template<typename VertexData>
        vertex_descriptor add_vertex(const VertexData &vertex_data) requires
            (graph::is_adjacency_list<graph_type> and not std::same_as<VertexData, vertex_data_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            return graph().add_vertex(vertex_data_type{
                                    vertex_data.id, vertex_data.point
                            });
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                return graph().add_vertex(vertex_data_type{
                        vertex_data.id, vertex_data.point
                });
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        vertex_descriptor add_vertex(vertex_data_type vertex_data) requires
            (graph::is_adjacency_list<graph_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            return graph().add_vertex(std::move(vertex_data));
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                return graph().add_vertex(std::move(vertex_data));
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        template<typename VertexData = vertex_data_type,
            typename TagHelperT = tag_helper_type>
        vertex_size_type add_vertex_with_index_mapping(const VertexData &vertex_data, const TagHelperT &other) requires
            (graph::is_adjacency_list<graph_type>) {
            const auto &_add = [this, &vertex_data, &other]() -> vertex_size_type {
                tag_helper().clone_node_tags(vertex_data.id, other);
                vertex_descriptor vertex_desc = graph().add_vertex(vertex_data_type{
                        vertex_data.id, vertex_data.point
                });
                return _add_to_vertex_index_map(vertex_desc);
            };

            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]()-> vertex_size_type {
                            return _add();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                return _add();
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        template<typename VertexData = vertex_data_type>
        vertex_size_type add_vertex_with_index_mapping(
                const VertexData &vertex_data, const osmium::TagList &tag_list) requires
            (graph::is_adjacency_list<graph_type>) {
            const auto &_add = [this, &vertex_data, &tag_list]() -> vertex_size_type {
                tag_helper().set_node_tags(vertex_data.id, tag_list);
                vertex_descriptor vertex_desc = graph().add_vertex(vertex_data_type{
                        vertex_data.id, vertex_data.point
                });
                return _add_to_vertex_index_map(vertex_desc);
            };

            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() -> vertex_size_type {
                            return _add();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                return _add();
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        template<typename VertexData>
        vertex_size_type add_vertex_with_index_mapping(const VertexData &vertex_data) requires
            (graph::is_adjacency_list<graph_type> and not std::same_as<VertexData, vertex_data_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            vertex_descriptor vertex_desc = graph().add_vertex(vertex_data_type{
                                    vertex_data.id, vertex_data.point
                            });
                            return _add_to_vertex_index_map(vertex_desc);
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                vertex_descriptor vertex_desc = graph().add_vertex(vertex_data_type{
                        vertex_data.id, vertex_data.point
                });
                return _add_to_vertex_index_map(vertex_desc);
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        vertex_size_type add_vertex_with_index_mapping(vertex_data_type vertex_data) requires
            (graph::is_adjacency_list<graph_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            vertex_descriptor vertex_desc = graph().add_vertex(std::move(vertex_data));
                            return _add_to_vertex_index_map(vertex_desc);
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                vertex_descriptor vertex_desc = graph().add_vertex(std::move(vertex_data));
                return _add_to_vertex_index_map(vertex_desc);
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        const vertex_descriptor &get_vertex_descriptor_for_index(vertex_size_type vertex_index) const requires
            (graph::is_adjacency_list<graph_type>) {
            return graph_storage().vertex_index_map().at(vertex_index);
        }

        template<typename EdgeData = edge_data_type,
            typename TagHelperT = tag_helper_type>
        edge_descriptor add_edge(const vertex_size_type source_index, const vertex_size_type target_index,
                const EdgeData &edge_data, const TagHelperT &other) requires
            (graph::is_adjacency_list<graph_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            const vertex_descriptor &source = get_vertex_descriptor_for_index(source_index);
                            const vertex_descriptor &target = get_vertex_descriptor_for_index(target_index);
                            tag_helper().clone_edge_tags(edge_data.id, other);
                            return graph().add_edge(source, target, edge_data_type{
                                    edge_data.id, rich_line_type{edge_data.rich_line, allocator()}
                            });
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                const vertex_descriptor &source = get_vertex_descriptor_for_index(source_index);
                const vertex_descriptor &target = get_vertex_descriptor_for_index(target_index);
                tag_helper().clone_edge_tags(edge_data.id, other);
                return graph().add_edge(source, target, edge_data_type{
                        edge_data.id, edge_data.rich_line
                });
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        template<typename EdgeData = edge_data_type>
        edge_descriptor add_edge(const vertex_size_type source_index, const vertex_size_type target_index,
                const EdgeData &edge_data, const osmium::TagList &tag_list) requires
            (graph::is_adjacency_list<graph_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            const vertex_descriptor &source = get_vertex_descriptor_for_index(source_index);
                            const vertex_descriptor &target = get_vertex_descriptor_for_index(target_index);
                            tag_helper().set_edge_tags(edge_data.id, tag_list);
                            return graph().add_edge(source, target, edge_data_type{
                                    edge_data.id, rich_line_type{edge_data.rich_line, allocator()}
                            });
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                const vertex_descriptor &source = get_vertex_descriptor_for_index(source_index);
                const vertex_descriptor &target = get_vertex_descriptor_for_index(target_index);
                tag_helper().set_edge_tags(edge_data.id, tag_list);
                return graph().add_edge(source, target, edge_data_type{
                        edge_data.id, edge_data.rich_line
                });
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        template<typename EdgeData>
        edge_descriptor add_edge(const vertex_size_type source_index, const vertex_size_type target_index,
                const EdgeData &edge_data) requires
            (graph::is_adjacency_list<graph_type> and not std::same_as<EdgeData, edge_data_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            const vertex_descriptor &source = get_vertex_descriptor_for_index(source_index);
                            const vertex_descriptor &target = get_vertex_descriptor_for_index(target_index);
                            return graph().add_edge(source, target, edge_data_type{
                                    edge_data.id, rich_line_type{edge_data.rich_line, allocator()}
                            });
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                const vertex_descriptor &source = get_vertex_descriptor_for_index(source_index);
                const vertex_descriptor &target = get_vertex_descriptor_for_index(target_index);
                return graph().add_edge(source, target, edge_data_type{
                        edge_data.id, edge_data.rich_line
                });
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        edge_descriptor add_edge(const vertex_size_type source_index, const vertex_size_type target_index,
                edge_data_type edge_data) requires
            (graph::is_adjacency_list<graph_type>) {
            if constexpr (is_mmap_graph_helper<graph_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            const vertex_descriptor &source = get_vertex_descriptor_for_index(source_index);
                            const vertex_descriptor &target = get_vertex_descriptor_for_index(target_index);
                            return graph().add_edge(source, target, std::move(edge_data));
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else if constexpr (is_memory_graph_helper<graph_helper>) {
                const vertex_descriptor &source = get_vertex_descriptor_for_index(source_index);
                const vertex_descriptor &target = get_vertex_descriptor_for_index(target_index);
                return graph().add_edge(source, target, std::move(edge_data));
            } else {
                static_assert(util::dependent_false_v<graph_helper>, "graph_helper type unknown");
                throw std::runtime_error{"graph_helper type unknown"};
            }
        }

        [[nodiscard]] std::size_t size() const {
            return _graph_storage.size();
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) requires
            (is_memory_graph_helper<graph_helper>) {
            ar & _graph_storage;
        }

    private:
        graph_storage_type _graph_storage;
        tag_helper_type _tag_helper;

        void grow() {
            if constexpr (io::memory_mapped::has_grow<graph_storage_type>) {
                std::size_t previous_vertices_size = graph().vertices_view().size();
                std::size_t previous_edges_size = graph().edges_view().size();
                graph_storage().grow();
                if (graph().vertices_view().size() != previous_vertices_size or
                    graph().edges_view().size() != previous_edges_size) {
                    throw std::runtime_error("error growing graph_storage");
                }
            }
        }

        void grow(const std::size_t size) {
            if constexpr (io::memory_mapped::has_grow<graph_storage_type>) {
                std::size_t previous_vertices_size = graph().vertices_view().size();
                std::size_t previous_edges_size = graph().edges_view().size();
                graph_storage().grow(size);
                if (graph().vertices_view().size() != previous_vertices_size or
                    graph().edges_view().size() != previous_edges_size) {
                    throw std::runtime_error("error growing graph_storage");
                }
            }
        }

        void handle_bad_alloc(auto &ex) requires
            (graph::is_adjacency_list<graph_type>) {
            if constexpr (io::memory_mapped::has_grow<graph_storage_type>) {
                std::size_t previous_vertices_size = graph().vertices_view().size();
                std::size_t previous_edges_size = graph().edges_view().size();
                graph_storage().grow();
                if (graph().vertices_view().size() != previous_vertices_size or
                    graph().edges_view().size() != previous_edges_size) {
                    std::rethrow_exception(std::current_exception());
                }
            } else {
                std::rethrow_exception(std::current_exception());
            }
        }

        void drop_handle_bad_alloc(auto &ex) requires
            (graph::is_compressed_sparse_row<graph_type>) {
            if constexpr (io::memory_mapped::has_grow<graph_storage_type>) {
                graph_storage().drop_grow();
                std::cout << "INFO: graph_storage.drop_grow() called." << std::endl;
            } else {
                std::rethrow_exception(std::current_exception());
            }
        }

        [[nodiscard]] vertex_size_type _add_to_vertex_index_map(const vertex_descriptor &vertex_desc) requires
            (graph::is_adjacency_list<graph_type>) {
            const auto current_size = graph_storage().vertex_index_map().size();

            vertex_size_type vertex_index = graph().vertex_index(vertex_desc);

            if (current_size != vertex_index) {
                throw std::runtime_error{
                        std::format("vertex_index_map size() {} is not equal to {}", current_size, vertex_index)
                };
            }

            try {
                graph_storage().vertex_index_map().emplace_back(vertex_desc);
            } catch (...) {
                while (graph_storage().vertex_index_map().size() > current_size) {
                    graph_storage().vertex_index_map().pop_back();
                }
                graph().remove_vertex(vertex_desc);
                throw;
            }

            return vertex_index;
        }

    };

}

#endif //MAP_MATCHING_2_IO_HELPER_GRAPH_HELPER_HPP
