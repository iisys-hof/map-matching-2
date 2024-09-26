// Copyright (C) 2023-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GRAPH_COMPRESSED_SPARSE_ROW_HPP
#define MAP_MATCHING_2_GRAPH_COMPRESSED_SPARSE_ROW_HPP

#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include "util/concepts.hpp"

#include "common.hpp"
#include "concepts.hpp"
#include "container_subview.hpp"

namespace map_matching_2::graph {

    // currently only directional supported
    template<template<typename, typename> typename VertexContainer = std::vector,
        template<typename, typename> typename EdgeContainer = std::vector,
        typename Direction = directional,
        typename VertexData = void,
        typename EdgeData = void,
        typename Allocator = std::allocator<void>,
        template<typename> typename AllocatorTraits = std::allocator_traits> requires
        (util::is_random_access_v<VertexContainer<char,
                typename AllocatorTraits<Allocator>::template rebind_alloc<char>>> and
            util::is_random_access_v<EdgeContainer<char,
                typename AllocatorTraits<Allocator>::template rebind_alloc<char>>>)
    class compressed_sparse_row {

    public:
        using allocator_type = Allocator;

        template<typename Type>
        using allocator_traits_type = AllocatorTraits<Type>;

        template<typename Type>
        using vertex_container_type = VertexContainer<Type,
            typename allocator_traits_type<allocator_type>::template rebind_alloc<Type>>;

        template<typename Type>
        using edge_container_type = EdgeContainer<Type,
            typename allocator_traits_type<allocator_type>::template rebind_alloc<Type>>;

        using direction_type = Direction;

        using is_directional = std::is_base_of<directional, direction_type>;
        static constexpr bool is_directional_v = is_directional::value;

        template<typename Data>
        struct vertex;

        using vertex_data_type = VertexData;
        using vertex_type = vertex<vertex_data_type>;
        using vertex_container = vertex_container_type<vertex_type>;
        using vertex_size_type = typename vertex_container::size_type;
        using vertex_difference_type = typename vertex_container::difference_type;
        using vertex_descriptor = vertex_size_type;

        template<typename Data>
        struct edge {
            using data_type = Data;

            vertex_descriptor source{};
            vertex_descriptor target{};
            data_type data;

            constexpr edge() requires
                (std::default_initializable<data_type>)
                : data{} {}

            constexpr edge(vertex_descriptor source, vertex_descriptor target, data_type data)
                : source{std::move(source)}, target{std::move(target)}, data{std::move(data)} {}

            constexpr edge(const edge &other) = default;

            constexpr edge(edge &&other) noexcept = default;

            constexpr edge &operator=(const edge &other) = default;

            constexpr edge &operator=(edge &&other) noexcept = default;

            constexpr ~edge() = default;

            constexpr const data_type &get() const {
                return data;
            }

            constexpr data_type &get() {
                return data;
            }

            constexpr edge &set(data_type new_data) noexcept {
                data = std::move(new_data);
                return *this;
            }

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & source;
                ar & target;
                ar & data;
            }
        };

        template<std::same_as<void> Data>
        struct edge<Data> {
            using data_type = void;

            vertex_descriptor source{};
            vertex_descriptor target{};

            constexpr edge() = default;

            constexpr explicit edge(vertex_descriptor source, vertex_descriptor target)
                : source{std::move(source)}, target{std::move(target)} {}

            constexpr edge(const edge &other) = default;

            constexpr edge(edge &&other) noexcept = default;

            constexpr edge &operator=(const edge &other) = default;

            constexpr edge &operator=(edge &&other) noexcept = default;

            constexpr ~edge() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & source;
                ar & target;
            }
        };

        using edge_data_type = EdgeData;
        using edge_type = edge<edge_data_type>;
        using edge_container = edge_container_type<edge_type>;
        using edge_size_type = typename edge_container::size_type;
        using edge_difference_type = typename edge_container::difference_type;
        using edge_descriptor = edge_size_type;

        template<typename Data>
        struct vertex {
            using data_type = Data;

            edge_descriptor start{};
            data_type data;

            constexpr vertex() requires
                (std::default_initializable<data_type>)
                : data{} {}

            constexpr vertex(edge_descriptor start, data_type data)
                : start{std::move(start)}, data{std::move(data)} {}

            constexpr vertex(const vertex &other) = default;

            constexpr vertex(vertex &&other) noexcept = default;

            constexpr vertex &operator=(const vertex &other) = default;

            constexpr vertex &operator=(vertex &&other) noexcept = default;

            constexpr ~vertex() = default;

            constexpr const data_type &get() const {
                return data;
            }

            constexpr data_type &get() {
                return data;
            }

            constexpr vertex &set(data_type new_data) noexcept {
                data = std::move(new_data);
                return *this;
            }

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & start;
                ar & data;
            }
        };

        template<std::same_as<void> Data>
        struct vertex<Data> {
            using data_type = void;

            edge_descriptor start{};

            constexpr vertex() = default;

            constexpr explicit vertex(edge_descriptor start)
                : start{std::move(start)} {}

            constexpr vertex(const vertex &other) = default;

            constexpr vertex(vertex &&other) noexcept = default;

            constexpr vertex &operator=(const vertex &other) = default;

            constexpr vertex &operator=(vertex &&other) noexcept = default;

            constexpr ~vertex() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & start;
            }
        };

        constexpr compressed_sparse_row() requires std::default_initializable<allocator_type>
            : vertices{}, edges{} {}

        constexpr explicit compressed_sparse_row(const allocator_type &allocator)
            : vertices(allocator), edges(allocator) {}

        constexpr compressed_sparse_row(const compressed_sparse_row &other) = default;

        constexpr compressed_sparse_row(const compressed_sparse_row &other, const allocator_type &allocator)
            : vertices{other.vertices, allocator}, edges{other.edges, allocator} {};

        constexpr compressed_sparse_row(compressed_sparse_row &&other) noexcept = default;

        constexpr compressed_sparse_row(compressed_sparse_row &&other, const allocator_type &allocator) noexcept
            : vertices{std::move(other.vertices), allocator}, edges{std::move(other.edges), allocator} {}

        constexpr compressed_sparse_row &operator=(const compressed_sparse_row &other) = default;

        constexpr compressed_sparse_row &operator=(compressed_sparse_row &&other) noexcept = default;

        constexpr ~compressed_sparse_row() = default;

        template<is_adjacency_list AdjacencyList>
        void from_adjacency_list(const AdjacencyList &adjacency_list) requires
            (std::default_initializable<allocator_type>) {
            return from_adjacency_list(adjacency_list, allocator_type{});
        }

        template<is_adjacency_list AdjacencyList>
        void from_adjacency_list(const AdjacencyList &adjacency_list, const allocator_type &allocator) {
            const auto &vertex_converter = []<typename VertexDataT>(
                    const VertexDataT &vertex_data, const allocator_type &allocator) -> vertex_data_type {
                if constexpr (not std::is_void_v<vertex_data_type>) {
                    if constexpr (util::has_allocator_conversion_constructor<
                        vertex_data_type, VertexDataT, allocator_type>) {
                        return vertex_data_type{vertex_data, allocator};
                    } else {
                        return vertex_data_type{vertex_data};
                    }
                } else {
                    return;
                }
            };

            const auto &edge_converter = []<typename EdgeDataT>(
                    const EdgeDataT &edge_data, const allocator_type &allocator) -> edge_data_type {
                if constexpr (not std::is_void_v<edge_data_type>) {
                    if constexpr (util::has_allocator_conversion_constructor<
                        edge_data_type, EdgeDataT, allocator_type>) {
                        return edge_data_type{edge_data, allocator};
                    } else {
                        return edge_data_type{edge_data};
                    }
                } else {
                    return;
                }
            };

            return from_adjacency_list(adjacency_list, allocator, vertex_converter, edge_converter);
        }

        template<is_adjacency_list AdjacencyList, typename VertexConverter, typename EdgeConverter>
        void from_adjacency_list(const AdjacencyList &adjacency_list, const allocator_type &allocator,
                const VertexConverter &vertex_converter, const EdgeConverter &edge_converter) {
            constexpr bool is_undirected_v = AdjacencyList::is_undirected_v;

            const auto vertices_v = adjacency_list.vertices_view();
            const auto edges_v = adjacency_list.edges_view();

            reserve(vertices_v.size(), edges_v.size());

            vertex_size_type vertices_capacity = vertices.capacity();
            edge_size_type edges_capacity = edges.capacity();

            edge_descriptor edge_desc{0};

            for (auto v_it = vertices_v.begin(); v_it != vertices_v.end(); v_it = vertices_v.next(v_it)) {
                const auto &vertex = vertices_v[v_it];

                auto vertex_index = adjacency_list.vertex_index(v_it);

                vertex_descriptor vertex_desc = _add_vertex(edge_desc, vertex, allocator, vertex_converter);

                // vertex indices need to be the same, if not, adjacency_list did not receive a reindex
                assert(vertex_desc == vertex_index);

                const auto out_edges_v = vertex.out_edges_view();

                for (auto oe_it = out_edges_v.begin(); oe_it != out_edges_v.end(); oe_it = out_edges_v.next(oe_it)) {
                    auto e_it = out_edges_v[oe_it];
                    const auto &edge = edges_v[e_it];

                    auto source_index = adjacency_list.vertex_index(edge.source);
                    auto target_index = adjacency_list.vertex_index(edge.target);

                    if constexpr (is_undirected_v) {
                        if (source_index != vertex_index and target_index == vertex_index) {
                            // undirected reverse edge not supported, skipped
                            continue;
                        }
                    }

                    // edge source needs to be the same as the current vertex to that the edge is attached
                    assert(source_index == vertex_index);

                    // edge target needs at least to be within bounds
                    assert(target_index < vertices_v.size());

                    edge_desc = _add_edge(source_index, target_index, edge, allocator, edge_converter);

                    // increase to the next empty edge pointer for the start value for the next vertex
                    ++edge_desc;
                }
            }

            // capacity is not allowed to change during conversion
            assert(vertices.capacity() == vertices_capacity);
            assert(edges.capacity() == edges_capacity);
        }

        template<typename Vertex, typename VertexConverter>
        vertex_descriptor _add_vertex(edge_descriptor start, const Vertex &vertex, const allocator_type &allocator,
                const VertexConverter &vertex_converter) {
            vertex_descriptor index = vertices.size();
            if constexpr (std::is_void_v<vertex_data_type> or std::is_void_v<typename Vertex::data_type>) {
                vertices.emplace_back(vertex_type{start});
            } else {
                vertices.emplace_back(vertex_type{start, vertex_converter(vertex.get(), allocator)});
            }
            return index;
        }

        template<typename Edge, typename EdgeConverter>
        edge_descriptor _add_edge(vertex_descriptor source, vertex_descriptor target, const Edge &edge,
                const allocator_type &allocator, const EdgeConverter &edge_converter) {
            edge_descriptor index = edges.size();
            if constexpr (std::is_void_v<edge_data_type> or std::is_void_v<typename Edge::data_type>) {
                edges.emplace_back(edge_type{source, target});
            } else {
                edges.emplace_back(edge_type{source, target, edge_converter(edge.get(), allocator)});
            }
            return index;
        }

        void reserve(std::size_t num_vertices, std::size_t num_edges) {
            vertices.reserve(num_vertices);
            edges.reserve(num_edges);
        }

        [[nodiscard]] constexpr container_view<const vertex_container> vertices_view() const {
            return container_view<const vertex_container>{vertices};
        }

        [[nodiscard]] constexpr container_view<vertex_container> vertices_view() {
            return container_view<vertex_container>{vertices};
        }

        [[nodiscard]] constexpr container_view<const edge_container> edges_view() const {
            return container_view<const edge_container>{edges};
        }

        [[nodiscard]] constexpr container_view<edge_container> edges_view() {
            return container_view<edge_container>{edges};
        }

        [[nodiscard]] constexpr const vertex_type &get_vertex(vertex_descriptor vertex) const {
            const auto vertices_v = vertices_view();
            return vertices_v[vertex];
        }

        [[nodiscard]] constexpr vertex_type &get_vertex(vertex_descriptor vertex) {
            return vertices_view()[vertex];
        }

        [[nodiscard]] constexpr std::add_lvalue_reference_t<const vertex_data_type>
        get_vertex_data(vertex_descriptor vertex) const requires (not std::is_void_v<vertex_data_type>) {
            return get_vertex(vertex).get();
        }

        [[nodiscard]] constexpr std::add_lvalue_reference_t<vertex_data_type>
        get_vertex_data(vertex_descriptor vertex) requires (not std::is_void_v<vertex_data_type>) {
            return get_vertex(vertex).get();
        }

        [[nodiscard]] constexpr const edge_type &get_edge(edge_descriptor edge) const {
            const auto edges_v = edges_view();
            return edges_v[edge];
        }

        [[nodiscard]] constexpr edge_type &get_edge(edge_descriptor edge) {
            return edges_view()[edge];
        }

        [[nodiscard]] constexpr std::add_lvalue_reference_t<const edge_data_type>
        get_edge_data(edge_descriptor edge) const requires (not std::is_void_v<edge_data_type>) {
            return get_edge(edge).get();
        }

        [[nodiscard]] constexpr std::add_lvalue_reference_t<edge_data_type>
        get_edge_data(edge_descriptor edge) requires (not std::is_void_v<edge_data_type>) {
            return get_edge(edge).get();
        }

        [[nodiscard]] constexpr container_subview<const edge_container>
        out_edges_view(vertex_descriptor vertex) const {
            auto [start_e, end_e] = _out_edges_start_end(vertex);
            return container_subview<const edge_container>{edges, start_e, end_e};
        }

        [[nodiscard]] constexpr container_subview<edge_container> out_edges_view(vertex_descriptor vertex) {
            auto [start_e, end_e] = const_cast<const compressed_sparse_row *>(this)->_out_edges_start_end(vertex);
            return container_subview<edge_container>{edges, start_e, end_e};
        }

        [[nodiscard]] constexpr std::pair<edge_descriptor, edge_descriptor>
        _out_edges_start_end(vertex_descriptor vertex) const {
            const auto vertices_v = vertices_view();
            vertex_descriptor next_v = vertices_v.next(vertex);
            edge_descriptor start_e = vertices_v[vertex].start;
            edge_descriptor end_e = (next_v < vertices_v.size()) ? vertices_v[next_v].start : edges_view().size();
            return std::pair<edge_descriptor, edge_descriptor>{start_e, end_e};
        }

        [[nodiscard]] constexpr const vertex_descriptor &source(edge_descriptor edge) const {
            const auto edges_v = edges_view();
            return edges_v[edge].source;
        }

        [[nodiscard]] constexpr vertex_descriptor &source(edge_descriptor edge) {
            return edges_view()[edge].source;
        }

        [[nodiscard]] constexpr const vertex_descriptor &target(edge_descriptor edge) const {
            const auto edges_v = edges_view();
            return edges_v[edge].target;
        }

        [[nodiscard]] constexpr vertex_descriptor &target(edge_descriptor edge) {
            return edges_view()[edge].target;
        }

        [[nodiscard]] constexpr vertex_size_type out_degree(vertex_descriptor vertex) const {
            return out_edges_view(vertex).size();
        }

        [[nodiscard]] constexpr vertex_size_type vertex_index(vertex_descriptor vertex) const {
            return vertex;
        }

        [[nodiscard]] constexpr edge_size_type edge_index(edge_descriptor edge) const {
            return edge;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & vertices;
            ar & edges;
        }

        vertex_container vertices;
        edge_container edges;

    };

}

#endif //MAP_MATCHING_2_GRAPH_COMPRESSED_SPARSE_ROW_HPP
