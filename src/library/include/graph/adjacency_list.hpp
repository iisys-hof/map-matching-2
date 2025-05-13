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

#ifndef MAP_MATCHING_2_GRAPH_ADJACENCY_LIST_HPP
#define MAP_MATCHING_2_GRAPH_ADJACENCY_LIST_HPP

#include <list>
#include <functional>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>

#include "util/concepts.hpp"

#include "common.hpp"
#include "container_view.hpp"

namespace map_matching_2::graph {

    template<template<typename, typename> typename VertexContainer = std::list,
        template<typename, typename> typename OutEdgeContainer = std::list,
        typename Direction = directional,
        typename VertexData = void,
        typename EdgeData = void,
        template<typename, typename> typename EdgeContainer = std::list,
        typename Allocator = std::allocator<void>,
        template<typename> typename AllocatorTraits = std::allocator_traits>
    class adjacency_list {

    public:
        using allocator_type = Allocator;

        template<typename Type>
        using allocator_traits_type = AllocatorTraits<Type>;

        template<typename Type>
        using vertex_container_type = VertexContainer<Type,
            typename allocator_traits_type<allocator_type>::template rebind_alloc<Type>>;

        template<typename Type>
        using out_edge_container_type = OutEdgeContainer<Type,
            typename allocator_traits_type<allocator_type>::template rebind_alloc<Type>>;

        template<typename Type>
        using edge_container_type = EdgeContainer<Type,
            typename allocator_traits_type<allocator_type>::template rebind_alloc<Type>>;

        using direction_type = Direction;

        using is_undirected = std::is_base_of<undirected, direction_type>;
        static constexpr bool is_undirected_v = is_undirected::value;

        using is_directional = std::is_base_of<directional, direction_type>;
        static constexpr bool is_directional_v = is_directional::value;

        using is_bidirectional = std::is_base_of<bidirectional, direction_type>;
        static constexpr bool is_bidirectional_v = is_bidirectional::value;

        template<typename Data>
        struct vertex;

        template<typename Data>
        struct bi_vertex;

        using vertex_data_type = VertexData;
        using vertex_type = std::conditional_t<is_bidirectional_v,
            bi_vertex<vertex_data_type>, vertex<vertex_data_type>>;
        using vertex_container = vertex_container_type<vertex_type>;
        using vertex_size_type = typename vertex_container::size_type;
        using vertex_difference_type = typename vertex_container::difference_type;
        using vertex_iterator = typename vertex_container::iterator;
        using vertex_const_iterator = typename vertex_container::const_iterator;
        using vertex_descriptor = std::conditional_t<util::is_random_access_v<vertex_container>,
            vertex_size_type, vertex_iterator>;
        using vertex_const_descriptor = std::conditional_t<util::is_random_access_v<vertex_container>,
            vertex_size_type, vertex_const_iterator>;

        template<typename Data>
        struct edge;

        using edge_data_type = EdgeData;
        using edge_type = edge<edge_data_type>;
        using edge_container = edge_container_type<edge_type>;
        using edge_size_type = typename edge_container::size_type;
        using edge_difference_type = typename edge_container::difference_type;
        using edge_iterator = typename edge_container::iterator;
        using edge_const_iterator = typename edge_container::const_iterator;
        using edge_descriptor = std::conditional_t<util::is_random_access_v<edge_container>,
            edge_size_type, edge_iterator>;
        using edge_const_descriptor = std::conditional_t<util::is_random_access_v<edge_container>,
            edge_size_type, edge_const_iterator>;

        struct base_edge {
            vertex_descriptor source{};
            vertex_descriptor target{};

            constexpr base_edge() = default;

            constexpr base_edge(vertex_descriptor source, vertex_descriptor target)
                : source{std::move(source)}, target{std::move(target)} {}

            constexpr base_edge(const base_edge &other) = default;

            constexpr base_edge(base_edge &&other) noexcept = default;

            constexpr base_edge &operator=(const base_edge &other) = default;

            constexpr base_edge &operator=(base_edge &&other) noexcept = default;

            constexpr ~base_edge() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & source;
                ar & target;
            }
        };

        struct base_index_edge : base_edge {
            edge_size_type index{};

            constexpr base_index_edge() = default;

            constexpr base_index_edge(edge_size_type index, vertex_descriptor source, vertex_descriptor target)
                : base_edge{std::move(source), std::move(target)}, index{std::move(index)} {}

            constexpr base_index_edge(const base_index_edge &other) = default;

            constexpr base_index_edge(base_index_edge &&other) noexcept = default;

            constexpr base_index_edge &operator=(const base_index_edge &other) = default;

            constexpr base_index_edge &operator=(base_index_edge &&other) noexcept = default;

            constexpr ~base_index_edge() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & boost::serialization::base_object<base_edge>(*this);
                ar & index;
            }
        };

        using base_edge_type = std::conditional_t<util::is_random_access_v<edge_container>,
            base_edge, base_index_edge>;

        template<typename Data>
        struct edge : base_edge_type {
            using data_type = Data;

            data_type data;

            constexpr edge() requires
                (std::default_initializable<data_type>)
                : base_edge_type{}, data{} {}

            constexpr edge(vertex_descriptor source, vertex_descriptor target, data_type data) requires
                (util::is_random_access_v<edge_container>)
                : base_edge_type{std::move(source), std::move(target)}, data{std::move(data)} {}

            constexpr edge(edge_size_type index, vertex_descriptor source, vertex_descriptor target,
                    data_type data) requires
                (not util::is_random_access_v<edge_container>)
                : base_edge_type{std::move(index), std::move(source), std::move(target)}, data{std::move(data)} {}

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
                ar & boost::serialization::base_object<base_edge_type>(*this);
                ar & data;
            }
        };

        template<std::same_as<void> Data>
        struct edge<Data> : base_edge_type {
            using data_type = void;

            constexpr edge() = default;

            constexpr edge(vertex_descriptor source, vertex_descriptor target) requires
                (util::is_random_access_v<edge_container>)
                : base_edge_type{std::move(source), std::move(target)} {}

            constexpr edge(edge_size_type index, vertex_descriptor source, vertex_descriptor target) requires
                (not util::is_random_access_v<edge_container>)
                : base_edge_type{std::move(index), std::move(source), std::move(target)} {}

            constexpr edge(const edge &other) = default;

            constexpr edge(edge &&other) noexcept = default;

            constexpr edge &operator=(const edge &other) = default;

            constexpr edge &operator=(edge &&other) noexcept = default;

            constexpr ~edge() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & boost::serialization::base_object<base_edge_type>(*this);
            }

        };

        using out_edge_container = out_edge_container_type<edge_descriptor>;
        using out_edge_size_type = typename out_edge_container::size_type;
        using out_edge_difference_type = typename out_edge_container::difference_type;
        using out_edge_iterator = typename out_edge_container::iterator;
        using out_edge_const_iterator = typename out_edge_container::const_iterator;
        using out_edge_descriptor = std::conditional_t<util::is_random_access_v<out_edge_container>,
            out_edge_size_type, out_edge_iterator>;
        using out_edge_const_descriptor = std::conditional_t<util::is_random_access_v<out_edge_container>,
            out_edge_size_type, out_edge_const_iterator>;

        using in_edge_container = out_edge_container;
        using in_edge_size_type = typename in_edge_container::size_type;
        using in_edge_difference_type = typename in_edge_container::difference_type;
        using in_edge_iterator = typename in_edge_container::iterator;
        using in_edge_const_iterator = typename in_edge_container::const_iterator;
        using in_edge_descriptor = std::conditional_t<util::is_random_access_v<in_edge_container>,
            in_edge_size_type, in_edge_iterator>;
        using in_edge_const_descriptor = std::conditional_t<util::is_random_access_v<in_edge_container>,
            in_edge_size_type, in_edge_const_iterator>;

        struct base_vertex {
            out_edge_container out_edges;

            constexpr base_vertex() requires
                (std::default_initializable<allocator_type>)
                : out_edges{} {}

            constexpr explicit base_vertex(const allocator_type &allocator)
                : out_edges(allocator) {}

            constexpr base_vertex(const base_vertex &other) = default;

            constexpr base_vertex(const base_vertex &other, const allocator_type &allocator)
                : out_edges{other.out_edges, allocator} {}

            constexpr base_vertex(base_vertex &&other) noexcept = default;

            constexpr base_vertex(base_vertex &&other, const allocator_type &allocator) noexcept
                : out_edges{std::move(other.out_edges), allocator} {}

            constexpr base_vertex &operator=(const base_vertex &other) = default;

            constexpr base_vertex &operator=(base_vertex &&other) noexcept = default;

            constexpr ~base_vertex() = default;

            [[nodiscard]] constexpr container_view<const out_edge_container> out_edges_view() const {
                return container_view<const out_edge_container>{out_edges};
            }

            [[nodiscard]] constexpr container_view<out_edge_container> out_edges_view() {
                return container_view<out_edge_container>{out_edges};
            }

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & out_edges;
            }
        };

        struct base_index_vertex : base_vertex {
            vertex_size_type index{};

            constexpr base_index_vertex() requires
                (std::default_initializable<allocator_type>)
                : base_vertex{} {}

            constexpr explicit base_index_vertex(vertex_size_type index) requires
                (std::default_initializable<allocator_type>)
                : base_vertex{}, index{std::move(index)} {}

            constexpr base_index_vertex(vertex_size_type index, const allocator_type &allocator)
                : base_vertex{allocator}, index{std::move(index)} {}

            constexpr base_index_vertex(const base_index_vertex &other) = default;

            constexpr base_index_vertex(const base_index_vertex &other, const allocator_type &allocator)
                : base_vertex{other, allocator}, index{other.index} {}

            constexpr base_index_vertex(base_index_vertex &&other) noexcept = default;

            constexpr base_index_vertex(base_index_vertex &&other, const allocator_type &allocator) noexcept
                : base_vertex{std::move(other), allocator}, index{std::move(other.index)} {}

            constexpr base_index_vertex &operator=(const base_index_vertex &other) = default;

            constexpr base_index_vertex &operator=(base_index_vertex &&other) noexcept = default;

            constexpr ~base_index_vertex() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & boost::serialization::base_object<base_vertex>(*this);
                ar & index;
            }
        };

        using base_vertex_type = std::conditional_t<util::is_random_access_v<vertex_container>,
            base_vertex, base_index_vertex>;

        template<typename Data>
        struct vertex : base_vertex_type {
            using data_type = Data;

            data_type data;

            constexpr vertex() requires
                (std::default_initializable<data_type> and std::default_initializable<allocator_type> and
                    util::is_random_access_v<vertex_container>)
                : base_vertex_type{}, data{} {}

            constexpr explicit vertex(data_type data) requires
                (std::default_initializable<allocator_type> and util::is_random_access_v<vertex_container>)
                : base_vertex_type{}, data{std::move(data)} {}

            constexpr vertex(data_type data, const allocator_type &allocator) requires
                (util::is_random_access_v<vertex_container>)
                : base_vertex_type{allocator}, data{std::move(data)} {}

            constexpr explicit vertex(vertex_size_type index) requires
                (std::default_initializable<data_type> and std::default_initializable<allocator_type> and
                    not util::is_random_access_v<vertex_container>)
                : base_vertex_type{std::move(index)}, data{} {}

            constexpr vertex(vertex_size_type index, data_type data) requires
                (std::default_initializable<allocator_type> and not util::is_random_access_v<vertex_container>)
                : base_vertex_type{std::move(index)}, data{std::move(data)} {}

            constexpr vertex(vertex_size_type index, data_type data, const allocator_type &allocator) requires
                (not util::is_random_access_v<vertex_container>)
                : base_vertex_type{std::move(index), allocator}, data{std::move(data)} {}

            constexpr vertex(const vertex &other) = default;

            constexpr vertex(const vertex &other, const allocator_type &allocator)
                : base_vertex_type{other, allocator}, data{other.data} {}

            constexpr vertex(vertex &&other) noexcept = default;

            constexpr vertex(vertex &&other, const allocator_type &allocator) noexcept
                : base_vertex_type{std::move(other), allocator}, data{std::move(other.data)} {}

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
                ar & boost::serialization::base_object<base_vertex_type>(*this);
                ar & data;
            }
        };

        template<std::same_as<void> Data>
        struct vertex<Data> : base_vertex_type {
            using data_type = void;

            constexpr vertex() requires
                (std::default_initializable<allocator_type> and util::is_random_access_v<vertex_container>) = default;

            constexpr explicit vertex(const allocator_type &allocator) requires
                (util::is_random_access_v<vertex_container>)
                : base_vertex_type{allocator} {}

            constexpr explicit vertex(vertex_size_type index) requires
                (std::default_initializable<allocator_type> and not util::is_random_access_v<vertex_container>)
                : base_vertex_type{std::move(index)} {}

            constexpr vertex(vertex_size_type index, const allocator_type &allocator) requires
                (not util::is_random_access_v<vertex_container>)
                : base_vertex_type{std::move(index), allocator} {}

            constexpr vertex(const vertex &other) = default;

            constexpr vertex(const vertex &other, const allocator_type &allocator)
                : base_vertex_type{other, allocator} {}

            constexpr vertex(vertex &&other) noexcept = default;

            constexpr vertex(vertex &&other, const allocator_type &allocator) noexcept
                : base_vertex_type{std::move(other), allocator} {}

            constexpr vertex &operator=(const vertex &other) = default;

            constexpr vertex &operator=(vertex &&other) noexcept = default;

            constexpr ~vertex() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & boost::serialization::base_object<base_vertex_type>(*this);
            }
        };

        struct base_bi_vertex {
            in_edge_container in_edges;

            constexpr base_bi_vertex() requires
                (std::default_initializable<allocator_type>)
                : in_edges{} {}

            constexpr explicit base_bi_vertex(const allocator_type &allocator)
                : in_edges(allocator) {}

            constexpr base_bi_vertex(const base_bi_vertex &other) = default;

            constexpr base_bi_vertex(const base_bi_vertex &other, const allocator_type &allocator)
                : in_edges{other.in_edges, allocator} {}

            constexpr base_bi_vertex(base_bi_vertex &&other) = default;

            constexpr base_bi_vertex(base_bi_vertex &&other, const allocator_type &allocator) noexcept
                : in_edges{std::move(other.in_edges), allocator} {}

            constexpr base_bi_vertex &operator=(const base_bi_vertex &other) = default;

            constexpr base_bi_vertex &operator=(base_bi_vertex &&other) noexcept = default;

            constexpr ~base_bi_vertex() = default;

            [[nodiscard]] constexpr container_view<const in_edge_container> in_edges_view() const {
                return container_view<const in_edge_container>{in_edges};
            }

            [[nodiscard]] constexpr container_view<in_edge_container> in_edges_view() {
                return container_view<in_edge_container>{in_edges};
            }

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & in_edges;
            }
        };

        template<typename Data>
        struct bi_vertex : base_bi_vertex, vertex<Data> {
            using data_type = Data;

            constexpr bi_vertex() requires
                (std::default_initializable<data_type> and std::default_initializable<allocator_type> &&
                    util::is_random_access_v<vertex_container>) = default;

            constexpr explicit bi_vertex(data_type data) requires
                (std::default_initializable<allocator_type> and util::is_random_access_v<vertex_container>)
                : base_bi_vertex{}, vertex<Data>{std::move(data)} {}

            constexpr bi_vertex(data_type data, const allocator_type &allocator) requires
                (util::is_random_access_v<vertex_container>)
                : base_bi_vertex{allocator}, vertex<Data>{std::move(data), allocator} {}

            constexpr explicit bi_vertex(vertex_size_type index) requires
                (std::default_initializable<data_type> and std::default_initializable<allocator_type> and
                    not util::is_random_access_v<vertex_container>)
                : base_bi_vertex{}, vertex<Data>{std::move(index)} {}

            constexpr bi_vertex(vertex_size_type index, data_type data) requires
                (std::default_initializable<allocator_type> and not util::is_random_access_v<vertex_container>)
                : base_bi_vertex{}, vertex<Data>{std::move(index), std::move(data)} {}

            constexpr bi_vertex(vertex_size_type index, data_type data, const allocator_type &allocator) requires
                (not util::is_random_access_v<vertex_container>)
                : base_bi_vertex{allocator}, vertex<Data>{std::move(index), std::move(data), allocator} {}

            constexpr bi_vertex(const bi_vertex &other) = default;

            constexpr bi_vertex(const bi_vertex &other, const allocator_type &allocator)
                : base_bi_vertex{other, allocator}, vertex<Data>{other, allocator} {}

            constexpr bi_vertex(bi_vertex &&other) noexcept = default;

            constexpr bi_vertex(bi_vertex &&other, const allocator_type &allocator) noexcept
                : base_bi_vertex{std::move(other), allocator}, vertex<Data>{std::move(other), allocator} {}

            constexpr bi_vertex &operator=(const bi_vertex &other) = default;

            constexpr bi_vertex &operator=(bi_vertex &&other) noexcept = default;

            constexpr ~bi_vertex() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & boost::serialization::base_object<base_bi_vertex>(*this);
                ar & boost::serialization::base_object<vertex<Data>>(*this);
            }
        };

        template<std::same_as<void> Data>
        struct bi_vertex<Data> : base_bi_vertex, vertex<Data> {
            using data_type = void;

            constexpr bi_vertex() requires
                (std::default_initializable<allocator_type> and util::is_random_access_v<vertex_container>) = default;

            constexpr explicit bi_vertex(const allocator_type &allocator) requires
                (util::is_random_access_v<vertex_container>)
                : base_bi_vertex{allocator}, vertex<Data>{allocator} {}

            constexpr explicit bi_vertex(vertex_size_type index) requires
                (std::default_initializable<allocator_type> and not util::is_random_access_v<vertex_container>)
                : base_bi_vertex{}, vertex<Data>{std::move(index)} {}

            constexpr bi_vertex(vertex_size_type index, const allocator_type &allocator) requires
                (not util::is_random_access_v<vertex_container>)
                : base_bi_vertex{allocator}, vertex<Data>{std::move(index), allocator} {}

            constexpr bi_vertex(const bi_vertex &other) = default;

            constexpr bi_vertex(const bi_vertex &other, const allocator_type &allocator)
                : base_bi_vertex{other, allocator}, vertex<Data>{other, allocator} {}

            constexpr bi_vertex(bi_vertex &&other) noexcept = default;

            constexpr bi_vertex(bi_vertex &&other, const allocator_type &allocator) noexcept
                : base_bi_vertex{std::move(other), allocator}, vertex<Data>{std::move(other), allocator} {}

            constexpr bi_vertex &operator=(const bi_vertex &other) = default;

            constexpr bi_vertex &operator=(bi_vertex &&other) noexcept = default;

            constexpr ~bi_vertex() = default;

            friend class boost::serialization::access;

            template<typename Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & boost::serialization::base_object<base_bi_vertex>(*this);
                ar & boost::serialization::base_object<vertex<Data>>(*this);
            }
        };

        constexpr adjacency_list() requires std::default_initializable<allocator_type>
            : vertices{}, edges{} {}

        constexpr explicit adjacency_list(const allocator_type &allocator)
            : vertices(allocator), edges(allocator) {}

        adjacency_list(const adjacency_list &other)
            : vertices(other.vertices.get_allocator()), edges(other.edges.get_allocator()) {
            copy_from(other);
        }

        adjacency_list(const adjacency_list &other, const allocator_type &allocator)
            : vertices(allocator), edges(allocator) {
            copy_from(other);
        }

        constexpr adjacency_list(adjacency_list &&other) noexcept = default;

        constexpr adjacency_list(adjacency_list &&other, const allocator_type &allocator) noexcept
            : vertices{std::move(other.vertices), allocator}, edges{std::move(other.edges), allocator} {}

        adjacency_list &operator=(const adjacency_list &other) {
            if (this != &other) {
                vertices = vertex_container(other.vertices.get_allocator());
                edges = edge_container(other.edges.get_allocator());
                copy_from(other);
            }
            return *this;
        }

        constexpr adjacency_list &operator=(adjacency_list &&other) noexcept = default;

        constexpr ~adjacency_list() = default;

        template<typename AdjacencyList,
            template<typename, typename> typename VertexMap = std::vector,
            template<typename> typename VertexAllocator = std::allocator>
        void copy_from(const AdjacencyList &other,
                VertexMap<vertex_descriptor, VertexAllocator<vertex_descriptor>> &vertex_map) {
            const auto vertices_v = other.vertices_view();

            if (vertices_v.size() > vertex_map.size()) {
                vertex_map.assign(other.vertices_view().size(), vertex_descriptor{});
            }

            for (auto v_it = vertices_v.begin(); v_it != vertices_v.end(); v_it = vertices_v.next(v_it)) {
                const auto &o_vertex = vertices_v[v_it];

                vertex_descriptor vertex_desc;
                if constexpr (std::is_void_v<vertex_data_type>) {
                    vertex_desc = add_vertex();
                } else {
                    vertex_desc = add_vertex(o_vertex.get());
                }

                const auto o_vi = other.vertex_index(v_it);
                const auto vi = vertex_index(vertex_desc);
                // vertex indices need to be the same, if not, other did not receive a reindex
                assert(vi == o_vi);

                vertex_map[vi] = vertex_desc;
            }

            // check if all vertices were copied
            assert(vertices_view().size() == vertices_v.size());

            const auto edges_v = other.edges_view();
            for (auto e_it = edges_v.begin(); e_it != edges_v.end(); e_it = edges_v.next(e_it)) {
                const auto &o_edge = edges_v[e_it];

                const auto si = other.vertex_index(o_edge.source);
                const auto ti = other.vertex_index(o_edge.target);

                edge_descriptor edge_desc;
                if constexpr (std::is_void_v<edge_data_type>) {
                    edge_desc = add_edge(vertex_map[si], vertex_map[ti]);
                } else {
                    edge_desc = add_edge(vertex_map[si], vertex_map[ti], o_edge.get());
                }

                const auto o_ei = other.edge_index(e_it);
                const auto ei = edge_index(edge_desc);
                // edge indices need to be the same, if not, other did not receive a reindex
                assert(ei == o_ei);
            }

            // check if all edges were copied
            assert(edges_view().size() == edges_v.size());
        }

        template<typename AdjacencyList,
            template<typename, typename> typename VertexMap = std::vector>
        void copy_from(const AdjacencyList &other) {
            VertexMap<vertex_descriptor, std::allocator<vertex_descriptor>> vertex_map(
                    other.vertices_view().size(), vertex_descriptor{});
            copy_from<AdjacencyList, VertexMap>(other, vertex_map);
        }

        vertex_descriptor add_vertex() requires
            (std::is_void_v<vertex_data_type> and util::is_random_access_v<vertex_container>) {
            return _add_vertex(vertex_type{vertices.get_allocator()});
        }

        vertex_descriptor add_vertex() requires
            (std::is_void_v<vertex_data_type> and not util::is_random_access_v<vertex_container>) {
            return _add_vertex(vertex_type{vertices.size(), vertices.get_allocator()});
        }

        vertex_descriptor add_vertex(auto &&data) requires
            (not std::is_void_v<vertex_data_type> and util::is_random_access_v<vertex_container>) {
            return _add_vertex(vertex_type{std::forward<decltype(data)>(data), vertices.get_allocator()});
        }

        vertex_descriptor add_vertex(auto &&data) requires
            (not std::is_void_v<vertex_data_type> and not util::is_random_access_v<vertex_container>) {
            return _add_vertex(vertex_type{
                    vertices.size(),
                    std::forward<decltype(data)>(data), vertices.get_allocator()
            });
        }

        [[nodiscard]] vertex_descriptor _add_vertex(vertex_type &&vertex) {
            const auto &additional_tidy = []() constexpr -> void {};
            return try_add(vertices, additional_tidy, std::move(vertex));
        }

        edge_descriptor add_edge(const vertex_descriptor &source_desc, const vertex_descriptor &target_desc) requires
            (std::is_void_v<edge_data_type> and util::is_random_access_v<edge_container>) {
            return _add_edge(source_desc, target_desc, edge_type{source_desc, target_desc});
        }

        edge_descriptor add_edge(const vertex_descriptor &source_desc, const vertex_descriptor &target_desc) requires
            (std::is_void_v<edge_data_type> and not util::is_random_access_v<edge_container>) {
            return _add_edge(source_desc, target_desc, edge_type{edges.size(), source_desc, target_desc});
        }

        edge_descriptor add_edge(
                const vertex_descriptor &source_desc, const vertex_descriptor &target_desc, auto &&data) requires
            (not std::is_void_v<edge_data_type> and util::is_random_access_v<edge_container>) {
            return _add_edge(source_desc, target_desc,
                    edge_type{source_desc, target_desc, std::forward<decltype(data)>(data)});
        }

        edge_descriptor add_edge(
                const vertex_descriptor &source_desc, const vertex_descriptor &target_desc, auto &&data) requires
            (not std::is_void_v<edge_data_type> and not util::is_random_access_v<edge_container>) {
            return _add_edge(source_desc, target_desc,
                    edge_type{edges.size(), source_desc, target_desc, std::forward<decltype(data)>(data)});
        }

        [[nodiscard]] edge_descriptor _add_edge(
                const vertex_descriptor &source_desc, const vertex_descriptor &target_desc, edge_type &&edge) {
            const auto index = vertex_index(source_desc);
            // std::cout << "From " << vertex_index(source_desc) << " to " << vertex_index(target_desc) << std::endl;
            // std::cout << "Size Before: " << get_vertex(source_desc).out_edges.size() << std::endl;
            const auto &additional_tidy = []() constexpr -> void {};
            edge_descriptor edge_desc = try_add(edges, additional_tidy, std::move(edge));

            edge_type &edge_r = get_edge(edge_desc);
            auto edges_tidy = [&]() {
                auto edges_v = edges_view();
                if (not edges.empty() and std::addressof(edges.back()) == std::addressof(edge_r)) {
                    edges.pop_back();
                }
            };

            vertex_type &source_vertex = get_vertex(source_desc);
            try_add(source_vertex.out_edges, edges_tidy, edge_descriptor{edge_desc});

            if constexpr (is_undirected_v or is_bidirectional_v) {
                auto source_out_edges_tidy = [&]() {
                    if (not source_vertex.out_edges.empty() and
                        std::addressof(get_edge(source_vertex.out_edges.back())) == std::addressof(edge_r)) {
                        source_vertex.out_edges.pop_back();
                    }
                    edges_tidy();
                };

                vertex_type &target_vertex = get_vertex(target_desc);

                if constexpr (is_undirected_v) {
                    if (std::addressof(source_vertex) != std::addressof(target_vertex)) {
                        try_add(target_vertex.out_edges, source_out_edges_tidy, edge_descriptor{edge_desc});
                    }
                }

                if constexpr (is_bidirectional_v) {
                    auto target_out_edges_tidy = [&]() {
                        if constexpr (is_undirected_v) {
                            if (not target_vertex.out_edges.empty() and
                                std::addressof(get_edge(target_vertex.out_edges.back())) == std::addressof(edge_r)) {
                                target_vertex.out_edges.pop_back();
                            }
                        }
                        source_out_edges_tidy();
                    };

                    try_add(target_vertex.in_edges, target_out_edges_tidy, edge_descriptor{edge_desc});
                }
            }

            return edge_desc;
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

        [[nodiscard]] constexpr const vertex_type &get_vertex(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex) const {
            const auto vertices_v = vertices_view();
            return vertices_v[vertex];
        }

        [[nodiscard]] constexpr vertex_type &get_vertex(const vertex_descriptor &vertex) {
            return vertices_view()[vertex];
        }

        [[nodiscard]] constexpr std::add_lvalue_reference_t<const vertex_data_type> get_vertex_data(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex) const requires
            (not std::is_void_v<vertex_data_type>) {
            return get_vertex(vertex).get();
        }

        [[nodiscard]] constexpr std::add_lvalue_reference_t<vertex_data_type> get_vertex_data(
                const vertex_descriptor &vertex) requires
            (not std::is_void_v<vertex_data_type>) {
            return get_vertex(vertex).get();
        }

        [[nodiscard]] constexpr const edge_type &get_edge(
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge) const {
            const auto edges_v = edges_view();
            return edges_v[edge];
        }

        [[nodiscard]] constexpr edge_type &get_edge(const edge_descriptor &edge) {
            return edges_view()[edge];
        }

        [[nodiscard]] constexpr std::add_lvalue_reference_t<const edge_data_type> get_edge_data(
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge) const requires
            (not std::is_void_v<edge_data_type>) {
            return get_edge(edge).get();
        }

        [[nodiscard]] constexpr std::add_lvalue_reference_t<edge_data_type> get_edge_data(
                const edge_descriptor &edge) requires (not std::is_void_v<edge_data_type>) {
            return get_edge(edge).get();
        }

        [[nodiscard]] constexpr container_view<const out_edge_container> out_edges_view(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex) const {
            return get_vertex(vertex).out_edges_view();
        }

        [[nodiscard]] constexpr container_view<out_edge_container> out_edges_view(const vertex_descriptor &vertex) {
            return get_vertex(vertex).out_edges_view();
        }

        [[nodiscard]] constexpr container_view<const in_edge_container> in_edges_view(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex) const
            requires is_bidirectional_v {
            return get_vertex(vertex).in_edges_view();
        }

        [[nodiscard]] constexpr container_view<in_edge_container> in_edges_view(
                const vertex_descriptor &vertex) requires is_bidirectional_v {
            return get_vertex(vertex).in_edges_view();
        }

        [[nodiscard]] constexpr const vertex_descriptor &source(
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge) const {
            return get_edge(edge).source;
        }

        [[nodiscard]] constexpr vertex_descriptor &source(const edge_descriptor &edge) {
            return get_edge(edge).source;
        }

        [[nodiscard]] constexpr const vertex_descriptor &target(
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge) const {
            return get_edge(edge).target;
        }

        [[nodiscard]] constexpr vertex_descriptor &target(const edge_descriptor &edge) {
            return get_edge(edge).target;
        }

        [[nodiscard]] constexpr vertex_size_type out_degree(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex) const {
            return out_edges_view(vertex).size();
        }

        [[nodiscard]] constexpr vertex_size_type in_degree(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex) const requires
            is_bidirectional_v {
            return in_edges_view(vertex).size();
        }

        [[nodiscard]] constexpr vertex_size_type degree(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex) const requires
            is_bidirectional_v {
            return out_degree(vertex) + in_degree(vertex);
        }

        [[nodiscard]] constexpr vertex_size_type vertex_index(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex) const {
            if constexpr (util::is_random_access_v<vertex_container>) {
                return vertex;
            } else {
                return get_vertex(vertex).index;
            }
        }

        [[nodiscard]] constexpr edge_size_type edge_index(
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge) const {
            if constexpr (util::is_random_access_v<edge_container>) {
                return edge;
            } else {
                return get_edge(edge).index;
            }
        }

        template<typename Container, typename AdditionalTidy, typename... Args> requires
            requires(Container container, Args &&... args) {
                { container.emplace_back(std::forward<Args>(args)...) };
                { container.pop_back() };
            }
        std::conditional_t<util::is_random_access_v<Container>,
            typename Container::size_type, typename Container::iterator>
        try_add(Container &container, const AdditionalTidy &additional_tidy, Args &&... args) {
            typename Container::size_type current_size = container.size();
            try {
                container.emplace_back(std::forward<Args>(args)...);
            } catch (...) {
                while (container.size() > current_size) {
                    container.pop_back();
                }
                additional_tidy();
                throw;
            }

            if constexpr (util::is_random_access_v<Container>) {
                return current_size;
            } else {
                return std::prev(std::end(container));
            }
        }

        void reindex() requires
            (not util::is_random_access_v<vertex_container> and not util::is_random_access_v<edge_container>) {
            if constexpr (not util::is_random_access_v<vertex_container>) {
                vertex_size_type vertex_index{0};
                auto vertices_v = vertices_view();
                for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                    vertices_v[it].index = vertex_index++;
                }
            }

            if constexpr (not util::is_random_access_v<edge_container>) {
                edge_size_type edge_index{0};
                auto edges_v = edges_view();
                for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                    edges_v[it].index = edge_index++;
                }
            }
        }

        [[nodiscard]] constexpr bool is_same_vertex(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex_a,
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex_b) const {
            return std::addressof(get_vertex(vertex_a)) == std::addressof(get_vertex(vertex_b));
        }

        [[nodiscard]] constexpr bool is_same_edge(
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge_a,
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge_b) const {
            return std::addressof(get_edge(edge_a)) == std::addressof(get_edge(edge_b));
        }

        [[nodiscard]] constexpr bool is_self_loop(
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge) const {
            return std::addressof(get_vertex(source(edge))) == std::addressof(get_vertex(target(edge)));
        }

        [[nodiscard]] constexpr bool is_undirected_reverse(
                util::is_any_of<vertex_descriptor, vertex_const_descriptor> auto vertex,
                util::is_any_of<edge_descriptor, edge_const_descriptor> auto edge) const requires
            (is_undirected_v) {
            return not is_self_loop(edge) and
                    std::addressof(get_vertex(target(edge))) == std::addressof(get_vertex(vertex));
        }

        void remove_vertex(const vertex_descriptor &vertex) requires
            (not util::is_random_access_v<vertex_container>) {
            if (not out_edges_view(vertex).empty()) {
                throw std::runtime_error("vertex is still connected.");
            }

            vertices.erase(vertex);
        }

        void clear_vertex(const vertex_descriptor &vertex_desc) requires
            (not util::is_random_access_v<vertex_container> and
                not util::is_random_access_v<edge_container> and
                not util::is_random_access_v<out_edge_container> and
                (not is_bidirectional_v or not util::is_random_access_v<in_edge_container>)) {
            std::vector<edge_descriptor> delete_edges;

            vertex_type &vertex = get_vertex(vertex_desc);

            out_edge_container &out_edges = vertex.out_edges;
            for (const edge_descriptor &edge_desc : out_edges) {

                if (not is_self_loop(edge_desc)) {
                    if constexpr (is_undirected_v) {
                        if (is_undirected_reverse(vertex_desc, edge_desc)) {
                            out_remove_if(source(edge_desc), [&](const edge_descriptor &source_out_edge_desc) {
                                return is_same_vertex(target(source_out_edge_desc), vertex_desc) and
                                        is_same_edge(source_out_edge_desc, edge_desc);
                            });
                        }
                    }

                    if constexpr (is_bidirectional_v) {
                        in_remove_if(target(edge_desc), [&](const edge_descriptor &target_in_edge_desc) {
                            return is_same_vertex(source(target_in_edge_desc), vertex_desc) and
                                    is_same_edge(target_in_edge_desc, edge_desc);
                        });
                    }

                    out_remove_if(target(edge_desc), [&](const edge_descriptor &target_out_edge_desc) {
                        return is_same_vertex(source(target_out_edge_desc), vertex_desc) and
                                is_same_edge(target_out_edge_desc, edge_desc);
                    });
                }

                delete_edges.emplace_back(edge_desc);
            }

            if constexpr (is_bidirectional_v) {
                vertex.in_edges.clear();
            }

            out_edges.clear();

            if constexpr (is_directional_v) {
                // ingoing edges to the vertex to be cleared can only be found by traversing all vertices
                auto vertices_v = vertices_view();
                for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                    out_remove_if(it, [&](const edge_descriptor &out_edge_desc) {
                        if (is_same_vertex(target(out_edge_desc), vertex_desc)) {
                            delete_edges.emplace_back(out_edge_desc);
                            return true;
                        }
                        return false;
                    });
                }
            }

            for (auto &delete_edge : delete_edges) {
                edges.erase(delete_edge);
            }
        }

        void remove_edge(const vertex_descriptor &source_desc, const vertex_descriptor &target_desc) requires
            (not util::is_random_access_v<edge_container> and
                not util::is_random_access_v<out_edge_container> and
                (not is_bidirectional_v or not util::is_random_access_v<in_edge_container>)) {
            std::vector<edge_descriptor> delete_edges;

            vertex_type &source_vertex = get_vertex(source_desc);

            out_edge_container &out_edges = source_vertex.out_edges;
            out_edge_descriptor out_edge_desc = out_edges.begin();
            while (out_edge_desc != out_edges.end()) {
                const edge_descriptor &source_out_edge_desc = *out_edge_desc;

                const vertex_descriptor &out_edge_source_desc = source(source_out_edge_desc);
                const vertex_descriptor &out_edge_target_desc = target(source_out_edge_desc);

                if (is_same_vertex(out_edge_source_desc, source_desc) and
                    is_same_vertex(out_edge_target_desc, target_desc)) {
                    if constexpr (is_bidirectional_v) {
                        in_remove_if(out_edge_target_desc, [&](const edge_descriptor &target_in_edge_desc) {
                            return is_same_edge(target_in_edge_desc, source_out_edge_desc);
                        });
                    }

                    if constexpr (is_undirected_v) {
                        if (not is_same_vertex(source_desc, target_desc)) {
                            out_remove_if(out_edge_target_desc, [&](const edge_descriptor &target_out_edge_desc) {
                                return is_same_edge(target_out_edge_desc, source_out_edge_desc);
                            });
                        }
                    }

                    delete_edges.emplace_back(source_out_edge_desc);

                    out_edge_desc = out_edges.erase(out_edge_desc);
                } else {
                    ++out_edge_desc;
                }
            }

            for (auto &delete_edge : delete_edges) {
                edges.erase(delete_edge);
            }
        }

        void remove_edge(const edge_descriptor &edge_desc) requires
            (not util::is_random_access_v<edge_container> and
                not util::is_random_access_v<out_edge_container> and
                (not is_bidirectional_v or not util::is_random_access_v<in_edge_container>)) {
            vertex_type &source_vertex = get_vertex(source(edge_desc));

            out_edge_container &out_edges = source_vertex.out_edges;
            out_edge_descriptor out_edge_desc = out_edges.begin();
            while (out_edge_desc != out_edges.end()) {
                const edge_descriptor &source_out_edge_desc = *out_edge_desc;

                if (is_same_edge(source_out_edge_desc, edge_desc)) {
                    const vertex_descriptor &out_edge_target_desc = target(source_out_edge_desc);

                    if constexpr (is_bidirectional_v) {
                        in_remove_if(out_edge_target_desc, [&](const edge_descriptor &target_in_edge_desc) {
                            return is_same_edge(target_in_edge_desc, edge_desc);
                        });
                    }

                    if constexpr (is_undirected_v) {
                        if (not is_same_vertex(source(edge_desc), target(edge_desc))) {
                            out_remove_if(out_edge_target_desc, [&](const edge_descriptor &target_out_edge_desc) {
                                return is_same_vertex(out_edge_target_desc, target(target_out_edge_desc)) and
                                        is_same_edge(target_out_edge_desc, edge_desc);
                            });
                        }
                    }

                    edges.erase(source_out_edge_desc);

                    out_edges.erase(out_edge_desc);

                    break;
                } else {
                    ++out_edge_desc;
                }
            }
        }

        template<typename Predicate>
        void out_remove_if(const vertex_descriptor &vertex_desc, const Predicate &predicate) requires
            (not util::is_random_access_v<out_edge_container>) {
            out_edge_container &out_edges = get_vertex(vertex_desc).out_edges;
            out_edge_descriptor out_edge_desc = out_edges.begin();
            while (out_edge_desc != out_edges.end()) {
                if (predicate(*out_edge_desc)) {
                    out_edge_desc = out_edges.erase(out_edge_desc);
                } else {
                    ++out_edge_desc;
                }
            }
        }

        void out_remove_if(const vertex_descriptor &vertex_desc) requires
            (not util::is_random_access_v<out_edge_container>) {
            const auto &predicate = [](const edge_descriptor &out_edge_desc) constexpr -> bool { return false; };

            return out_remove_if(vertex_desc, predicate);
        }

        template<typename Predicate>
        void in_remove_if(const vertex_descriptor &vertex_desc, const Predicate &predicate) requires
            (is_bidirectional_v and not util::is_random_access_v<in_edge_container>) {
            in_edge_container &in_edges = get_vertex(vertex_desc).in_edges;
            in_edge_descriptor in_edge_desc = in_edges.begin();
            while (in_edge_desc != in_edges.end()) {
                if (predicate(*in_edge_desc)) {
                    in_edge_desc = in_edges.erase(in_edge_desc);
                } else {
                    ++in_edge_desc;
                }
            }
        }

        void in_remove_if(const vertex_descriptor &vertex_desc) requires
            (is_bidirectional_v and not util::is_random_access_v<in_edge_container>) {
            const auto &predicate = [](const edge_descriptor &out_edge_desc) constexpr -> bool { return false; };

            return in_remove_if(vertex_desc, predicate);
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

#endif //MAP_MATCHING_2_GRAPH_ADJACENCY_LIST_HPP
