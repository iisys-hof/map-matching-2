// Copyright (C) 2022 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GRAPH_HPP
#define MAP_MATCHING_2_GRAPH_HPP

#include <memory>
#include <vector>
#include <list>
#include <functional>

#include <boost/serialization/serialization.hpp>

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/allocators/private_adaptive_pool.hpp>
#include <boost/interprocess/containers/vector.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/copy.hpp>

#include <boost/property_map/transform_value_property_map.hpp>

namespace map_matching_2::geometry::network {

    template<typename Graph,
            typename Node = typename Graph::vertex_property_type,
            typename Edge = typename Graph::edge_property_type,
            typename VertexIndexMap = typename boost::property_map<Graph, std::size_t Node::*>::type,
            typename EdgeIndexMap = typename boost::property_map<Graph, std::size_t Edge::*>::type,
            typename VertexMap = typename boost::property_map<Graph, boost::vertex_bundle_t>::type,
            typename EdgeMap = typename boost::property_map<Graph, boost::edge_bundle_t>::type>
    class graph_memory {

    public:
        using graph_type = Graph;
        using node_type = Node;
        using edge_type = Edge;
        using vertex_index_map_type = VertexIndexMap;
        using edge_index_map_type = EdgeIndexMap;
        using vertex_map_type = VertexMap;
        using edge_map_type = EdgeMap;
        using graph_traits = typename boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        graph_memory()
                : graph_memory(graph_type{}) {}

        explicit graph_memory(graph_type graph)
                : _graph{std::move(graph)} {
            set_maps();
            remap();
        }

        explicit graph_memory(graph_type graph, bool no_maps_tag)
                : _graph{std::move(graph)} {}

        ~graph_memory() = default;

        graph_memory(const graph_memory &other) {
            _graph = other._graph;
            set_maps();
            remap();
        }

        graph_memory(graph_memory &&other) noexcept {
            _graph = std::move(other._graph);
            set_maps();
            remap();
        }

        graph_memory &operator=(const graph_memory &other) {
            _graph = other._graph;
            set_maps();
            remap();
            return *this;
        }

        graph_memory &operator=(graph_memory &&other) noexcept {
            _graph = std::move(other._graph);
            set_maps();
            remap();
            return *this;
        }

        template<typename GraphT, typename NodeT, typename EdgeT,
                typename VertexIndexMapT, typename EdgeIndexMapT, typename VertexMapT, typename EdgeMapT>
        graph_memory(const graph_memory<GraphT, NodeT, EdgeT,
                VertexIndexMapT, EdgeIndexMapT, VertexMapT, EdgeMapT> &other) {
            auto &other_cast = const_cast<graph_memory<GraphT, NodeT, EdgeT,
                    VertexIndexMapT, EdgeIndexMapT, VertexMapT, EdgeMapT> &>(other);

            vertex_converter<GraphT, graph_type> _vertex_converter{other_cast.graph(), this->graph()};
            boost::copy_graph(other_cast.graph(), this->graph(),
                              boost::vertex_index_map(other_cast.vertex_index_map())
                                      .vertex_copy(_vertex_converter));

            set_maps();
            remap();
        }

        template<typename GraphT, typename NodeT, typename EdgeT,
                typename VertexIndexMapT, typename EdgeIndexMapT, typename VertexMapT, typename EdgeMapT>
        graph_memory(graph_memory<GraphT, NodeT, EdgeT,
                VertexIndexMapT, EdgeIndexMapT, VertexMapT, EdgeMapT> &&other) noexcept = delete;

        template<typename GraphT, typename NodeT, typename EdgeT,
                typename VertexIndexMapT, typename EdgeIndexMapT, typename VertexMapT, typename EdgeMapT>
        graph_memory &operator=(const graph_memory<GraphT, NodeT, EdgeT,
                VertexIndexMapT, EdgeIndexMapT, VertexMapT, EdgeMapT> &other) {
            auto &other_cast = const_cast<graph_memory<GraphT, NodeT, EdgeT,
                    VertexIndexMapT, EdgeIndexMapT, VertexMapT, EdgeMapT> &>(other);

            vertex_converter<GraphT, graph_type> _vertex_converter{other_cast.graph(), this->graph()};
            boost::copy_graph(other_cast.graph(), this->graph(),
                              boost::vertex_index_map(other_cast.vertex_index_map())
                                      .vertex_copy(_vertex_converter));

            set_maps();
            remap();
            return *this;
        }

        template<typename GraphT, typename NodeT, typename EdgeT,
                typename VertexIndexMapT, typename EdgeIndexMapT, typename VertexMapT, typename EdgeMapT>
        graph_memory &operator=(graph_memory<GraphT, NodeT, EdgeT,
                VertexIndexMapT, EdgeIndexMapT, VertexMapT, EdgeMapT> &&other) noexcept = delete;

        [[nodiscard]] graph_type &graph() {
            return _graph;
        }

        [[nodiscard]] node_type &vertex(const vertex_descriptor &v) {
            return _graph[v];
        }

        [[nodiscard]] edge_type &edge(const edge_descriptor &e) {
            return _graph[e];
        }

        [[nodiscard]] vertex_index_map_type &vertex_index_map() {
            return _vertex_index_map;
        }

        [[nodiscard]] edge_index_map_type &edge_index_map() {
            return _edge_index_map;
        }

        [[nodiscard]] vertex_map_type &vertex_map() {
            return _vertex_map;
        }

        [[nodiscard]] edge_map_type &edge_map() {
            return _edge_map;
        }

        [[nodiscard]] std::size_t vertex_index(const vertex_descriptor &v) const {
            return boost::get(_vertex_index_map, v);
        }

        [[nodiscard]] std::size_t edge_index(const edge_descriptor &e) const {
            return boost::get(_edge_index_map, e);
        }

        void set_maps() {
            _vertex_index_map = boost::get(&node_type::index, _graph);
            _edge_index_map = boost::get(&edge_type::index, _graph);
        }

        vertex_descriptor add_vertex(const node_type &node) {
            std::size_t index = boost::num_vertices(_graph);
            vertex_descriptor v = boost::add_vertex(node, _graph);
            boost::put(_vertex_index_map, v, index);
            return v;
        }

        std::pair<edge_descriptor, bool>
        add_edge(const vertex_descriptor &u, const vertex_descriptor &v, const edge_type &edge) {
            std::size_t index = boost::num_edges(_graph);
            std::pair<edge_descriptor, bool> edge_pair = boost::add_edge(u, v, edge, _graph);
            if (edge_pair.second) {
                boost::put(_edge_index_map, edge_pair.first, index);
            }
            return edge_pair;
        }

        void clear_vertex(const vertex_descriptor &v) {
            boost::clear_vertex(v, _graph);
        }

        void remove_vertex(const vertex_descriptor &v) {
            boost::remove_vertex(v, _graph);
        }

        void remove_edge(const edge_descriptor &e) {
            boost::remove_edge(e, _graph);
        }

        void remove_edge(const vertex_descriptor &u, const vertex_descriptor &v) {
            boost::remove_edge(u, v, _graph);
        }

        void remap() {
            _vertex_map = boost::get(boost::vertex_bundle, _graph);
            _edge_map = boost::get(boost::edge_bundle, _graph);
        }

        void reindex() {
            std::size_t index = 0;
            for (const vertex_descriptor &v: boost::make_iterator_range(boost::vertices(_graph))) {
                boost::put(_vertex_index_map, v, index++);
            }

            index = 0;
            for (const edge_descriptor &e: boost::make_iterator_range(boost::edges(_graph))) {
                boost::put(_edge_index_map, e, index++);
            }
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void save(Archive &ar, const unsigned int version) const {
            ar << _graph;
        }

        template<typename Archive>
        void load(Archive &ar, const unsigned int version) {
            ar >> _graph;

            set_maps();
            remap();
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()

    protected:
        graph_type _graph;

        vertex_index_map_type _vertex_index_map;
        edge_index_map_type _edge_index_map;

        vertex_map_type _vertex_map;
        edge_map_type _edge_map;

        template<typename GraphA, typename GraphB = graph_type,
                typename = void>
        struct vertex_converter {
            using vertex_descriptor_in = typename GraphA::vertex_descriptor;
            using vertex_descriptor_out = typename GraphB::vertex_descriptor;

            vertex_converter(const GraphA &graph_in, GraphB &graph_out)
                    : vertex_all_map_in{boost::get(boost::vertex_all, graph_in)},
                      vertex_all_map_out{boost::get(boost::vertex_all, graph_out)} {}

            void operator()(const vertex_descriptor_in &vertex_original,
                            vertex_descriptor_out &vertex_converted) const {
                boost::put(vertex_all_map_out, vertex_converted, boost::get(vertex_all_map_in, vertex_original));
            }

            typename boost::property_map<GraphA, boost::vertex_all_t>::const_type vertex_all_map_in;
            mutable typename boost::property_map<GraphB, boost::vertex_all_t>::type vertex_all_map_out;
        };

        template<typename GraphA, typename GraphB>
        struct vertex_converter<GraphA, GraphB,
                std::enable_if_t<std::is_same_v<typename GraphB::vertex_property_type, boost::no_property>>> {
            using vertex_descriptor_in = typename GraphA::vertex_descriptor;
            using vertex_descriptor_out = typename GraphB::vertex_descriptor;

            vertex_converter(const GraphA &graph_in, GraphB &graph_out)
                    : vertex_all_map_in{boost::detail::remove_first_property(),
                                        boost::get(boost::vertex_all, graph_in)},
                      vertex_all_map_out{boost::get(boost::vertex_all, graph_out)} {}

            void operator()(const vertex_descriptor_in &vertex_original,
                            vertex_descriptor_out &vertex_converted) const {
                boost::put(vertex_all_map_out, vertex_converted, boost::get(vertex_all_map_in, vertex_original));
            }

            typename boost::transform_value_property_map<boost::detail::remove_first_property,
                    typename boost::property_map<GraphA, boost::vertex_all_t>::const_type> vertex_all_map_in;
            mutable typename boost::property_map<GraphB, boost::vertex_all_t>::type vertex_all_map_out;
        };

        template<typename GraphA, typename GraphB>
        struct vertex_converter<GraphA, GraphB,
                std::enable_if_t<std::is_same_v<typename GraphA::vertex_property_type, boost::no_property>>> {
            using vertex_descriptor_in = typename GraphA::vertex_descriptor;
            using vertex_descriptor_out = typename GraphB::vertex_descriptor;

            vertex_converter(const GraphA &graph_in, GraphB &graph_out)
                    : vertex_all_map_in{boost::get(boost::vertex_all, graph_in)},
                      vertex_all_map_out{boost::detail::remove_first_property(),
                                         boost::get(boost::vertex_all, graph_out)} {}

            void operator()(const vertex_descriptor_in &vertex_original,
                            vertex_descriptor_out &vertex_converted) const {
                boost::put(vertex_all_map_out, vertex_converted, boost::get(vertex_all_map_in, vertex_original));
            }

            typename boost::property_map<GraphA, boost::vertex_all_t>::const_type vertex_all_map_in;
            mutable typename boost::transform_value_property_map<boost::detail::remove_first_property,
                    typename boost::property_map<GraphB, boost::vertex_all_t>::type> vertex_all_map_out;
        };

    };

    template<typename Graph, typename Node, typename Edge,
            typename VertexIndexMap = typename boost::property_map<Graph, boost::vertex_index_t>::type,
            typename EdgeIndexMap = typename boost::property_map<Graph, boost::edge_index_t>::type,
            typename NodeAllocator = std::allocator<Node>,
            typename EdgeAllocator = std::allocator<Edge>,
            typename NodeStorage = std::vector<Node, NodeAllocator>,
            typename EdgeStorage = std::vector<Edge, EdgeAllocator>,
            typename VertexMap = boost::iterator_property_map<typename NodeStorage::iterator, VertexIndexMap>,
            typename EdgeMap = boost::iterator_property_map<typename EdgeStorage::iterator, EdgeIndexMap>>
    class graph_memory_external
            : public graph_memory<Graph, Node, Edge, VertexIndexMap, EdgeIndexMap, VertexMap, EdgeMap> {

    public:
        using graph_type = Graph;
        using node_type = Node;
        using edge_type = Edge;
        using vertex_index_map_type = VertexIndexMap;
        using edge_index_map_type = EdgeIndexMap;
        using node_allocator_type = NodeAllocator;
        using edge_allocator_type = EdgeAllocator;
        using node_storage_type = NodeStorage;
        using edge_storage_type = EdgeStorage;
        using vertex_map_type = VertexMap;
        using edge_map_type = EdgeMap;
        using base_type = graph_memory<graph_type, node_type, edge_type,
                vertex_index_map_type, edge_index_map_type, vertex_map_type, edge_map_type>;
        using graph_traits = typename boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        graph_memory_external(node_storage_type *node_storage, edge_storage_type *edge_storage)
                : graph_memory_external{Graph{}, node_storage, edge_storage} {}

        graph_memory_external(graph_type graph, node_storage_type *node_storage, edge_storage_type *edge_storage)
                : base_type{std::move(graph), false}, _node_storage{node_storage}, _edge_storage{edge_storage} {
            set_maps();
        }

        ~graph_memory_external() = default;

        graph_memory_external(const graph_memory_external &other) = delete;

        graph_memory_external(graph_memory_external &&other) noexcept = delete;

        graph_memory_external &operator=(const graph_memory_external &other) {
            base_type::operator=(other);

            std::copy(other._node_storage->cbegin(), other._node_storage->cend(),
                      std::back_inserter(*this->_node_storage));
            std::copy(other._edge_storage->cbegin(), other._edge_storage->cend(),
                      std::back_inserter(*this->_edge_storage));

            set_maps();
            remap();

            return *this;
        }

        graph_memory_external &operator=(graph_memory_external &&other) noexcept {
            base_type::operator=(std::move(other));

            std::move(other._node_storage->begin(), other._node_storage->end(),
                      std::back_inserter(*this->_node_storage));
            std::move(other._edge_storage->begin(), other._edge_storage->end(),
                      std::back_inserter(*this->_edge_storage));

            set_maps();
            remap();

            return *this;
        }

        template<typename GraphT, typename NodeT, typename EdgeT, typename VertexIndexMapT, typename EdgeIndexMapT,
                typename NodeAllocatorT, typename EdgeAllocatorT, typename NodeStorageT, typename EdgeStorageT,
                typename VertexMapT, typename EdgeMapT>
        graph_memory_external(const graph_memory_external<GraphT, NodeT, EdgeT, VertexIndexMapT, EdgeIndexMapT,
                NodeAllocatorT, EdgeAllocatorT, NodeStorageT, EdgeStorageT, VertexMapT, EdgeMapT> &other) = delete;

        template<typename GraphT, typename NodeT, typename EdgeT, typename VertexIndexMapT, typename EdgeIndexMapT,
                typename NodeAllocatorT, typename EdgeAllocatorT, typename NodeStorageT, typename EdgeStorageT,
                typename VertexMapT, typename EdgeMapT>
        graph_memory_external(graph_memory_external<GraphT, NodeT, EdgeT, VertexIndexMapT, EdgeIndexMapT,
                NodeAllocatorT, EdgeAllocatorT, NodeStorageT, EdgeStorageT, VertexMapT, EdgeMapT> &&other) noexcept = delete;

        template<typename GraphT, typename NodeT, typename EdgeT, typename VertexIndexMapT, typename EdgeIndexMapT,
                typename NodeAllocatorT, typename EdgeAllocatorT, typename NodeStorageT, typename EdgeStorageT,
                typename VertexMapT, typename EdgeMapT>
        graph_memory_external &
        operator=(const graph_memory_external<GraphT, NodeT, EdgeT, VertexIndexMapT, EdgeIndexMapT,
                NodeAllocatorT, EdgeAllocatorT, NodeStorageT, EdgeStorageT, VertexMapT, EdgeMapT> &other) {
            auto &other_cast = const_cast<graph_memory_external<GraphT, NodeT, EdgeT, VertexIndexMapT, EdgeIndexMapT,
                    NodeAllocatorT, EdgeAllocatorT, NodeStorageT, EdgeStorageT, VertexMapT, EdgeMapT> &>(other);

            typename base_type::template vertex_converter<GraphT, graph_type>
                    _vertex_converter{other_cast.graph(), this->graph()};
            boost::copy_graph(other_cast.graph(), this->graph(),
                              boost::vertex_index_map(other_cast.vertex_index_map())
                                      .vertex_copy(_vertex_converter));

            std::copy(other_cast.node_storage()->cbegin(), other_cast.node_storage()->cend(),
                      std::back_inserter(*this->_node_storage));
            std::copy(other_cast.edge_storage()->cbegin(), other_cast.edge_storage()->cend(),
                      std::back_inserter(*this->_edge_storage));

            set_maps();
            remap();

            return *this;
        }

        template<typename GraphT, typename NodeT, typename EdgeT, typename VertexIndexMapT, typename EdgeIndexMapT,
                typename NodeAllocatorT, typename EdgeAllocatorT, typename NodeStorageT, typename EdgeStorageT,
                typename VertexMapT, typename EdgeMapT>
        graph_memory_external &operator=(graph_memory_external<GraphT, NodeT, EdgeT, VertexIndexMapT, EdgeIndexMapT,
                NodeAllocatorT, EdgeAllocatorT, NodeStorageT, EdgeStorageT, VertexMapT, EdgeMapT> &&other) noexcept {
            auto &other_cast = const_cast<graph_memory_external<GraphT, NodeT, EdgeT, VertexIndexMapT, EdgeIndexMapT,
                    NodeAllocatorT, EdgeAllocatorT, NodeStorageT, EdgeStorageT, VertexMapT, EdgeMapT> &>(other);

            typename base_type::template vertex_converter<GraphT, graph_type>
                    _vertex_converter{other_cast.graph(), this->graph()};
            boost::copy_graph(other_cast.graph(), this->graph(),
                              boost::vertex_index_map(other_cast.vertex_index_map())
                                      .vertex_copy(_vertex_converter));

            std::move(other_cast.node_storage()->begin(), other_cast.node_storage()->end(),
                      std::back_inserter(*this->_node_storage));
            std::move(other_cast.edge_storage()->begin(), other_cast.edge_storage()->end(),
                      std::back_inserter(*this->_edge_storage));

            set_maps();
            remap();

            return *this;
        }

        [[nodiscard]] node_type &vertex(const vertex_descriptor &v) {
            return boost::get(this->_vertex_map, v);
        }

        [[nodiscard]] edge_type &edge(const edge_descriptor &e) {
            return boost::get(this->_edge_map, e);
        }

        [[nodiscard]] node_storage_type *node_storage() {
            return _node_storage;
        }

        [[nodiscard]] edge_storage_type *edge_storage() {
            return _edge_storage;
        }

        void set_maps() {
            this->_vertex_index_map = boost::get(boost::vertex_index, this->_graph);
            this->_edge_index_map = boost::get(boost::edge_index, this->_graph);
        }

        vertex_descriptor add_vertex(const node_type &node, bool remap = true) {
            return add_vertex([&node](node_storage_type *&node_storage) -> std::size_t {
                std::size_t index = node_storage->size();
                node_storage->emplace_back(node);
                return index;
            }, remap);
        }

        vertex_descriptor add_vertex(const std::function<std::size_t(node_storage_type *&)> &func, bool remap = true) {
            vertex_descriptor v;
            std::size_t index = func(_node_storage);
            if constexpr (std::is_same_v<typename graph_type::out_edge_list_selector, boost::vecS> and
                          std::is_same_v<typename graph_type::vertex_property_type, boost::no_property>) {
                v = boost::add_vertex(this->_graph);
            } else {
                v = boost::add_vertex(index, this->_graph);
            }
            if (remap) {
                this->remap();
            }
            return v;
        }

        std::pair<edge_descriptor, bool>
        add_edge(const vertex_descriptor &u, const vertex_descriptor &v, const edge_type &edge, bool remap = true) {
            return add_edge(u, v, [&edge](edge_storage_type *&edge_storage) -> std::size_t {
                std::size_t index = edge_storage->size();
                edge_storage->emplace_back(edge);
                return index;
            }, remap);
        }

        std::pair<edge_descriptor, bool>
        add_edge(const vertex_descriptor &u, const vertex_descriptor &v,
                 const std::function<std::size_t(edge_storage_type * &)> &func, bool remap = true) {
            std::pair<edge_descriptor, bool> e;
            std::size_t index = func(_edge_storage);
            e = boost::add_edge(u, v, index, this->_graph);
            if (remap) {
                this->remap();
            }
            return e;
        }

        void clear_vertex(const vertex_descriptor &v) {
            typename graph_traits::out_edge_iterator out_edge_it, out_edge_next, out_edge_end;
            std::tie(out_edge_it, out_edge_end) = boost::out_edges(v, this->_graph);
            for (out_edge_next = out_edge_it; out_edge_it != out_edge_end; out_edge_it = out_edge_next) {
                out_edge_next++;
                const edge_descriptor &e = *out_edge_it;
                remove_edge(e);
            }

            typename graph_traits::in_edge_iterator in_edge_it, in_edge_next, in_edge_end;
            std::tie(in_edge_it, in_edge_end) = boost::in_edges(v, this->_graph);
            for (in_edge_next = in_edge_it; in_edge_it != in_edge_end; in_edge_it = in_edge_next) {
                in_edge_next++;
                const edge_descriptor &e = *in_edge_it;
                remove_edge(e);
            }
        }

        void remove_vertex(const vertex_descriptor &v) {
            std::size_t index = this->vertex_index(v);
            _node_storage->operator[](index) = node_type{};
            boost::remove_vertex(v, this->_graph);
        }

        void remove_edge(const edge_descriptor &e) {
            std::size_t index = this->edge_index(e);
            _edge_storage->operator[](index) = edge_type{};
            boost::remove_edge(e, this->_graph);
        }

        void remove_edge(const vertex_descriptor &u, const vertex_descriptor &v) {
            std::pair<edge_descriptor, bool> edge = boost::edge(u, v, this->_graph);
            if (edge.second) {
                std::size_t index = this->edge_index(edge.first);
                _edge_storage->operator[](index) = edge_type{};
                boost::remove_edge(u, v, this->_graph);
            }
        }

        void remap() {
            this->_vertex_map = boost::make_iterator_property_map(_node_storage->begin(), this->vertex_index_map());
            this->_edge_map = boost::make_iterator_property_map(_edge_storage->begin(), this->edge_index_map());
        }

        void reindex() {
            std::erase(*_node_storage, node_type{});
            std::erase(*_edge_storage, edge_type{});

            graph_memory<Graph>::reindex();
            remap();
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void save(Archive &ar, const unsigned int version) const {
            ar << boost::serialization::base_object<graph_memory<Graph>>(*this);
        }

        template<typename Archive>
        void load(Archive &ar, const unsigned int version) {
            ar >> boost::serialization::base_object<graph_memory<Graph>>(*this);

            set_maps();
            remap();
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()

    protected:
        node_storage_type *_node_storage;
        edge_storage_type *_edge_storage;

    };

}

#endif //MAP_MATCHING_2_GRAPH_HPP
