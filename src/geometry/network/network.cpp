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

#include "network.hpp"

#ifndef BOOST_THREAD_VERSION
#define BOOST_THREAD_VERSION 5
#endif

#include <boost/thread.hpp>

#include <geometry/util.hpp>
#include <io/csv_exporter.hpp>

#include "types.hpp"

namespace map_matching_2::geometry::network {

    template<typename Graph>
    typename network<Graph>::vertex_descriptor network<Graph>::add_vertex(node_type &node) {
        node.index = boost::num_vertices(graph);
        return boost::add_vertex(node, graph);
    }

    template<typename Graph>
    typename network<Graph>::edge_descriptor
    network<Graph>::add_edge(const vertex_descriptor u, const vertex_descriptor v, edge_type &edge) {
        edge.index = boost::num_edges(graph);
        return boost::add_edge(u, v, edge, graph).first;
    }

    template<typename Graph>
    void network<Graph>::simplify(const bool simplify_network_complete) {
        BOOST_CONCEPT_ASSERT((boost::VertexAndEdgeListGraphConcept<Graph>));
        BOOST_CONCEPT_ASSERT((boost::BidirectionalGraphConcept<Graph>));

        typename graph_traits::vertex_iterator vertex_it, vertex_next, vertex_end;
        std::tie(vertex_it, vertex_end) = boost::vertices(graph);
        for (vertex_next = vertex_it; vertex_it != vertex_end; vertex_it = vertex_next) {
            vertex_next++;
            const auto vertex = *vertex_it;

            const auto degree = boost::degree(vertex, graph);
            const auto in_degree = boost::in_degree(vertex, graph);
            const auto out_degree = boost::out_degree(vertex, graph);

            if ((degree == 2 or degree == 4) and in_degree == out_degree) {
                // we can only merge when the in and out degree is equal
                const auto adj_vertices = _adjacent_vertices(vertex);
                if (_adjacent_mergable(adj_vertices, vertex)) {
                    // only continue if we are within a road or at a dead end
                    std::vector<std::pair<edge_descriptor, edge_descriptor>> merge_edges;
                    merge_edges.reserve(out_degree);

                    for (const auto in_edge: boost::make_iterator_range(boost::in_edges(vertex, graph))) {
                        for (const auto out_edge: boost::make_iterator_range(boost::out_edges(vertex, graph))) {
                            if (degree == 2) {
                                merge_edges.emplace_back(std::pair{in_edge, out_edge});
                            } else {
                                if (boost::source(in_edge, graph) != boost::target(out_edge, graph)) {
                                    merge_edges.emplace_back(std::pair{in_edge, out_edge});
                                }
                            }
                        }
                    }

                    for (const auto &merge_edge: merge_edges) {
                        _merge(merge_edge.first, merge_edge.second, simplify_network_complete);
                    }
                }
            }
        }

        reindex();
    }

    template<typename Graph>
    void network<Graph>::remove_unconnected_components() {
        auto index_map = boost::get(&node_type::index, graph);

        auto colors = make_colors();
        auto color_map = boost::make_iterator_property_map(colors.begin(), index_map);

        std::unordered_set<vertex_descriptor> visited;
        std::vector<std::unordered_set<vertex_descriptor>> components;

        for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
            if (not visited.contains(vertex)) {
                auto &discovered = components.emplace_back(std::unordered_set<vertex_descriptor>{});
                vertex_visitor vertex_visitor{visited, discovered};

                boost::breadth_first_visit(graph, vertex,
                                           boost::vertex_index_map(index_map)
                                                   .color_map(color_map)
                                                   .visitor(vertex_visitor));
            }
        }

        const auto largest_component_it = std::max_element(components.cbegin(), components.cend(),
                                                           [](const auto &a, const auto &b) {
                                                               return a.size() < b.size();
                                                           });
        const auto &largest_component = *largest_component_it;

        // found largest weakly component, remove all remaining nodes and edges
        for (const auto &component_nodes: components) {
            if (&component_nodes != &largest_component) {
                // only remove nodes not from the largest component
                for (const auto vertex_descriptor: component_nodes) {
                    boost::clear_vertex(vertex_descriptor, graph);
                    boost::remove_vertex(vertex_descriptor, graph);
                }
            }
        }

        reindex();
    }

    template<typename Graph>
    void network<Graph>::reindex() {
        std::size_t counter = 0;
        for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
            graph[vertex].index = counter++;
        }

        counter = 0;
        for (const auto &edge: boost::make_iterator_range(boost::edges(graph))) {
            graph[edge].index = counter++;
        }
    }

    template<typename Graph>
    void network<Graph>::rebuild_spatial_indices() {
        const auto build_points_index = [&]() {
            std::vector<std::pair<point_type, vertex_descriptor>> points;
            points.reserve(boost::num_vertices(graph));
            for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
                points.emplace_back(std::pair{graph[vertex].point, vertex});
            }

            points_index = boost::geometry::index::rtree<std::pair<point_type, vertex_descriptor>,
                    boost::geometry::index::rstar<16 >>
                    {points};
        };

        const auto build_segments_index = [&]() {
            std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor>>> segments;
            segments.reserve(boost::num_edges(graph));
            for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(graph))) {
                const auto &edge = graph[edge_descriptor];
                for (std::size_t i = 0; i < edge.rich_segments.size(); ++i) {
                    const auto &rich_segment = edge.rich_segments[i];
                    segments.emplace_back(std::pair{rich_segment.segment, std::pair{i, edge_descriptor}});
                }
            }

            segments_index =
                    boost::geometry::index::rtree<std::pair<segment_type, std::pair<std::size_t, edge_descriptor>>,
                            boost::geometry::index::rstar<16 >>
                            {segments};
        };

        std::uint32_t threads = boost::thread::hardware_concurrency();
        if (threads >= 2) {
            boost::thread points_index_thread{build_points_index};
            boost::thread segments_index_thread{build_segments_index};

            points_index_thread.join();
            segments_index_thread.join();
        } else {
            build_points_index();
            build_segments_index();
        }
    }

    template<typename Graph>
    std::vector<typename network<Graph>::vertex_descriptor>
    network<Graph>::find_node(const osmium::object_id_type node_id) const {
        std::vector<vertex_descriptor> nodes;
        for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
            const auto &node = graph[vertex];
            if (node.id == node_id) {
                nodes.emplace_back(vertex);
            }
        }
        return nodes;
    }

    template<typename Graph>
    std::vector<typename network<Graph>::edge_descriptor>
    network<Graph>::find_edge(const osmium::object_id_type edge_id) const {
        std::vector<edge_descriptor> edges;
        for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(graph))) {
            const auto &edge = graph[edge_descriptor];
            if (edge.id == edge_id) {
                edges.emplace_back(edge_descriptor);
            }
        }
        return edges;
    }

    template<typename Graph>
    std::pair<bool, typename network<Graph>::vertex_descriptor>
    network<Graph>::point_in_nodes(const point_type &point) const {
        const auto nearest_points = query_points(boost::geometry::index::nearest(point, 1));
        if (not nearest_points.empty()) {
            const auto &nearest_point = nearest_points.front().first;
            bool found = geometry::equals_points(nearest_point, point);
            const auto vertex = nearest_points.front().second;
            return std::pair{found, vertex};
        }
        return std::pair{false, typename network<Graph>::vertex_descriptor{}};
    }

    template<typename Graph>
    std::pair<bool, std::pair<std::size_t, typename network<Graph>::edge_descriptor>>
    network<Graph>::point_in_edges(const point_type &point) const {
        const auto nearest_segments = query_segments(boost::geometry::index::nearest(point, 1));
        if (not nearest_segments.empty()) {
            const auto &nearest_segment = nearest_segments.front().first;
            bool found = geometry::equals_points(nearest_segment.first, point) or
                         geometry::equals_points(nearest_segment.second, point);
            const auto edge = nearest_segments.front().second;
            return std::pair{found, edge};
        }
        return std::pair{false, std::pair<std::size_t, typename network<Graph>::edge_descriptor>{}};
    }

    template<typename Graph>
    std::pair<bool, typename network<Graph>::edge_descriptor>
    network<Graph>::fitting_edge(const segment_type &segment) const {
        using angle_type = typename edge_type::angle_type;
        angle_type best_angle = geometry::default_float_type<angle_type>::v180;
        const auto azimuth = geometry::azimuth_segment_deg(segment);
        const edge_descriptor *edge_descriptor;
        const segment_type *best_segment = nullptr;

        const auto segments = query_segments(boost::geometry::index::intersects(segment));
        for (const auto &segment_pair: segments) {
            const segment_type &near_segment = segment_pair.first;
            const auto near_azimuth = geometry::azimuth_segment_deg(near_segment);
            const auto angle_diff = std::fabs(geometry::angle_diff(azimuth, near_azimuth));
            if (best_segment == nullptr or
                (geometry::equals_points(segment.first, near_segment.first) and
                 geometry::equals_points(segment.second, near_segment.second)) or angle_diff < best_angle) {
                best_angle = angle_diff;
                best_segment = &near_segment;
                edge_descriptor = &segment_pair.second.second;
            }
        }

        if (best_segment != nullptr) {
            return std::pair{true, *edge_descriptor};
        }
        return std::pair{false, typename network<Graph>::edge_descriptor{}};
    }

    template<typename Graph>
    std::vector<typename network<Graph>::vertex_descriptor> network<Graph>::make_precedessors() const {
        std::vector<vertex_descriptor> precedessors;
        precedessors.reserve(boost::num_vertices(graph));
        for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
            precedessors.emplace_back(vertex);
        }
        return precedessors;
    }

    template<typename Graph>
    std::vector<typename network<Graph>::length_type> network<Graph>::make_distances() const {
        std::vector<length_type> distances;
        distances.reserve(boost::num_vertices(graph));
        for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
            distances.emplace_back(std::numeric_limits<length_type>::infinity());
        }
        return distances;
    }

    template<typename Graph>
    std::vector<typename boost::default_color_type> network<Graph>::make_colors() const {
        std::vector<typename boost::default_color_type> colors;
        colors.reserve(boost::num_vertices(graph));
        for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
            colors.emplace_back(boost::white_color);
        }
        return colors;
    }

    template<typename Graph>
    std::vector<std::size_t> network<Graph>::make_heap_index() const {
        std::vector<std::size_t> heap_index;
        heap_index.reserve(boost::num_vertices(graph));
        for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
            heap_index.emplace_back(0);
        }
        return heap_index;
    }

    template<typename Graph>
    std::vector<typename network<Graph>::vertex_descriptor> network<Graph>::make_queue() const {
        std::vector<vertex_descriptor> queue;
        queue.reserve(boost::num_vertices(graph));
        return queue;
    }

    template<typename Graph>
    std::vector<std::list<typename network<Graph>::vertex_descriptor>>
    network<Graph>::dijkstra_shortest_paths(const vertex_descriptor &start,
                                            const std::unordered_set<vertex_descriptor> &goals,
                                            const double max_distance_factor, const distance_type max_distance,
                                            std::vector<vertex_descriptor> &vertices,
                                            std::vector<vertex_descriptor> &predecessors,
                                            std::vector<length_type> &distances,
                                            std::vector<typename boost::default_color_type> &colors,
                                            std::vector<std::size_t> &heap_index,
                                            std::vector<vertex_descriptor> &queue) const {
        auto index_map = boost::get(&node_type::index, graph);
        auto weight_map = boost::get(&edge_type::length, graph);

        auto predecessor_map = boost::make_iterator_property_map(predecessors.begin(), index_map);
        auto distance_map = boost::make_iterator_property_map(distances.begin(), index_map);
        auto color_map = boost::make_iterator_property_map(colors.begin(), index_map);
        auto heap_index_map = boost::make_iterator_property_map(&(*heap_index.begin()), index_map);

        std::unordered_set<vertex_descriptor> goals_found;
        goals_visitor<decltype(index_map), decltype(weight_map), decltype(predecessor_map),
                decltype(distance_map), decltype(color_map)>
                goals_visitor{max_distance_factor, max_distance, goals, goals_found, vertices,
                              index_map, weight_map, predecessor_map, distance_map, color_map};

        auto compare = std::less<length_type>();
        auto combine = std::plus<length_type>();
        auto zero = geometry::default_float_type<length_type>::v0;

        boost::put(distance_map, start, geometry::default_float_type<length_type>::v0);

        try {
            // adapted from boost::dijkstra_shortest_paths_no_init
            boost::d_ary_heap_indirect<vertex_descriptor, 4,
                    decltype(heap_index_map), decltype(distance_map), decltype(compare)>
                    Q(distance_map, heap_index_map, compare, queue);

            boost::detail::dijkstra_bfs_visitor<decltype(goals_visitor), decltype(Q), decltype(weight_map),
                    decltype(predecessor_map), decltype(distance_map), decltype(combine), decltype(compare)>
                    bfs_vis(goals_visitor, Q, weight_map, predecessor_map, distance_map, combine, compare, zero);

            boost::breadth_first_visit(graph, &start, &start + 1, Q, bfs_vis, color_map);
        } catch (goals_found_exception &) {}

        std::vector<std::list<vertex_descriptor>> shortest_paths;
        shortest_paths.reserve(goals_found.size());
        for (const auto &goal: goals_found) {
            std::list<vertex_descriptor> shortest_path;
            for (auto vertex = goal;; vertex = predecessor_map[vertex]) {
                shortest_path.emplace_front(vertex);
                if (predecessor_map[vertex] == vertex)
                    break;
            }
            shortest_paths.emplace_back(std::move(shortest_path));
        }

        for (const auto &vertex: vertices) {
            boost::put(predecessor_map, vertex, vertex);
            boost::put(distance_map, vertex, std::numeric_limits<length_type>::infinity());
            boost::put(color_map, vertex, boost::white_color);
            boost::put(heap_index_map, vertex, 0);
        }
        vertices.clear();

        return shortest_paths;
    }

    template<typename Graph>
    typename network<Graph>::route_type
    network<Graph>::extract_route(const std::list<vertex_descriptor> &shortest_path) const {
        if (shortest_path.empty()) {
            // return invalid route
            return route_type{true};
        }

        std::vector<std::reference_wrapper<const rich_line_type>> edges;
        edges.reserve(std::distance(shortest_path.cbegin(), shortest_path.cend()) - 1);

        for (auto vertex_it = shortest_path.cbegin(); vertex_it != shortest_path.cend(); ++vertex_it) {
            const auto next_vertex_it = std::next(vertex_it);
            if (next_vertex_it != shortest_path.cend()) {
                const rich_line_type *edge_ptr = nullptr;
                length_type shortest_length = std::numeric_limits<length_type>::infinity();
                for (const auto out_edge: boost::make_iterator_range(boost::out_edges(*vertex_it, graph))) {
                    if (boost::target(out_edge, graph) == *next_vertex_it) {
                        const auto &edge = graph[out_edge];
                        if (not edge.has_length or edge.length < shortest_length) {
                            if (edge.has_length) {
                                shortest_length = edge.length;
                            }
                            edge_ptr = &edge;
                        }
                    }
                }

                if (edge_ptr != nullptr) {
                    edges.emplace_back(std::cref(*edge_ptr));
                }
            }
        }

        if (edges.empty()) {
            // only one node, so no paths found, empty route for staying here
            return route_type{false};
        }

        if (std::all_of(edges.cbegin(), edges.cend(), [](const rich_line_type &edge) {
            return edge.line.empty();
        })) {
            // still no line, empty route for staying here
            return route_type{false};
        }

        return route_type{edges};
    }

    template<typename Graph>
    typename network<Graph>::rtree_points_type::bounds_type network<Graph>::bounds() const {
        return points_index.bounds();
    }

    template<typename Graph>
    template<typename NetworkConverted>
    void network<Graph>::convert(NetworkConverted &network_converted) const {
        boost::copy_graph(graph, network_converted.graph,
                          boost::vertex_index_map(get(&node_type::index, graph)));
        network_converted.tag_helper = tag_helper;
    }

    template<typename Graph>
    template<typename NetworkReprojected>
    void network<Graph>::reproject(
            NetworkReprojected &network_reprojected,
            const boost::geometry::srs::proj4 &from, const boost::geometry::srs::proj4 &to) const {
        boost::geometry::srs::transformation<> transformation{from, to};
        vertex_reprojector<typename NetworkReprojected::graph_type>
                _vertex_reprojector{graph, network_reprojected.graph, transformation};
        edge_reprojector<typename NetworkReprojected::graph_type>
                _edge_reprojector{graph, network_reprojected.graph, transformation};

        boost::copy_graph(graph, network_reprojected.graph,
                          boost::vertex_index_map(get(&node_type::index, graph)).
                                  vertex_copy(_vertex_reprojector).
                                  edge_copy(_edge_reprojector));
        network_reprojected.tag_helper = tag_helper;
    }

    template<typename Graph>
    void network<Graph>::save_nodes(std::string filename) const {
        BOOST_CONCEPT_ASSERT((boost::VertexListGraphConcept<Graph>));

        io::csv_exporter<',', '"'> csv{std::move(filename)};
        bool first = true;
        for (const auto vertex: boost::make_iterator_range(boost::vertices(graph))) {
            const auto &node = graph[vertex];
            if (first) {
                csv << node.header();
                first = false;
            }
            csv << node.row(tag_helper);
        }
    }

    template<typename Graph>
    void network<Graph>::save_edges(std::string filename) const {
        BOOST_CONCEPT_ASSERT((boost::EdgeListGraphConcept<Graph>));

        io::csv_exporter<',', '"'> csv{std::move(filename)};
        bool first = true;
        for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(graph))) {
            const auto &edge = graph[edge_descriptor];
            if (first) {
                csv << edge.header();
                first = false;
            }
            csv << edge.row(tag_helper);
        }
    }

    template<typename Graph>
    void network<Graph>::save(std::string nodes, std::string edges) const {
        save_nodes(std::move(nodes));
        save_edges(std::move(edges));
    }

    template<typename Graph>
    void network<Graph>::_merge(const edge_descriptor in_edge, const edge_descriptor out_edge,
                                const bool simplify_network_complete) {
        BOOST_CONCEPT_ASSERT((boost::VertexMutablePropertyGraphConcept<Graph>));
        BOOST_CONCEPT_ASSERT((boost::EdgeMutablePropertyGraphConcept<Graph>));
        BOOST_CONCEPT_ASSERT((boost::MutableBidirectionalGraphConcept<Graph>));

        if (in_edge != out_edge) {
            // only merge if edges are not the same
            const auto source_vertex_in = boost::source(in_edge, graph);
            const auto target_vertex_in = boost::target(in_edge, graph);
            const auto source_vertex_out = boost::source(out_edge, graph);
            const auto target_vertex_out = boost::target(out_edge, graph);

            if (target_vertex_in == source_vertex_out and source_vertex_in != target_vertex_out) {
                // check that edges can be merged in same vertex
                // a oneway loop could end in itself, only continue if we have not this situation
                auto &source_edge = graph[in_edge];
                auto &target_edge = graph[out_edge];

                if (simplify_network_complete or
                    (source_edge.id == target_edge.id and source_edge.tags == target_edge.tags)) {
                    // only merge edges when they fit together concerning their id and tag_helper_ref
                    // this means we split when the tag_helper_ref change, for example maxspeed change within road
                    if (not source_edge.line.empty() and not target_edge.line.empty()) {
                        // only continue if there is something to merge
                        edge_type new_edge = edge_type::merge(source_edge, target_edge);

                        boost::remove_edge(in_edge, graph);
                        boost::remove_edge(out_edge, graph);
                        add_edge(source_vertex_in, target_vertex_out, new_edge);

                        if (boost::degree(target_vertex_in, graph) == 0) {
                            // only remove vertex if all edges were previously removed
                            boost::remove_vertex(target_vertex_in, graph);
                        }
                    }
                }
            }
        }
    }

    template<typename Graph>
    std::unordered_map<osmium::object_id_type, std::size_t>
    network<Graph>::_adjacent_vertices(const vertex_descriptor &vertex) const {
        BOOST_CONCEPT_ASSERT((boost::VertexAndEdgeListGraphConcept<Graph>));
        BOOST_CONCEPT_ASSERT((boost::BidirectionalGraphConcept<Graph>));

        std::unordered_map<osmium::object_id_type, std::size_t> adj_vertices_map;
        const auto emplace = [&adj_vertices_map](const osmium::object_id_type id) {
            auto search = adj_vertices_map.find(id);
            if (search == adj_vertices_map.end()) {
                adj_vertices_map.emplace(id, 1);
            } else {
                search->second++;
            }
        };

        for (const auto &in_edge: boost::make_iterator_range(boost::in_edges(vertex, graph))) {
            const auto &source_vertex = boost::source(in_edge, graph);
            emplace(graph[source_vertex].id);
        }

        for (const auto &out_edge: boost::make_iterator_range(boost::out_edges(vertex, graph))) {
            const auto &target_vertex = boost::target(out_edge, graph);
            emplace(graph[target_vertex].id);
        }

        return adj_vertices_map;
    }

    template<typename Graph>
    bool network<Graph>::_adjacent_mergable(
            const std::unordered_map<osmium::object_id_type, std::size_t> &adjacent_vertices,
            const vertex_descriptor &vertex) const {
        bool adj_vertices_okay = adjacent_vertices.size() <= 2;

        if (adj_vertices_okay and adjacent_vertices.size() > 1) {
            for (const auto &adj: adjacent_vertices) {
                if (adj.second > 2) {
                    // special situation when a vertex has two out and two in but three of these edges point to the same vertex
                    adj_vertices_okay = false;
                    break;
                }
            }
        }

        if (adj_vertices_okay and adjacent_vertices.size() > 1) {
            // special situation with degree 4 where both sources have the same source vertex
            std::unordered_set<vertex_descriptor> source_vertices;
            for (const auto &in_edge: boost::make_iterator_range(boost::in_edges(vertex, graph))) {
                const auto &source_vertex = boost::source(in_edge, graph);
                if (source_vertices.contains(source_vertex)) {
                    adj_vertices_okay = false;
                    break;
                } else {
                    source_vertices.emplace(source_vertex);
                }
            }
        }

        if (adj_vertices_okay and adjacent_vertices.size() > 1) {
            // special situation with degree 4 where both targets have the same target vertex
            std::unordered_set<vertex_descriptor> target_vertices;
            for (const auto &out_edge: boost::make_iterator_range(boost::out_edges(vertex, graph))) {
                const auto &target_vertex = boost::target(out_edge, graph);
                if (target_vertices.contains(target_vertex)) {
                    adj_vertices_okay = false;
                    break;
                } else {
                    target_vertices.emplace(target_vertex);
                }
            }
        }

        return adj_vertices_okay;
    }

    template<typename Graph>
    std::vector<typename network<Graph>::point_type> network<Graph>::_process_segments_points_result(
            std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor
            >>> &&pre_results) const {
        std::vector<point_type> results;
        results.
                reserve(pre_results
                                .

                                        size()

                        * 2);
        for (
                std::size_t i = 0;
                i < pre_results.

                        size();

                ++i) {
            auto &pre_result = pre_results[i];
            results.
                    emplace_back(std::move(pre_result.first.first)
            );
            results.
                    emplace_back(std::move(pre_result.first.second)
            );
        }
        return
                results;
    }

    template<typename Graph>
    std::set<typename network<Graph>::edge_descriptor> network<Graph>::_process_segments_unique_result(
            std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor
            >>> &&pre_results) const {
        std::set<edge_descriptor> results;
        for (
            const auto &pre_result
                : pre_results) {
            results.
                    emplace(pre_result
                                    .second.second);
        }
        return
                results;
    }

    template<typename Graph>
    template<typename GraphReprojected>
    network<Graph>::vertex_reprojector<GraphReprojected>::vertex_reprojector(
            const graph_type &graph_original, GraphReprojected &graph_reprojected,
            const boost::geometry::srs::transformation<> &transformation)
            : vertex_all_map_original(boost::get(boost::vertex_all, graph_original)),
              vertex_all_map_reprojected(boost::get(boost::vertex_all, graph_reprojected)),
              transformation{transformation} {}

    template<typename Graph>
    template<typename GraphReprojected>
    void network<Graph>::vertex_reprojector<GraphReprojected>::operator()(
            const original_vertex_descriptor &vertex_original,
            reprojected_vertex_descriptor &vertex_reprojected) const {
        const auto &node_original = boost::get(vertex_all_map_original, vertex_original);

        const original_point_type &point_original = node_original.point;
        reprojected_point_type point_reprojected;

        transformation.forward(point_original, point_reprojected);

        typename GraphReprojected::vertex_property_type node_reprojected
                {node_original.id, std::move(point_reprojected), node_original.tags};
        node_reprojected.index = node_original.index;

        put(vertex_all_map_reprojected, vertex_reprojected, node_reprojected);
    }

    template<typename Graph>
    template<typename GraphReprojected>
    network<Graph>::edge_reprojector<GraphReprojected>::edge_reprojector(
            const graph_type &graph_original, GraphReprojected &graph_reprojected,
            const boost::geometry::srs::transformation<> &transformation)
            : edge_all_map_original{boost::get(boost::edge_all, graph_original)},
              edge_all_map_reprojected{boost::get(boost::edge_all, graph_reprojected)},
              transformation{transformation} {}

    template<typename Graph>
    template<typename GraphReprojected>
    void network<Graph>::edge_reprojector<GraphReprojected>::operator()(
            const original_edge_descriptor &edge_descriptor_original,
            reprojected_edge_descriptor &edge_descriptor_reprojected) const {
        const auto &edge_original = boost::get(
                edge_all_map_original,
                boost::detail::add_reverse_edge_descriptor<original_edge_descriptor, graph_type>::convert(
                        edge_descriptor_original));

        const original_line_type &line_original = edge_original.line;
        reprojected_line_type line_reprojected;

        transformation.forward(line_original, line_reprojected);

        typename GraphReprojected::edge_property_type edge_reprojected
                {edge_original.id, edge_original.nodes, std::move(line_reprojected), edge_original.tags};
        edge_reprojected.index = edge_original.index;

        boost::put(edge_all_map_reprojected, edge_descriptor_reprojected, edge_reprojected);
    }

    template<typename Graph>
    network<Graph>::vertex_visitor::vertex_visitor(
            std::unordered_set<vertex_descriptor> &visited, std::unordered_set<vertex_descriptor> &discovered)
            : _visited{visited}, _discovered{discovered} {}

    template<typename Graph>
    template<typename graph_type_inner>
    void network<Graph>::vertex_visitor::discover_vertex(vertex_descriptor vertex, const graph_type_inner &_graph) {
        _visited.emplace(vertex);
        _discovered.emplace(vertex);
    }

    template
    class network<types_geographic::graph_modifiable>;

    template
    class network<types_cartesian::graph_modifiable>;

    template
    void network<types_geographic::graph_static>::rebuild_spatial_indices();

    template
    void network<types_cartesian::graph_static>::rebuild_spatial_indices();

    template
    std::vector<typename network<types_geographic::graph_static>::vertex_descriptor>
    network<types_geographic::graph_static>::find_node(const osmium::object_id_type node_id) const;

    template
    std::vector<typename network<types_cartesian::graph_static>::vertex_descriptor>
    network<types_cartesian::graph_static>::find_node(const osmium::object_id_type node_id) const;

    template
    std::vector<typename network<types_geographic::graph_static>::edge_descriptor>
    network<types_geographic::graph_static>::find_edge(const osmium::object_id_type edge_id) const;

    template
    std::vector<typename network<types_cartesian::graph_static>::edge_descriptor>
    network<types_cartesian::graph_static>::find_edge(const osmium::object_id_type edge_id) const;

    template
    std::pair<bool, typename network<types_geographic::graph_static>::vertex_descriptor>
    network<types_geographic::graph_static>::point_in_nodes(const point_type &point) const;

    template
    std::pair<bool, typename network<types_cartesian::graph_static>::vertex_descriptor>
    network<types_cartesian::graph_static>::point_in_nodes(const point_type &point) const;

    template
    std::pair<bool, std::pair<std::size_t, typename network<types_geographic::graph_static>::edge_descriptor>>
    network<types_geographic::graph_static>::point_in_edges(const point_type &point) const;

    template
    std::pair<bool, std::pair<std::size_t, typename network<types_cartesian::graph_static>::edge_descriptor>>
    network<types_cartesian::graph_static>::point_in_edges(const point_type &point) const;

    template
    std::pair<bool, typename network<types_geographic::graph_static>::edge_descriptor>
    network<types_geographic::graph_static>::fitting_edge(const segment_type &segment) const;

    template
    std::pair<bool, typename network<types_cartesian::graph_static>::edge_descriptor>
    network<types_cartesian::graph_static>::fitting_edge(const segment_type &segment) const;

    template
    std::vector<typename network<types_geographic::graph_static>::vertex_descriptor>
    network<types_geographic::graph_static>::make_precedessors() const;

    template
    std::vector<typename network<types_cartesian::graph_static>::vertex_descriptor>
    network<types_cartesian::graph_static>::make_precedessors() const;

    template
    std::vector<typename network<types_geographic::graph_static>::length_type>
    network<types_geographic::graph_static>::make_distances() const;

    template
    std::vector<typename network<types_cartesian::graph_static>::length_type>
    network<types_cartesian::graph_static>::make_distances() const;

    template
    std::vector<typename boost::default_color_type> network<types_geographic::graph_static>::make_colors() const;

    template
    std::vector<typename boost::default_color_type> network<types_cartesian::graph_static>::make_colors() const;

    template
    std::vector<std::size_t> network<types_geographic::graph_static>::make_heap_index() const;

    template
    std::vector<std::size_t> network<types_cartesian::graph_static>::make_heap_index() const;

    template
    std::vector<typename network<types_geographic::graph_static>::vertex_descriptor>
    network<types_geographic::graph_static>::make_queue() const;

    template
    std::vector<typename network<types_cartesian::graph_static>::vertex_descriptor>
    network<types_cartesian::graph_static>::make_queue() const;

    template
    std::vector<std::list<typename network<types_geographic::graph_static>::vertex_descriptor>>
    network<types_geographic::graph_static>::dijkstra_shortest_paths(
            const vertex_descriptor &start, const std::unordered_set<vertex_descriptor> &goals,
            const double max_distance_factor, const distance_type max_distance,
            std::vector<vertex_descriptor> &vertices, std::vector<vertex_descriptor> &predecessors,
            std::vector<length_type> &distances, std::vector<typename boost::default_color_type> &colors,
            std::vector<std::size_t> &heap_index, std::vector<vertex_descriptor> &queue) const;

    template
    std::vector<std::list<typename network<types_cartesian::graph_static>::vertex_descriptor>>
    network<types_cartesian::graph_static>::dijkstra_shortest_paths(
            const vertex_descriptor &start, const std::unordered_set<vertex_descriptor> &goals,
            const double max_distance_factor, const distance_type max_distance,
            std::vector<vertex_descriptor> &vertices, std::vector<vertex_descriptor> &predecessors,
            std::vector<length_type> &distances, std::vector<typename boost::default_color_type> &colors,
            std::vector<std::size_t> &heap_index, std::vector<vertex_descriptor> &queue) const;

    template
    typename network<types_geographic::graph_static>::route_type
    network<types_geographic::graph_static>::extract_route(const std::list<vertex_descriptor> &shortest_path) const;

    template
    typename network<types_cartesian::graph_static>::route_type
    network<types_cartesian::graph_static>::extract_route(const std::list<vertex_descriptor> &shortest_path) const;

    template
    typename network<types_geographic::graph_static>::rtree_points_type::bounds_type
    network<types_geographic::graph_static>::bounds() const;

    template
    typename network<types_cartesian::graph_static>::rtree_points_type::bounds_type
    network<types_cartesian::graph_static>::bounds() const;

    template
    void network<types_geographic::graph_modifiable>::convert(
            types_geographic::network_static &network_converted) const;

    template
    void network<types_cartesian::graph_modifiable>::convert(
            types_cartesian::network_static &network_converted) const;

    template
    void network<types_geographic::graph_modifiable>::reproject(
            network<types_cartesian::graph_modifiable> &network_reprojected,
            const boost::geometry::srs::proj4 &from, const boost::geometry::srs::proj4 &to) const;

    template
    void network<types_cartesian::graph_modifiable>::reproject(
            network<types_geographic::graph_modifiable> &network_reprojected,
            const boost::geometry::srs::proj4 &from, const boost::geometry::srs::proj4 &to) const;

    template
    void network<types_geographic::graph_static>::save_nodes(std::string filename) const;

    template
    void network<types_cartesian::graph_static>::save_nodes(std::string filename) const;

    template
    void network<types_geographic::graph_static>::save_edges(std::string filename) const;

    template
    void network<types_cartesian::graph_static>::save_edges(std::string filename) const;

    template
    void network<types_geographic::graph_static>::save(std::string nodes, std::string edges) const;

    template
    void network<types_cartesian::graph_static>::save(std::string nodes, std::string edges) const;

    template
    std::vector<typename network<types_geographic::graph_static>::point_type>
    network<types_geographic::graph_static>::_process_segments_points_result(
            std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor
            >>> &&pre_results) const;

    template
    std::vector<typename network<types_cartesian::graph_static>::point_type>
    network<types_cartesian::graph_static>::_process_segments_points_result(
            std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor
            >>> &&pre_results) const;

    template
    std::set<typename network<types_geographic::graph_static>::edge_descriptor>
    network<types_geographic::graph_static>::_process_segments_unique_result(
            std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor
            >>> &&pre_results) const;

    template
    std::set<typename network<types_cartesian::graph_static>::edge_descriptor>
    network<types_cartesian::graph_static>::_process_segments_unique_result(
            std::vector<std::pair<segment_type, std::pair<std::size_t, edge_descriptor
            >>> &&pre_results) const;

    template
    class network<types_geographic::graph_modifiable>::vertex_reprojector<types_cartesian::graph_modifiable>;

    template
    class network<types_cartesian::graph_modifiable>::vertex_reprojector<types_geographic::graph_modifiable>;

    template
    class network<types_geographic::graph_modifiable>::edge_reprojector<types_cartesian::graph_modifiable>;

    template
    class network<types_cartesian::graph_modifiable>::edge_reprojector<types_geographic::graph_modifiable>;

    template
    class network<types_geographic::graph_static>::vertex_visitor;

    template
    class network<types_cartesian::graph_static>::vertex_visitor;

}
