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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_IMPORT_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_IMPORT_HPP

#include "network_base.hpp"

#include <boost/unordered/unordered_flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>

#include "geometry/algorithm/reverse.hpp"

#include "graph/algorithm/breadth_first_search.hpp"

namespace map_matching_2::geometry::network {

    template<typename GraphHelper>
    class network_import : public network_base<GraphHelper> {

    public:
        using base_type = network_base<GraphHelper>;

        using graph_helper_type = GraphHelper;
        using tag_helper_type = typename graph_helper_type::tag_helper_type;

        using graph_type = typename graph_helper_type::graph_type;
        using vertex_data_type = typename graph::graph_traits<graph_type>::vertex_data_type;
        using edge_data_type = typename graph::graph_traits<graph_type>::edge_data_type;
        using vertex_descriptor = typename graph::graph_traits<graph_type>::vertex_descriptor;
        using vertex_size_type = typename graph::graph_traits<graph_type>::vertex_size_type;
        using edge_descriptor = typename graph::graph_traits<graph_type>::edge_descriptor;
        using edge_size_type = typename graph::graph_traits<graph_type>::edge_size_type;

        using rich_line_type = typename edge_data_type::rich_line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;

        constexpr network_import(graph_helper_type graph_helper = {})
            : base_type{std::move(graph_helper)} {}

        constexpr network_import(const network_import &other) = default;

        constexpr network_import(network_import &&other) noexcept = default;

        constexpr network_import &operator=(const network_import &other) = default;

        constexpr network_import &operator=(network_import &&other) noexcept = default;

        constexpr ~network_import() = default;

        void simplify(const bool simplify_network_complete = true) {
            bool was_running = _simplify_running;
            _simplify_running = true;

            graph_type &graph = this->graph();
            auto vertices_v = graph.vertices_view();

            std::vector<std::pair<edge_descriptor, edge_descriptor>> adjacent_edges;
            std::vector<std::pair<edge_descriptor, edge_descriptor>> merge_edges;

            vertex_descriptor vertex_desc, vertex_val;
            vertex_desc = vertices_v.begin();
            for (vertex_val = vertex_desc; vertex_desc != vertices_v.end(); vertex_desc = vertex_val) {
                vertex_val = vertices_v.next(vertex_val);

                if (was_running and _simplify_vertex_set) {
                    const vertex_size_type vertex_index = graph.vertex_index(vertex_desc);
                    if (vertex_index != _simplify_vertex) {
                        // skip all vertices from the previous run
                        continue;
                    } else {
                        // stop skipping from now on
                        was_running = false;
                    }
                }

                if (_is_mergeable(vertex_desc, adjacent_edges)) {
                    _populate_merges(vertex_desc, merge_edges, simplify_network_complete);

                    if (not merge_edges.empty()) {
                        // mark current vertex index, in case of an error and restart, it can continue from here
                        _simplify_vertex = graph.vertex_index(vertex_desc);
                        _simplify_vertex_set = true;

                        for (const auto &merge_edge : merge_edges) {
                            _merge(merge_edge.first, merge_edge.second);
                        }

                        merge_edges.clear();
                    }
                }
            }

            graph.reindex();

            _simplify_running = false;
            _simplify_vertex_set = false;
        }

        void remove_unconnected_components() {
            boost::unordered_flat_set<std::size_t> visited;
            graph::queue_type<vertex_descriptor> queue;

            using discovered_type = std::vector<vertex_descriptor>;
            std::vector<discovered_type> components;

            const auto vertices_v = this->graph().vertices_view();
            for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                const auto vertex_index = this->graph().vertex_index(it);
                if (not visited.contains(vertex_index)) {
                    auto &discovered = components.emplace_back(discovered_type{});
                    vertex_visitor vertex_visitor{discovered};

                    graph::breadth_first_search(this->graph(), it, visited, queue, vertex_visitor);
                }
            }

            const auto component_comparator = [](const auto &a, const auto &b) {
                return a.size() < b.size();
            };
            const auto largest_component_it = std::max_element(
                    components.cbegin(), components.cend(), component_comparator);
            const auto &largest_component = *largest_component_it;

            // found largest weakly component, remove all remaining nodes and edges
            for (const discovered_type &component_nodes : components) {
                if (&component_nodes != &largest_component) {
                    // only remove nodes not from the largest component
                    for (const vertex_descriptor vertex_desc : component_nodes) {
                        this->graph().clear_vertex(vertex_desc);
                        this->graph().remove_vertex(vertex_desc);
                    }
                }
            }

            this->graph().reindex();
        }

        void remove_duplicate_nodes(const bool simplify = true, const bool simplify_network_complete = true) {
            using out_edge_descriptor = typename graph_type::out_edge_descriptor;
            using in_edge_descriptor = typename graph_type::in_edge_descriptor;

            boost::unordered_flat_map<point_type, vertex_descriptor> node_map;

            std::vector<std::pair<edge_descriptor, edge_descriptor>> adjacent_edges;
            std::vector<std::pair<edge_descriptor, edge_descriptor>> merge_edges;

            graph_type &graph = this->graph();
            auto vertices_v = graph.vertices_view();
            auto edges_v = graph.edges_view();

            vertex_descriptor vertex_desc, vertex_val;
            vertex_desc = vertices_v.begin();
            for (vertex_val = vertex_desc; vertex_desc != vertices_v.end(); vertex_desc = vertex_val) {
                vertex_val = vertices_v.next(vertex_val);

                const vertex_data_type &vertex = vertices_v[vertex_desc].get();
                const point_type &point = vertex.point;

                auto it = node_map.find(point);
                if (it != node_map.end()) {
                    // duplicate node found with same coordinates
                    vertex_descriptor existing_vertex_desc = it->second;

                    auto in_edges_v = graph.in_edges_view(vertex_desc);

                    in_edge_descriptor in_edge_desc, in_edge_val;
                    in_edge_desc = in_edges_v.begin();
                    for (in_edge_val = in_edge_desc; in_edge_desc != in_edges_v.end(); in_edge_desc = in_edge_val) {
                        in_edge_val = in_edges_v.next(in_edge_val);

                        const edge_descriptor &in_edge = in_edges_v[in_edge_desc];
                        edge_data_type &edge = edges_v[in_edge].get();
                        const vertex_descriptor source_vertex = graph.source(in_edge);

                        graph.add_edge(source_vertex, existing_vertex_desc, std::move(edge));
                        graph.remove_edge(in_edge);
                    }

                    auto out_edges_v = graph.out_edges_view(vertex_desc);

                    out_edge_descriptor out_edge_desc, out_edge_val;
                    out_edge_desc = out_edges_v.begin();
                    for (out_edge_val = out_edge_desc; out_edge_desc != out_edges_v.end();
                         out_edge_desc = out_edge_val) {
                        out_edge_val = out_edges_v.next(out_edge_val);

                        const edge_descriptor &out_edge = out_edges_v[out_edge_desc];
                        edge_data_type &edge = edges_v[out_edge].get();
                        const vertex_descriptor target_vertex = graph.target(out_edge);

                        graph.add_edge(existing_vertex_desc, target_vertex, std::move(edge));
                        graph.remove_edge(out_edge);
                    }

                    assert(graph.degree(vertex_desc) == 0);
                    graph.remove_vertex(vertex_desc);

                    if (simplify) {
                        // check if current node can be omitted
                        if (_is_mergeable(existing_vertex_desc, adjacent_edges)) {
                            _populate_merges(existing_vertex_desc, merge_edges, simplify_network_complete);

                            if (not merge_edges.empty()) {
                                for (const auto &merge_edge : merge_edges) {
                                    _merge(merge_edge.first, merge_edge.second);
                                }

                                merge_edges.clear();
                            }

                            node_map.erase(it);
                        }
                    }
                } else {
                    node_map.emplace(point, vertex_desc);
                }
            }

            graph.reindex();
        }

        void remove_duplicate_edges() {
            bool reindex = false;
            boost::unordered_flat_set<std::reference_wrapper<const edge_data_type>,
                boost::hash<edge_data_type>, std::equal_to<edge_data_type>> edge_set;

            graph_type &graph = this->graph();
            const auto edges_v = graph.edges_view();

            edge_descriptor edge_desc, edge_val;
            edge_desc = edges_v.begin();
            for (edge_val = edge_desc; edge_desc != edges_v.end(); edge_desc = edge_val) {
                edge_val = edges_v.next(edge_val);

                const edge_data_type &edge = edges_v[edge_desc].get();

                if (edge_set.contains(edge)) {
                    graph.remove_edge(edge_desc);
                    reindex = true;
                }

                edge_set.emplace(edge);
            }

            if (reindex) {
                graph.reindex();
            }
        }

    private:
        bool _simplify_running{false}, _simplify_vertex_set{false};
        vertex_size_type _simplify_vertex{};

        void _merge(const edge_descriptor &in_edge, const edge_descriptor &out_edge) {
            graph_type &graph = this->graph();

            const vertex_descriptor source_vertex_in = graph.source(in_edge);
            const vertex_descriptor target_vertex_in = graph.target(in_edge);
            const vertex_descriptor target_vertex_out = graph.target(out_edge);

            edge_data_type &source_edge = graph.get_edge_data(in_edge);
            edge_data_type &target_edge = graph.get_edge_data(out_edge);

            // merge the edges
            this->tag_helper().merge_edge_tags(source_edge.id, target_edge.id);
            edge_data_type new_edge = edge_data_type::merge(std::move(source_edge), std::move(target_edge));

            // remove the original edges
            graph.remove_edge(in_edge);
            graph.remove_edge(out_edge);

            // add newly merged edge in place of the original edges
            graph.add_edge(source_vertex_in, target_vertex_out, std::move(new_edge));

            if (graph.degree(target_vertex_in) == 0) {
                // only remove vertex if all edges were previously removed
                graph.remove_vertex(target_vertex_in);
            }
        }

        [[nodiscard]] bool _is_mergeable(const vertex_descriptor &vertex_desc,
                std::vector<std::pair<edge_descriptor, edge_descriptor>> &adjacent_edges) const {
            const graph_type &graph = this->graph();

            auto in_edges_v = graph.in_edges_view(vertex_desc);
            auto out_edges_v = graph.out_edges_view(vertex_desc);

            const edge_size_type out_degree = out_edges_v.size();
            const edge_size_type in_degree = in_edges_v.size();

            if (in_degree == 1 and out_degree == 1) {
                // we have two edges in a one-way pattern, this certainly can be merged
                return true;
            }

            const auto degree = out_degree + in_degree;
            if (not((degree == 2 or degree == 4) and in_degree == out_degree)) {
                // degree is not 2 or 4 with the in and out degree being the same, so not mergable
                return false;
            }

            adjacent_edges.clear();

            for (auto in_it = in_edges_v.begin();
                 in_it != in_edges_v.end(); in_it = in_edges_v.next(in_it)) {
                const edge_descriptor &in_edge = in_edges_v[in_it];

                for (auto out_it = out_edges_v.begin();
                     out_it != out_edges_v.end(); out_it = out_edges_v.next(out_it)) {
                    const edge_descriptor &out_edge = out_edges_v[out_it];

                    if (out_edge == in_edge) {
                        // skip same underlying edge, we need to check the reverse direction edges
                        continue;
                    }

                    const vertex_descriptor source_vertex_in = graph.source(in_edge);
                    const vertex_descriptor target_vertex_in = graph.target(in_edge);
                    const vertex_descriptor source_vertex_out = graph.source(out_edge);
                    const vertex_descriptor target_vertex_out = graph.target(out_edge);

                    if (target_vertex_in == source_vertex_out and source_vertex_in == target_vertex_out) {
                        // these are reverse direction edges
                        const edge_data_type &source_edge = graph.get_edge_data(in_edge);
                        const edge_data_type &target_edge = graph.get_edge_data(out_edge);

                        const auto find_it = std::find_if(std::cbegin(adjacent_edges), std::cend(adjacent_edges),
                                [&out_edge](const auto &pair) -> bool {
                                    return pair.second == out_edge;
                                });
                        if (find_it == std::cend(adjacent_edges)) {
                            // out_edge not already matching to another in_edge, proceed
                            if (geometry::lines_are_reversed(source_edge.rich_line.line(),
                                    target_edge.rich_line.line())) {
                                // this pair of edges fits together
                                adjacent_edges.emplace_back(std::pair{in_edge, out_edge});
                            }
                        }
                    }
                }

                // check if in_edge got a match
                const auto find_it = std::find_if(std::cbegin(adjacent_edges), std::cend(adjacent_edges),
                        [&in_edge](const auto &pair) -> bool {
                            return pair.first == in_edge;
                        });
                if (find_it == std::cend(adjacent_edges)) {
                    // did not find a matching out_edge for the current in_edge, so merging is impossible
                    return false;
                }
            }

            // check if all out_edges got matches
            for (auto out_it = out_edges_v.begin();
                 out_it != out_edges_v.end(); out_it = out_edges_v.next(out_it)) {
                const edge_descriptor &out_edge = out_edges_v[out_it];

                const auto find_it = std::find_if(std::cbegin(adjacent_edges), std::cend(adjacent_edges),
                        [&out_edge](const auto &pair) -> bool {
                            return pair.second == out_edge;
                        });
                if (find_it == std::cend(adjacent_edges)) {
                    // did not find a matching in_edge for the current out_edge, so merging is impossible
                    return false;
                }
            }

            // all edge pairs matched and they should match the degree, then they can be merged
            return 2 * adjacent_edges.size() == out_degree + in_degree;
        }

        void _populate_merges(const vertex_descriptor &vertex_desc,
                std::vector<std::pair<edge_descriptor, edge_descriptor>> &merge_edges,
                const bool simplify_network_complete = true) const {
            const graph_type &graph = this->graph();

            auto in_edges_v = graph.in_edges_view(vertex_desc);
            auto out_edges_v = graph.out_edges_view(vertex_desc);

            for (auto in_it = in_edges_v.begin();
                 in_it != in_edges_v.end(); in_it = in_edges_v.next(in_it)) {
                const edge_descriptor &in_edge = in_edges_v[in_it];

                for (auto out_it = out_edges_v.begin();
                     out_it != out_edges_v.end(); out_it = out_edges_v.next(out_it)) {
                    const edge_descriptor &out_edge = out_edges_v[out_it];

                    if (out_edge == in_edge) {
                        // cannot merge same underlying edge, skip
                        continue;
                    }

                    const vertex_descriptor source_vertex_in = graph.source(in_edge);
                    const vertex_descriptor target_vertex_in = graph.target(in_edge);
                    const vertex_descriptor source_vertex_out = graph.source(out_edge);
                    const vertex_descriptor target_vertex_out = graph.target(out_edge);

                    if (target_vertex_in == source_vertex_out and source_vertex_in != target_vertex_out) {
                        // these are edges that can be merged, since they don't loop in themselves
                        const edge_data_type &source_edge = graph.get_edge_data(in_edge);
                        const edge_data_type &target_edge = graph.get_edge_data(out_edge);

                        const auto &source_tags = this->tag_helper().edge_tags(source_edge.id);
                        const auto &target_tags = this->tag_helper().edge_tags(target_edge.id);

                        if (simplify_network_complete or
                            (source_edge.id == target_edge.id and source_tags == target_tags)) {
                            // only merge edges when they fit together concerning their ids and tags,
                            // for example maxspeed change within road may be retained, if not ignored

                            const auto find_it = std::find_if(std::cbegin(merge_edges), std::cend(merge_edges),
                                    [&](const auto &pair) -> bool {
                                        return pair.first == in_edge or pair.second == out_edge;
                                    });
                            if (find_it == std::cend(merge_edges)) {
                                // edges not already added, can be merged once
                                merge_edges.emplace_back(std::pair{in_edge, out_edge});
                            }
                        }
                    }
                }
            }
        }

        class vertex_visitor : public graph::visitor {

        public:
            explicit constexpr vertex_visitor(std::vector<vertex_descriptor> &discovered)
                : _discovered{discovered} {}

            template<typename Graph>
            void discover_vertex(const Graph &graph, typename Graph::vertex_descriptor vertex_desc) {
                _discovered.emplace_back(vertex_desc);
            }

        private:
            std::vector<vertex_descriptor> &_discovered;

        };

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_IMPORT_HPP
