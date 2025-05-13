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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_HPP

#include "network_base.hpp"

#include <boost/container/set.hpp>
#include <boost/unordered/unordered_flat_set.hpp>

#include "graph/algorithm/dijkstra_shortest_path.hpp"
#include "graph/algorithm/a_star_shortest_path.hpp"

#include "types/geometry/network/route/lazy_route.hpp"

#include "types/io/helper/index_build_helper.hpp"

namespace map_matching_2::geometry::network {

    template<typename GraphHelper, typename IndexHelper>
    class network : public network_base<GraphHelper> {

    public:
        using base_type = network_base<GraphHelper>;

        using graph_helper_type = GraphHelper;
        using tag_helper_type = typename graph_helper_type::tag_helper_type;
        using index_helper_type = IndexHelper;
        using index_build_helper_type = typename index_helper_type::index_build_helper_type;

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
        using coordinate_system_type = typename rich_line_traits<rich_line_type>::coordinate_system_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;

        using route_type = lazy_route_type<coordinate_system_type>;

        using node_pair_type = std::pair<vertex_descriptor, vertex_data_type>;
        using edge_pair_type = std::pair<edge_descriptor, edge_data_type>;

        using rtree_points_type = typename index_helper_type::rtree_points_type;
        using rtree_segments_type = typename index_helper_type::rtree_segments_type;

        constexpr network(graph_helper_type graph_helper = {}, index_helper_type index_helper = {})
            : base_type{std::move(graph_helper)}, _index_helper{std::move(index_helper)} {}

        constexpr network(const network &other) = default;

        constexpr network(network &&other) noexcept = default;

        constexpr network &operator=(const network &other) = default;

        constexpr network &operator=(network &&other) noexcept = default;

        constexpr ~network() = default;

        [[nodiscard]] constexpr const index_helper_type &index_helper() const {
            return _index_helper;
        }

        [[nodiscard]] constexpr index_helper_type &index_helper() {
            return _index_helper;
        }

        [[nodiscard]] constexpr const rtree_points_type &points_index() const {
            return index_helper().points_index();
        }

        [[nodiscard]] constexpr rtree_points_type &points_index() {
            return index_helper().points_index();
        }

        [[nodiscard]] constexpr const rtree_segments_type &segments_index() const {
            return index_helper().segments_index();
        }

        [[nodiscard]] constexpr rtree_segments_type &segments_index() {
            return index_helper().segments_index();
        }

        void build_spatial_indices() requires
            (not io::helper::is_dummy_index_build_helper<index_build_helper_type>) {
            auto vertices_v = this->graph().vertices_view();
            index_helper().index_build_helper().reserve_points(vertices_v.size());
            for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                const vertex_data_type &vertex = vertices_v[it].get();
                index_helper().index_build_helper().add_point(std::pair{vertex.point, it});
            }

            auto edges_v = this->graph().edges_view();
            std::size_t segments_count = 0;
            for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                const edge_data_type &edge = edges_v[it].get();
                if (edge.rich_line.size() >= 1) {
                    segments_count += edge.rich_line.size() - 1;
                }
            }
            index_helper().index_build_helper().reserve_segments(segments_count);
            for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                const edge_data_type &edge = edges_v[it].get();
                if (edge.rich_line.size() >= 1) {
                    for (std::size_t i = 0; i < edge.rich_line.size() - 1; ++i) {
                        index_helper().index_build_helper().add_segment(
                                std::pair{edge.rich_line.segment(i), std::pair{it, i}});
                    }
                }
            }

            index_helper().build_indices();
        }

        [[nodiscard]] std::vector<vertex_descriptor> find_node(std::uint64_t node_id) const {
            std::vector<vertex_descriptor> nodes;
            const auto vertices_v = this->graph().vertices_view();
            for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                const vertex_data_type &vertex = vertices_v[it].get();
                if (vertex.id == node_id) {
                    nodes.emplace_back(it);
                }
            }
            return nodes;
        }

        [[nodiscard]] std::vector<edge_descriptor> find_edge(std::uint64_t edge_id) const {
            std::vector<edge_descriptor> edges;
            const auto edges_v = this->graph().edges_view();
            for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                const edge_data_type &edge = edges_v[it].get();
                if (edge.id == edge_id) {
                    edges.emplace_back(it);
                }
            }
            return edges;
        }

        [[nodiscard]] std::pair<bool, vertex_descriptor> point_in_nodes(const point_type &point) const {
            const auto nearest_points = query_points(boost::geometry::index::nearest(point, 1));
            if (not nearest_points.empty()) {
                const auto &nearest_point_pair = nearest_points.front();
                const auto &nearest_point = nearest_point_pair.first;
                bool found = geometry::equals_points(nearest_point, point);
                const vertex_descriptor &vertex_desc = nearest_point_pair.second;
                return std::pair{found, vertex_desc};
            }
            return std::pair{false, vertex_descriptor{}};
        }

        [[nodiscard]] std::pair<bool, std::pair<std::size_t, std::size_t>>
        point_in_edges(const point_type &point) const {
            const auto nearest_segments = query_segments(boost::geometry::index::nearest(point, 1));
            if (not nearest_segments.empty()) {
                const auto &nearest_segment_pair = nearest_segments.front();
                const auto &nearest_segment = nearest_segment_pair.first;
                bool found = geometry::equals_points(nearest_segment.first, point) or
                        geometry::equals_points(nearest_segment.second, point);
                const auto edge_pair = nearest_segment_pair.second;
                return std::pair{found, edge_pair};
            }
            return std::pair{false, std::pair<std::size_t, std::size_t>{}};
        }

        [[nodiscard]] std::pair<bool, edge_descriptor> fitting_edge(const segment_type &segment) const {
            using angle_type = typename edge_data_type::rich_line_type::angle_type;
            angle_type best_angle = geometry::default_float_type<angle_type>::v180;
            const auto azimuth = geometry::azimuth_segment_deg(segment);
            const edge_descriptor *edge = nullptr;
            const segment_type *best_segment = nullptr;

            const auto segments = query_segments(boost::geometry::index::intersects(segment));
            for (const auto &segment_pair : segments) {
                const segment_type &near_segment = segment_pair.first;
                const auto near_azimuth = geometry::azimuth_segment_deg(near_segment);
                const auto angle_diff = std::fabs(geometry::angle_diff(azimuth, near_azimuth));
                if (best_segment == nullptr or
                    (geometry::equals_points(segment.first, near_segment.first) and
                        geometry::equals_points(segment.second, near_segment.second)) or angle_diff < best_angle) {
                    best_angle = angle_diff;
                    best_segment = &near_segment;
                    edge = &segment_pair.second.first;
                }
            }

            if (best_segment != nullptr) {
                return std::pair{true, *edge};
            }
            return std::pair{false, edge_descriptor{}};
        }

        [[nodiscard]] std::vector<std::list<vertex_descriptor>>
        dijkstra_shortest_paths(const vertex_descriptor &start,
                const boost::unordered_flat_set<vertex_descriptor> &goals,
                distance_type max_distance, double max_distance_factor,
                graph::predecessors_type &predecessors,
                graph::distances_type<length_type> &distances,
                graph::colors_type &colors,
                graph::priority_queue_type<graph::priority_type<length_type, vertex_descriptor>> &queue) const {
            constexpr auto weight_func = [](const edge_data_type &edge) constexpr {
                return edge.rich_line.length();
            };

            boost::unordered_flat_set<vertex_descriptor> goals_found;
            goals_visitor<decltype(weight_func)> visitor{
                    max_distance, max_distance_factor, goals, goals_found, distances, colors, weight_func
            };

            graph::dijkstra_shortest_path(this->graph(), start,
                    predecessors, distances, colors, queue, visitor, weight_func);

            std::vector<std::list<vertex_descriptor>> shortest_paths;
            shortest_paths.reserve(goals.size());
            for (const auto &goal : goals_found) {
                std::list<vertex_descriptor> shortest_path;
                for (vertex_descriptor vertex = goal;; vertex = predecessors[vertex]) {
                    shortest_path.emplace_front(vertex);
                    if (predecessors[vertex] == vertex)
                        break;
                }
                shortest_paths.emplace_back(std::move(shortest_path));
            }

            predecessors.clear();
            distances.clear();
            colors.clear();
            while (not queue.empty()) {
                queue.pop();
            }

            return shortest_paths;
        }

        [[nodiscard]] std::list<vertex_descriptor>
        a_star_shortest_paths(const vertex_descriptor &start,
                const vertex_descriptor &goal,
                distance_type max_distance, double max_distance_factor,
                graph::predecessors_type &predecessors,
                graph::distances_type<length_type> &distances,
                graph::colors_type &colors,
                graph::priority_queue_type<graph::priority_type<length_type, vertex_descriptor>> &queue) const {
            constexpr auto weight_func = [](const edge_data_type &edge) constexpr {
                return edge.rich_line.length();
            };

            const auto heuristic_func = [this](const vertex_descriptor &vertex_desc,
                    const vertex_descriptor &goal_desc) constexpr {
                return geometry::euclidean_distance(
                        this->graph().get_vertex_data(vertex_desc).point,
                        this->graph().get_vertex_data(goal_desc).point);
            };

            goal_visitor<decltype(weight_func)> visitor{
                    max_distance, max_distance_factor, goal, distances, colors, weight_func
            };

            graph::a_star_shortest_path(this->graph(), start, goal,
                    predecessors, distances, colors, queue, visitor, weight_func, heuristic_func);

            std::list<vertex_descriptor> shortest_path;
            for (vertex_descriptor vertex = goal;; vertex = predecessors[vertex]) {
                shortest_path.emplace_front(vertex);
                if (predecessors[vertex] == vertex)
                    break;
            }

            predecessors.clear();
            distances.clear();
            colors.clear();
            while (not queue.empty()) {
                queue.pop();
            }

            return shortest_path;
        }

        [[nodiscard]] route_type extract_route(const std::list<vertex_descriptor> &shortest_path) const {
            if (shortest_path.empty()) {
                // return invalid route
                return route_type{true};
            }

            using rich_line_reference_variant_type = typename route_type::rich_line_reference_variant_type;

            std::vector<rich_line_reference_variant_type> edges;
            edges.reserve(std::distance(shortest_path.cbegin(), shortest_path.cend()) - 1);

            for (auto vertex_it = shortest_path.cbegin(); vertex_it != shortest_path.cend(); ++vertex_it) {
                const auto next_vertex_it = std::next(vertex_it);
                if (next_vertex_it != shortest_path.cend()) {
                    const rich_line_type *rich_line_ptr = nullptr;
                    length_type shortest_length = std::numeric_limits<length_type>::infinity();

                    const auto out_edges_v = this->graph().out_edges_view(*vertex_it);
                    for (auto it = out_edges_v.begin(); it != out_edges_v.end(); it = out_edges_v.next(it)) {
                        if (this->graph().target(it) == *next_vertex_it) {
                            const edge_data_type &edge = this->graph().get_edge_data(it);
                            if (not edge.rich_line.empty() and
                                (not edge.rich_line.has_length() or edge.rich_line.length() < shortest_length)) {
                                if (edge.rich_line.has_length()) {
                                    shortest_length = edge.rich_line.length();
                                }
                                rich_line_ptr = &edge.rich_line;
                            }
                        }
                    }

                    if (rich_line_ptr != nullptr) {
                        edges.emplace_back(std::cref(*rich_line_ptr));
                    }
                }
            }

            if (edges.empty()) {
                // only one node, so no paths found, empty route for staying here
                return route_type{false};
            }

            return route_type{std::move(edges)};
        }

        template<typename Predicate>
        [[nodiscard]] std::vector<typename index_helper_type::rtree_points_value_type>
        query_points(const Predicate &predicate) const {
            std::vector<typename index_helper_type::rtree_points_value_type> results;
            points_index().query(predicate, std::back_inserter(results));
            return results;
        }

        template<typename Predicate>
        void query_segments(const Predicate &predicate,
                std::vector<typename index_helper_type::rtree_segments_value_type> &results) const {
            segments_index().query(predicate, std::back_inserter(results));
        }

        template<typename Predicate>
        [[nodiscard]] std::vector<typename index_helper_type::rtree_segments_value_type>
        query_segments(const Predicate &predicate) const {
            std::vector<typename index_helper_type::rtree_segments_value_type> results;
            query_segments(predicate, results);
            return results;
        }

        template<typename Predicate>
        [[nodiscard]] std::vector<point_type>
        query_segments_points(const Predicate &predicate) const {
            return _process_segments_points_result(query_segments(predicate));
        }

        template<typename Predicate>
        [[nodiscard]] boost::container::set<edge_descriptor>
        query_segments_unique(const Predicate &predicate) const {
            return _process_segments_unique_result(query_segments(predicate));
        }

        [[nodiscard]] typename rtree_points_type::bounds_type bounds() const {
            return points_index().bounds();
        }

    private:
        index_helper_type _index_helper;

        [[nodiscard]] std::vector<point_type> _process_segments_points_result(
                std::vector<typename index_helper_type::rtree_segments_value_type> &&pre_results) const {
            std::vector<point_type> results;
            results.reserve(pre_results.size() * 2);
            for (std::size_t i = 0; i < pre_results.size(); ++i) {
                auto &pre_result = pre_results[i];
                results.emplace_back(std::move(pre_result.first.first));
                results.emplace_back(std::move(pre_result.first.second));
            }
            return results;
        }

        [[nodiscard]] boost::container::set<edge_descriptor> _process_segments_unique_result(
                std::vector<typename index_helper_type::rtree_segments_value_type> &&pre_results) const {
            boost::container::set<edge_descriptor> results;
            for (const auto &pre_result : pre_results) {
                results.emplace(pre_result.second.first);
            }
            return results;
        }

        template<typename WeightFunc>
        class base_max_distance_visitor : public graph::visitor {

        public:
            constexpr base_max_distance_visitor(const distance_type max_distance,
                    const double max_distance_factor,
                    graph::distances_type<length_type> &distances,
                    graph::colors_type &colors,
                    const WeightFunc &weight_func)
                : _max_distance{max_distance}, _max_distance_factor{max_distance_factor}, _distances{distances},
                _colors{colors}, _weight_func{weight_func} {}

            void examine_edge(const graph_type &graph,
                    const typename graph::graph_traits<graph_type>::edge_descriptor &edge_desc) {
                if (_max_distance_factor < 0.0) {
                    // effectively disable this method if the max-distance-factor is disabled
                    return;
                }

                const vertex_descriptor &source = graph.source(edge_desc);
                const auto distance_source = _distances[source];
                const auto weight = _weight_func(graph.get_edge_data(edge_desc));
                const auto distance_target = distance_source + weight;
                if (distance_target > _max_distance_factor * _max_distance) {
                    // disable target
                    const vertex_descriptor &target = graph.target(edge_desc);
                    _colors[target] = graph::color::BLACK;
                }
            }

        private:
            const distance_type _max_distance;
            const double _max_distance_factor;

            graph::distances_type<length_type> &_distances;
            graph::colors_type &_colors;
            const WeightFunc &_weight_func;

        };

        template<typename WeightFunc>
        class goals_visitor : public base_max_distance_visitor<WeightFunc> {

        public:
            using base_type = base_max_distance_visitor<WeightFunc>;

            constexpr goals_visitor(const distance_type max_distance,
                    const double max_distance_factor,
                    const boost::unordered_flat_set<vertex_descriptor> &goals,
                    boost::unordered_flat_set<vertex_descriptor> &goals_found,
                    graph::distances_type<length_type> &distances,
                    graph::colors_type &colors,
                    const WeightFunc &weight_func)
                : base_type{max_distance, max_distance_factor, distances, colors, weight_func},
                _goals{goals}, _goals_found{goals_found} {}

            void finish_vertex(const graph_type &graph,
                    const typename graph::graph_traits<graph_type>::vertex_descriptor &vertex_desc) {
                if (_goals.contains(vertex_desc)) {
                    _goals_found.emplace(vertex_desc);
                }

                if (_goals_found.size() >= _goals.size()) {
                    this->stop();
                }
            }

        private:
            const boost::unordered_flat_set<vertex_descriptor> &_goals;
            boost::unordered_flat_set<vertex_descriptor> &_goals_found;

        };

        template<typename WeightFunc>
        class goal_visitor : public base_max_distance_visitor<WeightFunc> {

        public:
            using base_type = base_max_distance_visitor<WeightFunc>;

            constexpr goal_visitor(const distance_type max_distance,
                    const double max_distance_factor,
                    const vertex_descriptor &goal,
                    graph::distances_type<length_type> &distances,
                    graph::colors_type &colors,
                    const WeightFunc &weight_func)
                : base_type{max_distance, max_distance_factor, distances, colors, weight_func},
                _goal{goal} {}

            void finish_vertex(const graph_type &graph,
                    const typename graph::graph_traits<graph_type>::vertex_descriptor &vertex_desc) {
                if (_goal == vertex_desc) {
                    this->stop();
                }
            }

        private:
            const vertex_descriptor &_goal;

        };

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_HPP
