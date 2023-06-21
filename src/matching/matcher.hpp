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

#ifndef MAP_MATCHING_2_MATCHER_HPP
#define MAP_MATCHING_2_MATCHER_HPP

#ifndef BOOST_THREAD_VERSION
#define BOOST_THREAD_VERSION 5
#endif

#include <absl/container/flat_hash_map.h>
#include <absl/container/btree_set.h>
#include <absl/container/btree_map.h>

#include <boost/thread.hpp>
#include <boost/thread/executors/basic_thread_pool.hpp>

#include <geometry/network/network.hpp>
#include <geometry/track/multi_track.hpp>
#include <geometry/substring.hpp>
#include <geometry/util.hpp>
#include <io/csv_exporter.hpp>
#include <util/benchmark.hpp>
#include <learning/settings.hpp>

#include "candidate.hpp"
#include "settings.hpp"
#include "detector.hpp"

namespace map_matching_2::matching {

    template<typename Network, typename MultiTrack>
    class matcher {

    public:
        using network_type = Network;
        using graph_type = typename geometry::network_traits<network_type>::graph_type;
        using node_type = typename geometry::network_traits<network_type>::node_type;
        using edge_type = typename geometry::network_traits<network_type>::edge_type;
        using vertex_descriptor = typename geometry::network_traits<network_type>::vertex_descriptor;
        using edge_descriptor = typename geometry::network_traits<network_type>::edge_descriptor;
        using point_type = typename geometry::network_traits<network_type>::point_type;
        using rich_line_type = typename geometry::network_traits<network_type>::rich_line_type;

        using route_type = geometry::network::route<rich_line_type>;

        using multi_track_type = MultiTrack;
        using track_type = typename multi_track_type::track_type;
        using measurement_type = typename track_type::measurement_type;

        using coordinate_type = typename geometry::rich_line_traits<edge_type>::coordinate_type;
        using distance_type = typename geometry::rich_line_traits<edge_type>::distance_type;
        using length_type = typename geometry::rich_line_traits<edge_type>::length_type;

        using candidate_node_type = candidate_node<Network>;
        using candidate_edge_type = candidate_edge<Network>;
        using candidate_type = candidate<Network, track_type>;

        using candidate_node_distance_comparator_type = decltype(&candidate_node_type::distance_comparator);
        using candidate_edge_distance_comparator_type = decltype(&candidate_edge_type::distance_comparator);

        const Network &network;

        explicit matcher(const Network &network, bool single_threaded = false)
                : matcher{network, single_threaded ? 1 : boost::thread::hardware_concurrency()} {}

        explicit matcher(const Network &network, std::uint32_t threads = boost::thread::hardware_concurrency())
                : network{network}, _single_threaded{threads == 1}, _matching_thread_pool{threads} {}

        [[nodiscard]] std::vector<candidate_type> candidate_search_buffer(
                const track_type &track, double buffer_radius, double buffer_upper_radius = 10000.0,
                double buffer_lower_radius = 200.0, bool adaptive_radius = true,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) {
            std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> candidate_edge_sets{
                    track.size(), std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>{
                            &candidate_edge_type::distance_comparator}};

            std::vector<double> radii;
            radii.reserve(track.size());

            for (std::size_t i = 0; i < track.size(); ++i) {
                radii.emplace_back(_candidate_search_buffer_measurement(
                        candidate_edge_sets[i], 1, track, i, buffer_radius, buffer_upper_radius, buffer_lower_radius,
                        adaptive_radius));
            }

            std::vector<candidate_type> candidates;
            candidates.reserve(candidate_edge_sets.size());
            _finalize_candidates_buffer(
                    candidates, 1, track, radii, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby, candidate_adoption_reverse);

            return candidates;
        }

        [[nodiscard]] std::vector<candidate_type> candidate_search_nearest(
                const track_type &track, std::size_t number, bool with_reverse = false, bool with_adjacent = false,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) {
            std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> candidate_edge_sets{
                    track.size(), std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>{
                            &candidate_edge_type::distance_comparator}};

            std::vector<std::size_t> numbers;
            numbers.reserve(track.size());

            for (std::size_t i = 0; i < track.size(); ++i) {
                numbers.emplace_back(_candidate_search_nearest_measurement(
                        candidate_edge_sets[i], 1, track, i, number, with_reverse, with_adjacent));
            }

            std::vector<candidate_type> candidates;
            candidates.reserve(candidate_edge_sets.size());
            _finalize_candidates_nearest(
                    candidates, 1, track, numbers, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby, candidate_adoption_reverse);

            return candidates;
        }

        [[nodiscard]] std::vector<candidate_type> candidate_search_combined(
                const track_type &track, double buffer_radius, std::size_t number, double buffer_upper_radius = 10000.0,
                double buffer_lower_radius = 200.0, bool adaptive_radius = true, bool with_reverse = false,
                bool with_adjacent = false, bool candidate_adoption_siblings = false,
                bool candidate_adoption_nearby = false, bool candidate_adoption_reverse = false) {
            std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> candidate_edge_sets{
                    track.size(), std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>{
                            &candidate_edge_type::distance_comparator}};

            std::vector<std::pair<double, std::size_t>> radii_numbers;
            radii_numbers.reserve(track.size());

            for (std::size_t i = 0; i < track.size(); ++i) {
                radii_numbers.emplace_back(_candidate_search_combined_measurement(
                        candidate_edge_sets[i], 1, track, i, buffer_radius, number, buffer_upper_radius,
                        buffer_lower_radius, adaptive_radius, with_reverse, with_adjacent));
            }

            std::vector<candidate_type> candidates;
            candidates.reserve(candidate_edge_sets.size());
            _finalize_candidates_combined(
                    candidates, 1, track, radii_numbers, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby, candidate_adoption_reverse);

            return candidates;
        }

        [[nodiscard]] std::vector<candidate_type> candidate_search(
                const track_type &track, double buffer_radius, std::size_t number, bool buffer_candidate_search = true,
                bool nearest_candidate_search = false, double buffer_upper_radius = 10000.0,
                double buffer_lower_radius = 200.0, bool adaptive_radius = true, bool with_reverse = false,
                bool with_adjacent = false, bool candidate_adoption_siblings = false,
                bool candidate_adoption_nearby = false, bool candidate_adoption_reverse = false) {
            if (buffer_candidate_search and not nearest_candidate_search) {
                return candidate_search_buffer(
                        track, buffer_radius, buffer_upper_radius, buffer_lower_radius, adaptive_radius,
                        candidate_adoption_siblings, candidate_adoption_nearby, candidate_adoption_reverse);
            } else if (not buffer_candidate_search and nearest_candidate_search) {
                return candidate_search_nearest(
                        track, number, with_reverse, with_adjacent,
                        candidate_adoption_siblings, candidate_adoption_nearby, candidate_adoption_reverse);
            } else if (buffer_candidate_search and nearest_candidate_search) {
                return candidate_search_combined(
                        track, buffer_radius, number, buffer_upper_radius, buffer_lower_radius,
                        adaptive_radius, with_reverse, with_adjacent,
                        candidate_adoption_siblings, candidate_adoption_nearby, candidate_adoption_reverse);
            } else {
                throw std::invalid_argument(
                        "invalid candidate search arguments, either buffer, nearest or both needs to be enabled");
            }
        }

        void resize_candidates_buffer(
                const track_type &track, std::vector<candidate_type> &candidates,
                const std::vector<std::size_t> &positions,
                std::size_t round, double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0,
                bool adaptive_radius = true, bool adaptive_resize = true, bool candidate_adoption_siblings = false,
                bool candidate_adoption_nearby = false, bool candidate_adoption_reverse = false) {
            std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> candidate_edge_sets{
                    candidates.size(), std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>{
                            &candidate_edge_type::distance_comparator}};

            std::vector<double> radii;
            radii.reserve(candidates.size());

            for (std::size_t i = 0; i < candidates.size(); ++i) {
                auto &candidate = candidates[i];
                radii.emplace_back(candidate.radius);
                auto &candidate_edges = candidate.edges;
                auto &candidate_edge_set = candidate_edge_sets[i];
                for (auto &candidate_edge: candidate_edges) {
                    candidate_edge_set.emplace(std::move(candidate_edge));
                }
            }

            absl::btree_map<std::size_t, double> position_map;
            for (const auto i: positions) {
                position_map.emplace(i, radii[i] * 2);
            }

            if (adaptive_resize) {
                using index_value_type = std::pair<point_type, std::size_t>;

                std::vector<index_value_type> measurement_index_values;
                measurement_index_values.reserve(track.size());
                for (std::size_t i = 0; i < track.size(); ++i) {
                    const point_type &point = track.at(i);
                    measurement_index_values.emplace_back(std::pair{point, i});
                }

                boost::geometry::index::rtree<index_value_type, boost::geometry::index::rstar<16>>
                        measurements_index{std::move(measurement_index_values)};

                for (const auto i: positions) {
                    const point_type &point = track.at(i);
                    auto &edge_set = candidate_edge_sets[i];
                    auto edge_set_it = edge_set.crbegin();
                    const auto longest_distance = edge_set_it->distance;

                    const auto buffer_radius = radii[i] * 2;
                    const auto buffer = geometry::buffer_box(point, buffer_radius);

                    std::vector<index_value_type> results;
                    measurements_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(results));

                    for (const auto &result: results) {
                        if (geometry::point_distance(point, result.first) <= buffer_radius) {
                            const auto j = result.second;
                            position_map.emplace(j, radii[j] * 2);
                        }
                    }
                }
            }

            for (const auto &pos: position_map) {
                const auto i = pos.first;
                const auto radius = pos.second;
                // disable adaptive radius this time as we used it the previous time if it was enabled
                radii[i] = _candidate_search_buffer_measurement(
                        candidate_edge_sets[i], round, track, i, radius, buffer_upper_radius, buffer_lower_radius,
                        false);
            }

            _finalize_candidates_buffer(
                    candidates, round, track, radii, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby, candidate_adoption_reverse);
        }

        void resize_candidates_nearest(
                const track_type &track, std::vector<candidate_type> &candidates,
                const std::vector<std::size_t> &positions,
                std::size_t round, bool with_reverse = false, bool with_adjacent = false,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) {
            std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> candidate_edge_sets{
                    candidates.size(), std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>{
                            &candidate_edge_type::distance_comparator}};

            std::vector<std::size_t> numbers;
            numbers.reserve(candidates.size());

            for (std::size_t i = 0; i < candidates.size(); ++i) {
                auto &candidate = candidates[i];
                numbers.emplace_back(candidate.number);
                auto &candidate_edges = candidate.edges;
                auto &candidate_edge_set = candidate_edge_sets[i];
                for (auto &candidate_edge: candidate_edges) {
                    candidate_edge_set.emplace(std::move(candidate_edge));
                }
            }

            for (const auto i: positions) {
                numbers[i] = _candidate_search_nearest_measurement(
                        candidate_edge_sets[i], round, track, i, numbers[i] * 2, with_reverse, with_adjacent);
            }

            _finalize_candidates_nearest(
                    candidates, round, track, numbers, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby, candidate_adoption_reverse);
        }

        void resize_candidates_combined(
                const track_type &track, std::vector<candidate_type> &candidates,
                const std::vector<std::size_t> &positions,
                std::size_t round, double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0,
                bool adaptive_radius = true, bool adaptive_resize = true, bool with_reverse = false,
                bool with_adjacent = false, bool candidate_adoption_siblings = false,
                bool candidate_adoption_nearby = false, bool candidate_adoption_reverse = false) {
            std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> candidate_edge_sets{
                    candidates.size(), std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>{
                            &candidate_edge_type::distance_comparator}};

            std::vector<std::pair<double, std::size_t>> radii_numbers;
            radii_numbers.reserve(candidates.size());

            for (std::size_t i = 0; i < candidates.size(); ++i) {
                auto &candidate = candidates[i];
                radii_numbers.emplace_back(std::pair{candidate.radius, candidate.number});
                auto &candidate_edges = candidate.edges;
                auto &candidate_edge_set = candidate_edge_sets[i];
                for (auto &candidate_edge: candidate_edges) {
                    candidate_edge_set.emplace(std::move(candidate_edge));
                }
            }

            absl::btree_map<std::size_t, double> position_map;
            for (const auto i: positions) {
                position_map.emplace(i, radii_numbers[i].first * 2);
            }

            if (adaptive_resize) {
                using index_value_type = std::pair<point_type, std::size_t>;

                std::vector<index_value_type> measurement_index_values;
                measurement_index_values.reserve(track.size());
                for (std::size_t i = 0; i < track.size(); ++i) {
                    const point_type &point = track.at(i);
                    measurement_index_values.emplace_back(std::pair{point, i});
                }

                boost::geometry::index::rtree<index_value_type, boost::geometry::index::rstar<16>>
                        measurements_index{std::move(measurement_index_values)};

                for (const auto i: positions) {
                    const point_type &point = track.at(i);
                    auto &edge_set = candidate_edge_sets[i];
                    auto edge_set_it = edge_set.crbegin();
                    const auto longest_distance = edge_set_it->distance;

                    const auto buffer_radius = radii_numbers[i].first * 2;
                    const auto buffer = geometry::buffer(point, buffer_radius);

                    std::vector<index_value_type> results;
                    measurements_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(results));

                    for (const auto &result: results) {
                        if (geometry::point_distance(point, result.first) <= buffer_radius) {
                            const auto j = result.second;
                            position_map.emplace(j, radii_numbers[j].first * 2);
                        }
                    }
                }
            }

            for (const auto &pos: position_map) {
                const auto i = pos.first;
                const auto radius = pos.second;
                // disable adaptive radius this time as we used it the previous time if it was enabled
                radii_numbers[i] = _candidate_search_combined_measurement(
                        candidate_edge_sets[i], round, track, i, radius, radii_numbers[i].second * 2,
                        buffer_upper_radius,
                        buffer_lower_radius, false, with_reverse, with_adjacent);
            }

            _finalize_candidates_combined(
                    candidates, round, track, radii_numbers, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby, candidate_adoption_reverse);
        }

        [[nodiscard]] std::vector<std::list<vertex_descriptor>> shortest_paths(
                const std::vector<candidate_type> &candidates,
                std::size_t from, std::size_t source, std::size_t to, double max_distance_factor = 10.0) {
            const auto &candidate_edge_start = candidates[from].edges[source];
            const auto vertex_start = boost::target(candidate_edge_start.edge_descriptor, network.graph());
            const auto &node_start = network.vertex(vertex_start);

            distance_type max_distance = geometry::default_float_type<distance_type>::v0;
            absl::flat_hash_set<typename boost::graph_traits<graph_type>::vertex_descriptor> goals;
            for (const auto &candidate_node: candidates[to].nodes) {
                const auto vertex_goal = candidate_node.vertex_descriptor;
                goals.emplace(vertex_goal);

                const auto &node_goal = network.vertex(vertex_goal);
                const auto distance = geometry::point_distance(node_start.point, node_goal.point);
                if (distance > max_distance) {
                    max_distance = distance;
                }
            }

            _prepare_shortest_path_memory();
            return network.dijkstra_shortest_paths(vertex_start, goals, max_distance_factor, max_distance,
                                                   *_vertices, *_predecessors, *_distances, *_colors,
                                                   *_heap_index, *_queue);
        }

        [[nodiscard]] const route_type &route(
                const std::vector<candidate_type> &candidates,
                std::size_t from, std::size_t source, std::size_t to, std::size_t target,
                double max_distance_factor = 10.0) {
            const auto from_target = boost::target(
                    candidates[from].edges[source].edge_descriptor, network.graph());
            const auto to_source = boost::source(
                    candidates[to].edges[target].edge_descriptor, network.graph());

            route_cache_key_type key{from_target, to_source};

            if (not _route_cache.get()) {
                _route_cache.reset(new route_cache_type{});
            }

            auto search = _route_cache->find(key);
            bool exists = search != _route_cache->end();
            const route_type *return_route;

            if (exists) {
                return_route = &(*(search->second));
            } else {
                bool found = false;

                const auto shortest_paths = this->shortest_paths(candidates, from, source, to, max_distance_factor);
                for (const auto &shortest_path: shortest_paths) {
                    route_cache_key_type new_key{shortest_path.front(), shortest_path.back()};

                    auto new_search = _route_cache->find(new_key);
                    exists = new_search != _route_cache->end();

                    if (exists) {
                        if (key == new_key) {
                            return_route = &(*(new_search->second));
                            found = true;
                        }
                    } else {
                        auto new_route = network.extract_route(shortest_path);

                        auto inserted = _route_cache->emplace(new_key,
                                                              absl::make_unique<const route_type>(
                                                                      std::move(new_route)));

                        if (key == new_key) {
                            return_route = &(*(inserted.first->second));
                            found = true;
                        }
                    }
                }

                if (not found) {
                    route_type no_route{true};
                    auto inserted = _route_cache->emplace(key,
                                                          absl::make_unique<const route_type>(std::move(no_route)));
                    return_route = &(*(inserted.first->second));
                }
            }

            return std::cref(*return_route);
        }

        void clear_route_cache() {
            _route_cache.reset(new route_cache_type{});
        }

        [[nodiscard]] route_type candidate_route(
                const std::vector<candidate_type> &candidates,
                std::size_t from, std::size_t source, std::size_t to, std::size_t target,
                bool within_edge_turns = false, double max_distance_factor = 10.0) {
            const auto &candidate_from = candidates[from].edges[source];
            const auto &candidate_to = candidates[to].edges[target];

            const auto &start = candidate_from.to;
            const auto &end = candidate_to.from;

            if (geometry::equals_points(candidate_from.projection_point, candidate_to.projection_point)) {
                // when both measurements point to same candidate, no route is needed as we are already there
                return route_type{false};
            }

            if (candidate_from.edge_descriptor == candidate_to.edge_descriptor) {
                // when both measurements point to the same edge
                if (start.has_length() and end.has_length()) {
                    // when both partial routes exist on the same edge
                    const auto &edge = network.edge(candidate_from.edge_descriptor);
                    if (edge.has_length() and start.length() + end.length() >= edge.length()) {
                        // if both lines overlap (which is only possible when sum of lengths is larger or equal than edge length)
                        if (candidate_from.from.has_length() and candidate_to.from.has_length() and
                            candidate_from.from.length() <= candidate_to.from.length()) {
                            // only continue if substring is possible, else routing is needed
//                            std::cout << edge << "\n" << candidate_from << "\n" << candidate_to << std::endl;
                            return route_type{geometry::substring<typename edge_type::rich_line_type>(
                                    edge, candidate_from.from.length(), candidate_to.from.length(),
                                    candidate_from.projection_point, candidate_to.projection_point, true, true)};
                        }
                    }
                }
            }

            route_type route = this->route(candidates, from, source, to, target, max_distance_factor);

            if (not route.is_invalid()) {
                route_type start_route{std::cref(start)};
                route_type end_route{std::cref(end)};

                try {
                    if (within_edge_turns) {
                        // trim route to allow turns within the edge
                        route_type start_return = start_route.trim_merge(route);
                        if (not start_return.is_invalid()) {
                            // start trim worked
                            route_type end_return = start_return.trim_merge(end_route);
                            if (not end_return.is_invalid()) {
                                // end trim also worked, return combination
                                return std::move(end_return);
                            } else {
                                // end trim did not work, merge
                                return route_type::merge({start_return, end_route}, false, true);
                            }
                        } else {
                            // start trim did not work, try end
                            route_type end_return = route.trim_merge(end_route);
                            if (not end_return.is_invalid()) {
                                // end trim worked, start did not, merge
                                return route_type::merge({start_route, end_return}, false, true);
                            } else {
                                // end trim also did not work, merge all
                                return route_type::merge({start_route, route, end_route}, false, true);
                            }
                        }
                    } else {
                        // only turn at graph nodes, never within edges, so regular merge
                        return route_type::merge({start_route, route, end_route}, false, true);
                    }
                } catch (geometry::network::route_merge_exception &exception) {
                    // could not merge route, return invalid route
                    return route_type{true};
                } catch (geometry::rich_line_merge_exception &exception) {
                    // could not merge rich lines in route, return invalid route
                    return route_type{true};
                }
            }

            // did not find a route, return invalid route
            return route_type{true};
        }

        [[nodiscard]] route_type candidates_route(
                const std::vector<candidate_type> &candidates,
                const std::vector<std::pair<std::size_t, std::size_t>> &policy,
                bool within_edge_turns = false, bool export_edges = false, bool join_merges = true,
                double max_distance_factor = 10.0) {
            std::vector<route_type> routes;
            routes.reserve(not candidates.empty() ? candidates.size() - 1 : 0);
            bool skipped = false;
            for (std::size_t i = 0, j = 1; j < policy.size(); ++i, ++j) {
                const auto &from_policy_pair = policy[i];
                const auto &to_policy_pair = policy[j];

                const auto from = from_policy_pair.first;
                const auto to = to_policy_pair.first;

                if (from == to) {
                    // skip detected
                    ++i;
                    ++j;
                    skipped = true;
                    continue;
                }

                const auto source = from_policy_pair.second;
                const auto target = to_policy_pair.second;

                route_type route = candidate_route(candidates, from, source, to, target,
                                                   within_edge_turns, max_distance_factor);

                if (join_merges and not skipped and
                    not routes.empty() and not route.is_invalid() and not route.rich_lines().empty()) {
                    route_type &prev_route = routes.back();
                    if (not prev_route.is_invalid() and not prev_route.rich_lines().empty()) {
                        const auto &prev_route_back = prev_route.rich_lines().back().get();
                        if (not prev_route_back.empty()) {
                            const point_type &prev_route_last_point = prev_route_back.back();
                            const auto point_search = network.point_in_edges(prev_route_last_point);
                            if (not point_search.first) {
                                route_type::join(prev_route, route, true);
                            }
                        }
                    }
                }

                if (not route.is_invalid() and route.has_length()) {
                    skipped = false;
                    routes.emplace_back(std::move(route));
                }
            }

//        std::cout << "\nRoutes:" << std::endl;
//        for (const auto &route : routes) {
//            std::cout << route.wkt() << std::endl;
//        }

            route_type route = route_type::merge({routes.begin(), routes.end()});

            if (not export_edges) {
                return route;
            } else {
                using rich_line_type = typename Network::rich_line_type;
                using multi_rich_line_type = geometry::eager_multi_rich_line<rich_line_type>;

                std::vector<std::reference_wrapper<const typename Network::rich_line_type>> edges;
                edges.reserve(not candidates.empty() ? candidates.size() - 1 : 0);

                const auto multi_rich_line = route.template get_multi_rich_line<multi_rich_line_type>();
                for (const auto &rich_line: multi_rich_line.rich_lines()) {
                    for (const auto &rich_segment: rich_line.rich_segments()) {
                        if (rich_segment.has_length() and rich_segment.has_azimuth()) {
                            const auto edge_search = network.fitting_edge(rich_segment.segment());
                            if (edge_search.first) {
                                const auto &edge = network.edge(edge_search.second);
                                if (edges.empty() or &edges.back().get() != &edge) {
                                    edges.emplace_back(std::cref(edge));
                                }
                            }
                        }
                    }
                }

//            std::cout << "\nEdges:" << std::endl;
//            for (const rich_line_type &edge : edges) {
//                std::cout << edge.wkt() << std::endl;
//            }

                return route_type{std::move(edges)};
            }
        }

        [[nodiscard]] std::vector<candidate_type> candidates_policy(
                const std::vector<candidate_type> &candidates,
                const std::vector<std::pair<std::size_t, std::size_t>> &policy) {
            std::vector<candidate_type> candidates_policy;
            candidates_policy.reserve(candidates.size());
            for (const auto &policy_pair: policy) {
                const auto candidate_index = policy_pair.first;
                const auto edge_index = policy_pair.second;

                const auto &candidate = candidates[candidate_index];
                if (not candidate.edges.empty()) {
                    candidates_policy.emplace_back(candidate_type{
                            *candidate.track, candidate.index, candidate.next_equal,
                            {}, {candidate.edges[edge_index]}});
                }
            }

            return candidates_policy;
        }

        void save_candidates(std::string filename, const std::vector<candidate_type> &candidates) const {
            io::csv_exporter<',', '"'> csv{std::move(filename)};
            bool first = true;
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                const auto &candidate = candidates[i];
                if (first) {
                    auto candidate_header = candidate.header();
                    std::vector<std::string> header;
                    header.reserve(1 + candidate_header.size());
                    header.emplace_back("id");
                    std::move(candidate_header.begin(), candidate_header.end(), std::back_inserter(header));

                    csv << header;
                    first = false;
                }
                auto candidate_rows = candidate.rows();
                for (auto &candidate_row: candidate_rows) {
                    std::vector<std::string> row;
                    row.reserve(1 + candidate_row.size());
                    row.emplace_back(std::to_string(i));
                    std::move(candidate_row.begin(), candidate_row.end(), std::back_inserter(row));

                    csv << row;
                }
            }
        }

        [[nodiscard]] std::vector<absl::btree_set < defect>> detect(
        const track_type &track,
        bool detect_duplicates = true,
        bool detect_forward_backward = true
        ) const {
            return _detector.detect(track, detect_duplicates, detect_forward_backward);
        }

        [[nodiscard]] absl::btree_set <defect> detect(
                const track_type &track, std::size_t from, std::size_t to, bool detect_forward_backward = true) const {
            return _detector.detect(track, from, to, detect_forward_backward);
        }

        [[nodiscard]] track_type
        remove_defects(const track_type &track, const std::vector<absl::btree_set < defect>>

        &defects) const {
            absl::btree_set <std::size_t> indices_to_remove;
            for (std::size_t i = 0; i < defects.size(); ++i) {
                const absl::btree_set <defect> &defect_set = defects[i];
                if (not defect_set.contains(defect::none)) {
                    indices_to_remove.emplace(i);
                }
            }

            return track.thin_out(indices_to_remove);
        }

        template<typename Learner>
        void match(const multi_track_type &multi_track,
                   const learning::settings &learn_settings = learning::settings{},
                   const matching::settings &match_settings = matching::settings{},
                   const std::function<void(const std::vector<typename Learner::environment_type> &,
                                            const std::vector<Learner> &,
                                            const std::vector<std::vector<std::pair<std::size_t, std::size_t>>> &,
                                            const multi_track_type &, const multi_track_type &, const route_type &,
                                            const double)>
                   &callback = [](const auto &environments, const auto &learners, const auto &policies,
                                  const auto &track, const auto &prepared, const auto &route, const auto duration) {}) {
            using environment_type = typename Learner::environment_type;
            std::vector<environment_type> environments;
            environments.reserve(multi_track.rich_lines().size());
            std::vector<Learner> learners;
            learners.reserve(multi_track.rich_lines().size());

            std::vector<std::vector<std::pair<std::size_t, std::size_t>>> policies;
            policies.reserve(multi_track.rich_lines().size());

            std::vector<track_type> tracks;
            tracks.reserve(multi_track.rich_lines().size());
            for (const auto &rich_line: multi_track.rich_lines()) {
                tracks.emplace_back(track_type{multi_track.id, rich_line});
            }

            std::vector<track_type> prepared;
            prepared.reserve(multi_track.rich_lines().size());
            std::vector<route_type> results;
            results.reserve(multi_track.rich_lines().size());

            double duration = 0.0;
            for (const auto &track: tracks) {
                environments.emplace_back(environment_type{*this, match_settings});
                auto &environment = environments.back();

                learners.emplace_back(Learner{environments.back(), learn_settings});
                auto &learner = learners.back();

                std::vector<std::pair<std::size_t, std::size_t>> policy;

                duration += util::benchmark([&]() {
                    environment.set(track);
                    policy = learner();
                });

                prepared.emplace_back(environment.track());
                results.emplace_back(environment.template result<route_type>(policy));
                policies.emplace_back(std::move(policy));
            }

            callback(environments, learners, policies, multi_track,
                     multi_track_type{multi_track.id, std::move(prepared)},
                     route_type::merge({results.begin(), results.end()}), duration);

            clear_route_cache();
            geometry::clear_distance_cache();
            geometry::clear_azimuth_cache();
        }

        template<typename Learner>
        void match(const track_type &track,
                   const learning::settings &learn_settings = learning::settings{},
                   const matching::settings &match_settings = matching::settings{},
                   const std::function<void(const std::vector<typename Learner::environment_type> &,
                                            const std::vector<Learner> &,
                                            const std::vector<std::vector<std::pair<std::size_t, std::size_t>>> &,
                                            const multi_track_type &, const multi_track_type &, const route_type &,
                                            const double)>
                   &callback = [](const auto &environments, const auto &learners, const auto &policies,
                                  const auto &track, const auto &prepared, const auto &route, const auto duration) {}) {
            multi_track_type multi_track{track.id, track};
            this->template match<Learner>(multi_track, learn_settings, match_settings, callback);
        }

        template<typename Learner>
        void match_all(const absl::flat_hash_map <std::string, multi_track_type> &tracks,
                       const learning::settings &learn_settings = learning::settings{},
                       const matching::settings &match_settings = matching::settings{},
                       const std::function<void(const std::vector<typename Learner::environment_type> &,
                                                const std::vector<Learner> &,
                                                const std::vector<std::vector<std::pair<std::size_t, std::size_t>>> &,
                                                const multi_track_type &, const multi_track_type &, const route_type &,
                                                const double)>
                       &callback = [](const auto &environments, const auto &learners, const auto &policies,
                                      const auto &track, const auto &prepared, const auto &route,
                                      const auto duration) {}) {
            if (_single_threaded) {
                for (const auto &track: tracks) {
                    this->template match<Learner>(track.second, learn_settings, match_settings, callback);
                }
            } else {
                std::vector<multi_track_type> sorted_tracks;
                sorted_tracks.reserve(tracks.size());
                for (const auto &track: tracks) {
                    sorted_tracks.emplace_back(track.second);
                }

                multi_track_type::sort_tracks(sorted_tracks, false);

                std::list<boost::future<void>> futures;
                for (const auto &track: sorted_tracks) {
                    futures.emplace_back(boost::async(_matching_thread_pool, [&]() {
                        this->template match<Learner>(track, learn_settings, match_settings, callback);
                    }));
                }

                while (not futures.empty()) {
                    futures.front().get();
                    futures.pop_front();
                }
            }
        }

    private:
        using route_cache_key_type = std::pair<vertex_descriptor, vertex_descriptor>;

        detector<track_type> _detector;

        bool _single_threaded;
        boost::basic_thread_pool _matching_thread_pool;

        boost::thread_specific_ptr<std::vector<vertex_descriptor>> _vertices;
        boost::thread_specific_ptr<std::vector<vertex_descriptor>> _predecessors;
        boost::thread_specific_ptr<std::vector<length_type>> _distances;
        boost::thread_specific_ptr<std::vector<typename boost::default_color_type>> _colors;
        boost::thread_specific_ptr<std::vector<std::size_t>> _heap_index;
        boost::thread_specific_ptr<std::vector<vertex_descriptor>> _queue;

        using route_cache_type = absl::flat_hash_map<route_cache_key_type, std::unique_ptr<const route_type>>;

        boost::thread_specific_ptr<route_cache_type> _route_cache;

        [[nodiscard]] double _candidate_search_buffer_measurement(
                std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type> &candidate_edge_set,
                std::size_t round, const track_type &track, std::size_t index, double buffer_radius,
                double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0,
                bool adaptive_radius = true) const {
            assert(index < track.size());
            const point_type &point = track.at(index);

            bool found = false;
            absl::btree_set <edge_descriptor> edge_result;

            double current_buffer_radius = buffer_radius;
            while (not found) {
                if (adaptive_radius) {
                    if (index > 0) {
                        current_buffer_radius =
                                std::min(current_buffer_radius, (double) track.rich_segments()[index - 1].length());
                    }
                    if (index < track.size() - 1) {
                        current_buffer_radius =
                                std::min(current_buffer_radius, (double) track.rich_segments()[index].length());
                    }

                    if (current_buffer_radius < buffer_lower_radius) {
                        // needs to be at least minimum radius
                        current_buffer_radius = buffer_lower_radius;
                    }
                }

                current_buffer_radius = std::min(current_buffer_radius, buffer_upper_radius);

                // create bounding box with at least buffer_distance
                const auto buffer = geometry::buffer_box(point, current_buffer_radius);

                // query rtree
                edge_result = network.query_segments_unique(boost::geometry::index::intersects(buffer));

                // remove results outside of current radius
                for (auto edge_it = edge_result.begin(); edge_it != edge_result.end();) {
                    const auto &edge = network.edge(*edge_it);
                    const auto edge_line = edge.line();
                    const auto distance = geometry::distance(point, edge_line);
                    if (distance > current_buffer_radius) {
                        edge_it = edge_result.erase(edge_it);
                    } else {
                        edge_it++;
                    }
                }

                if (edge_result.empty() and current_buffer_radius != buffer_upper_radius) {
                    current_buffer_radius *= 2;
                    adaptive_radius = false;
                } else {
                    found = true;
                }
            }

            _process_candidate_query(candidate_edge_set, round, track, index, edge_result);

            return current_buffer_radius;
        }

        [[nodiscard]] std::size_t _candidate_search_nearest_measurement(
                std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type> &candidate_edge_set,
                std::size_t round, const track_type &track, std::size_t index, std::size_t number,
                bool with_reverse = false, bool with_adjacent = false) const {
            assert(index < track.size());
            const point_type &point = track.at(index);

            // query rtree
            absl::btree_set <edge_descriptor> edge_result = network.query_segments_unique(
                    boost::geometry::index::nearest(point, number));
            std::list<edge_descriptor> additions;

            if (with_adjacent) {
                for (const auto &edge_descriptor: edge_result) {
                    const auto &edge = network.edge(edge_descriptor);
                    const auto edge_line = edge.line();
                    const auto edge_distance = geometry::distance(point, edge_line);

                    const auto source = boost::source(edge_descriptor, network.graph());
                    const auto target = boost::target(edge_descriptor, network.graph());

                    for (const auto vertex: std::array{source, target}) {
                        for (const auto out_edge: boost::make_iterator_range(
                                boost::out_edges(vertex, network.graph()))) {
                            const auto &adj_edge = network.edge(out_edge);
                            const auto adj_edge_line = edge.line();
                            const auto adj_distance = geometry::distance(point, adj_edge_line);
                            if (adj_distance <= edge_distance) {
                                additions.emplace_back(out_edge);
                            }
                        }
                    }
                }

                while (not additions.empty()) {
                    edge_result.emplace(std::move(additions.front()));
                    additions.pop_front();
                }
            }

            if (with_reverse) {
                for (const auto edge_descriptor: edge_result) {
                    const auto source = boost::source(edge_descriptor, network.graph());
                    const auto target = boost::target(edge_descriptor, network.graph());
                    for (const auto out_edge: boost::make_iterator_range(boost::out_edges(target, network.graph()))) {
                        if (boost::target(out_edge, network.graph()) == source) {
                            additions.emplace_back(out_edge);
                        }
                    }
                }

                while (not additions.empty()) {
                    edge_result.emplace(std::move(additions.front()));
                    additions.pop_front();
                }
            }

            _process_candidate_query(candidate_edge_set, round, track, index, edge_result);

            return number;
        }

        [[nodiscard]] std::pair<double, std::size_t> _candidate_search_combined_measurement(
                std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type> &candidate_edge_set,
                std::size_t round, const track_type &track, std::size_t index, double buffer_radius, std::size_t number,
                double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0, bool adaptive_radius = true,
                bool with_reverse = false, bool with_adjacent = false) const {
            assert(index < track.size());
            const point_type &point = track.at(index);

            // query rtree
            absl::btree_set <edge_descriptor> edge_result = network.query_segments_unique(
                    boost::geometry::index::nearest(point, number));
            std::list<edge_descriptor> additions;

            std::vector<edge_descriptor> removals;
            removals.reserve(edge_result.size());

            bool found = false;
            double current_buffer_radius = buffer_radius;
            while (not found) {
                if (adaptive_radius) {
                    if (index > 0) {
                        current_buffer_radius =
                                std::min(current_buffer_radius, (double) track.rich_segments()[index - 1].length());
                    }
                    if (index < track.size() - 1) {
                        current_buffer_radius =
                                std::min(current_buffer_radius, (double) track.rich_segments()[index].length());
                    }

                    if (current_buffer_radius < buffer_lower_radius) {
                        // needs to be at least minimum radius
                        current_buffer_radius = buffer_lower_radius;
                    }
                }

                current_buffer_radius = std::min(current_buffer_radius, buffer_upper_radius);

                for (auto it = edge_result.cbegin(); it != edge_result.cend(); it++) {
                    const auto &edge = network.edge(*it);
                    const auto edge_line = edge.line();
                    const auto edge_distance = geometry::distance(point, edge_line);

                    if (edge_distance > current_buffer_radius) {
                        removals.emplace_back(*it);
                    }
                }

                if (removals.size() >= edge_result.size() and current_buffer_radius != buffer_upper_radius) {
                    current_buffer_radius *= 2;
                    adaptive_radius = false;
                    removals.clear();
                } else {
                    for (const auto &remove: removals) {
                        edge_result.erase(remove);
                    }
                    found = true;
                }
            }

            if (with_adjacent) {
                for (const auto &edge_descriptor: edge_result) {
                    const auto &edge = network.edge(edge_descriptor);
                    const auto edge_line = edge.line();
                    const auto edge_distance = geometry::distance(point, edge_line);

                    const auto source = boost::source(edge_descriptor, network.graph());
                    const auto target = boost::target(edge_descriptor, network.graph());

                    for (const auto vertex: std::array{source, target}) {
                        for (const auto out_edge: boost::make_iterator_range(
                                boost::out_edges(vertex, network.graph()))) {
                            const auto &adj_edge = network.edge(out_edge);
                            const auto adj_edge_line = edge.line();
                            const auto adj_distance = geometry::distance(point, adj_edge_line);
                            if (adj_distance <= edge_distance) {
                                additions.emplace_back(out_edge);
                            }
                        }
                    }
                }

                while (not additions.empty()) {
                    edge_result.emplace(std::move(additions.front()));
                    additions.pop_front();
                }
            }

            if (with_reverse) {
                for (const auto edge_descriptor: edge_result) {
                    const auto source = boost::source(edge_descriptor, network.graph());
                    const auto target = boost::target(edge_descriptor, network.graph());
                    for (const auto out_edge: boost::make_iterator_range(boost::out_edges(target, network.graph()))) {
                        if (boost::target(out_edge, network.graph()) == source) {
                            additions.emplace_back(out_edge);
                        }
                    }
                }

                while (not additions.empty()) {
                    edge_result.emplace(std::move(additions.front()));
                    additions.pop_front();
                }
            }

            _process_candidate_query(candidate_edge_set, round, track, index, edge_result);

            return std::pair{current_buffer_radius, number};
        }

        void _process_candidate_query(
                std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type> &candidate_edge_set,
                std::size_t round, const track_type &track, std::size_t index,
                absl::btree_set <edge_descriptor> &edge_result) const {
            assert(index < track.size());
            const point_type &point = track.at(index);

            // remove edges from result that were already processes in candidate_edge_set
            for (const auto &candidate_edge: candidate_edge_set) {
                edge_result.erase(candidate_edge.edge_descriptor);
            }

            using segment_type = typename geometry::models<point_type>::segment_type;

            for (const auto &edge_descriptor: edge_result) {
                const auto &edge = network.edge(edge_descriptor);
                if (edge.has_length()) {
                    segment_type projection;
                    const auto edge_line = edge.line();
                    boost::geometry::closest_points(point, edge_line, projection);
                    auto projection_distance = geometry::point_distance(projection.first, projection.second);
                    if (projection_distance == geometry::default_float_type<distance_type>::v0) {
                        // when distance is zero, replace closest point with point point as it is already fine
                        projection.second = projection.first;
                    }

                    // check if point is near a segment point first, if so, replace with it
                    const auto nearest_segments = network.query_segments(
                            boost::geometry::index::nearest(projection.second, 1));
                    const point_type *segment_point = nullptr;
                    distance_type segment_point_distance = geometry::default_float_type<distance_type>::v1;
                    if (not nearest_segments.empty()) {
                        const auto &nearest_segment = nearest_segments.front().first;
                        segment_point = &nearest_segment.first;
                        segment_point_distance = geometry::point_distance(
                                nearest_segment.first, projection.second);
                        auto tmp_distance = geometry::point_distance(
                                nearest_segment.second, projection.second);
                        if (tmp_distance < segment_point_distance) {
                            segment_point_distance = tmp_distance;
                            segment_point = &nearest_segment.second;
                        }
                    }
                    if (segment_point != nullptr and not geometry::equals_points(*segment_point, projection.second) and
                        segment_point_distance < 1e-4) {
                        // if within one millimeter, replace point with found point, so "snap to network"
                        projection.second = *segment_point;
                        projection_distance = geometry::point_distance(projection.first, projection.second);
                    }

                    using rich_line_type = geometry::lazy_rich_line_type<point_type>;
                    rich_line_type from_line, to_line;
                    if (geometry::equals_points(edge.front(), projection.second)) {
                        // projection is equal to front, from is empty, to is edge
                        to_line = edge;
                    } else if (geometry::equals_points(edge.back(), projection.second)) {
                        // projection is equal to back, from is edge, to is empty
                        from_line = edge;
                    } else {
                        // projection is not equal, check distances
                        const auto from_distance = geometry::point_distance(edge.front(), projection.second);
                        if (from_distance == geometry::default_float_type<distance_type>::v0) {
                            // from distance is zero, from is empty, to is edge
                            to_line = edge;
                            projection.second = edge.front();
                            projection_distance = geometry::point_distance(projection.first, projection.second);
                        } else {
                            const auto to_distance = geometry::point_distance(edge.back(), projection.second);
                            if (to_distance == geometry::default_float_type<distance_type>::v0) {
                                // to distance is zero, from is edge, to is empty
                                from_line = edge;
                                projection.second = edge.back();
                                projection_distance = geometry::point_distance(projection.first, projection.second);
                            } else {
                                // we need to cut substrings
                                from_line = geometry::substring_point_to<rich_line_type>(edge, projection.second, true);
                                assert(from_line.has_length() and
                                       from_line.length() > geometry::default_float_type<length_type>::v0);
                                if (from_line.length() == edge.length()) {
                                    // from line has same length as edge, replace with edge again, to_line is empty
                                    from_line = edge;
                                    projection.second = edge.back();
                                    projection_distance = geometry::point_distance(projection.first, projection.second);
                                } else {
                                    to_line = geometry::substring_point_from<rich_line_type>(edge, projection.second,
                                                                                             true);
                                    assert(to_line.has_length() and
                                           to_line.length() > geometry::default_float_type<length_type>::v0);
                                    if (to_line.length() == edge.length()) {
                                        // to line has same length as edge, replace with edge again, replace from to empty
                                        from_line = rich_line_type{};
                                        to_line = edge;
                                        projection.second = edge.front();
                                        projection_distance = geometry::point_distance(
                                                projection.first, projection.second);
                                    }
                                }
                            }
                        }
                    }

                    candidate_edge_set.emplace(candidate_edge_type{
                            false, round, edge_descriptor, projection.second, projection_distance, std::move(from_line),
                            std::move(to_line)});
                }
            }
        }

        void _finalize_candidates(
                std::vector<candidate_type> &candidates, std::size_t round, const track_type &track,
                std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> &candidate_edge_sets,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) const {
            const auto add_candidate = [round](const auto &point, const auto &candidate_edge, auto &edge_set) {
                const auto distance = geometry::point_distance(point, candidate_edge.projection_point);

                candidate_edge_type new_candidate_edge{
                        true, round, candidate_edge.edge_descriptor, candidate_edge.projection_point,
                        distance, candidate_edge.from, candidate_edge.to};

                const auto range = edge_set.equal_range(new_candidate_edge);

                if (range.first == edge_set.cend()) {
                    edge_set.emplace(std::move(new_candidate_edge));
                } else if (range.first != edge_set.cend()) {
                    bool emplace = true;
                    for (auto it = range.first; it != range.second; it++) {
                        if (*it == new_candidate_edge) {
                            emplace = false;
                            break;
                        }
                    }

                    if (emplace) {
                        edge_set.emplace(std::move(new_candidate_edge));
                    }
                }
            };

            if (candidate_adoption_nearby) {
                using segment_type = typename geometry::models<point_type>::segment_type;

                using index_value_type = std::pair<segment_type,
                        std::pair<std::size_t, std::reference_wrapper<const candidate_edge_type>>>;

                // create candidates index
                std::size_t candidates_number = 0;
                for (const auto &edge_set: candidate_edge_sets) {
                    for (const auto &candidate: edge_set) {
                        if (not candidate.adopted) {
                            candidates_number++;
                        }
                    }
                }

                std::vector<index_value_type> candidates_index_values;
                candidates_index_values.reserve(candidates_number);
                for (std::size_t index = 0; index < candidate_edge_sets.size(); ++index) {
                    const point_type &point = track.at(index);
                    auto &edge_set = candidate_edge_sets[index];
                    for (const auto &candidate: edge_set) {
                        if (not candidate.adopted) {
                            candidates_index_values.emplace_back(
                                    std::pair{segment_type{point, candidate.projection_point},
                                              std::pair{index, std::cref(candidate)}});
                        }
                    }
                }

                boost::geometry::index::rtree<index_value_type, boost::geometry::index::rstar<16>>
                        candidates_index{std::move(candidates_index_values)};

                // search in index for adopting nearby candidates not directly attached
                for (std::size_t index = 0; index < candidate_edge_sets.size(); ++index) {
                    const point_type &point = track.at(index);
                    auto &edge_set = candidate_edge_sets[index];
                    auto edge_set_it = edge_set.crbegin();
                    if (edge_set_it != edge_set.crend()) {
                        const auto longest_distance = edge_set_it->distance;
                        if (longest_distance > 0.0) {
                            const auto buffer = geometry::buffer_box(point, longest_distance);

                            std::vector<index_value_type> results;
                            candidates_index.query(boost::geometry::index::intersects(buffer),
                                                   std::back_inserter(results));

                            for (const auto &index_value: results) {
                                if (geometry::distance(point, index_value.first) <= longest_distance) {
                                    const auto &candidate_edge_pair = index_value.second;
                                    std::size_t result_index = candidate_edge_pair.first;
                                    if (index != result_index) {
                                        // skip self references
                                        const candidate_edge_type &candidate_edge = candidate_edge_pair.second;
                                        add_candidate(point, candidate_edge, edge_set);

                                        if (candidate_adoption_reverse) {
                                            auto &reverse_candidate_edge_set = candidate_edge_sets[result_index];
                                            for (const auto &edge: edge_set) {
                                                if (not edge.adopted) {
                                                    const point_type &reverse_point = track.at(result_index);
                                                    add_candidate(reverse_point, edge, reverse_candidate_edge_set);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (candidate_adoption_siblings) {
                // adopt from previous and next candidate set the directly attached candidates
                for (std::size_t index = 0; index < candidate_edge_sets.size(); ++index) {
                    const point_type &point = track.at(index);
                    auto &edge_set = candidate_edge_sets[index];

                    // we have a previous candidate set
                    if (index >= 1) {
                        for (const auto &candidate_edge: candidate_edge_sets[index - 1]) {
                            if (not candidate_edge.adopted) {
                                add_candidate(point, candidate_edge, edge_set);
                            }
                        }
                    }

                    // we have a next candidate set
                    if (index + 1 < candidate_edge_sets.size()) {
                        for (const auto &candidate_edge: candidate_edge_sets[index + 1]) {
                            if (not candidate_edge.adopted) {
                                add_candidate(point, candidate_edge, edge_set);
                            }
                        }
                    }
                }
            }

            // preparing node set and finalizing candidates
            for (std::size_t index = 0; index < candidate_edge_sets.size(); ++index) {
                const point_type &point = track.at(index);
                auto &edge_set = candidate_edge_sets[index];

                // node set
                std::multiset<candidate_node_type, candidate_node_distance_comparator_type> node_set{
                        &candidate_node_type::distance_comparator};
                absl::btree_set<typename boost::graph_traits<graph_type>::vertex_descriptor> node_check;
                for (const auto &candidate_edge: edge_set) {
                    const auto edge_descriptor = candidate_edge.edge_descriptor;

                    const auto source = boost::source(edge_descriptor, network.graph());
                    if (not node_check.contains(source)) {
                        const auto &node = network.vertex(source);
                        const auto distance = geometry::point_distance(point, node.point);
                        node_set.emplace(candidate_node_type{source, distance});
                        node_check.emplace(source);
                    }

                    const auto target = boost::target(edge_descriptor, network.graph());
                    if (not node_check.contains(target)) {
                        const auto &node = network.vertex(target);
                        const auto distance = geometry::point_distance(point, node.point);
                        node_set.emplace(candidate_node_type{target, distance});
                        node_check.emplace(target);
                    }
                }

                std::vector<candidate_node_type> nodes;
                nodes.reserve(node_set.size());
                std::move(node_set.begin(), node_set.end(), std::back_inserter(nodes));

                std::vector<candidate_edge_type> edges;
                edges.reserve(edge_set.size());
                std::move(edge_set.begin(), edge_set.end(), std::back_inserter(edges));

                if (index >= candidates.size()) {
                    // next point is equal to current point
                    bool next_equal = false;
                    if (index + 1 < track.size()) {
                        next_equal = geometry::equals_points(point, track.at(index + 1));
                    }

                    candidates.emplace_back(
                            candidate_type{track, index, next_equal, std::move(nodes), std::move(edges)});
                } else {
                    auto &candidate = candidates[index];
                    candidate.nodes = std::move(nodes);
                    candidate.edges = std::move(edges);
                }
            }
        }

        void _finalize_candidates_buffer(
                std::vector<candidate_type> &candidates, std::size_t round, const track_type &track,
                const std::vector<double> &radii,
                std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> &candidate_edge_sets,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) const {
            _finalize_candidates(
                    candidates, round, track, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby,
                    candidate_adoption_reverse);
            assert(candidates.size() == radii.size());
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                candidates[i].radius = radii[i];
            }
        }

        void _finalize_candidates_nearest(
                std::vector<candidate_type> &candidates, std::size_t round, const track_type &track,
                const std::vector<std::size_t> &numbers,
                std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> &candidate_edge_sets,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) const {
            _finalize_candidates(
                    candidates, round, track, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby,
                    candidate_adoption_reverse);
            assert(candidates.size() == numbers.size());
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                candidates[i].number = numbers[i];
            }
        }

        void _finalize_candidates_combined(
                std::vector<candidate_type> &candidates, std::size_t round, const track_type &track,
                const std::vector<std::pair<double, std::size_t>> &radii_numbers,
                std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> &candidate_edge_sets,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) const {
            _finalize_candidates(
                    candidates, round, track, candidate_edge_sets, candidate_adoption_siblings,
                    candidate_adoption_nearby,
                    candidate_adoption_reverse);
            assert(candidates.size() == radii_numbers.size());
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                const auto &radius_number = radii_numbers[i];
                candidates[i].radius = radius_number.first;
                candidates[i].number = radius_number.second;
            }
        }

        void _prepare_shortest_path_memory() {
            if (not _vertices.get()) {
                _vertices.reset(new std::vector<vertex_descriptor>{});
                _vertices->reserve(boost::num_vertices(network.graph()));
            }
            if (not _predecessors.get()) {
                _predecessors.reset(new std::vector<vertex_descriptor>{});
                _predecessors->reserve(boost::num_vertices(network.graph()));
                for (const auto vertex: boost::make_iterator_range(boost::vertices(network.graph()))) {
                    _predecessors->emplace_back(vertex);
                }
            }
            if (not _distances.get()) {
                _distances.reset(new std::vector<length_type>{});
                _distances->reserve(boost::num_vertices(network.graph()));
                for (const auto vertex: boost::make_iterator_range(boost::vertices(network.graph()))) {
                    _distances->emplace_back(std::numeric_limits<length_type>::infinity());
                }
            }
            if (not _colors.get()) {
                _colors.reset(new std::vector<typename boost::default_color_type>{});
                _colors->reserve(boost::num_vertices(network.graph()));
                for (const auto vertex: boost::make_iterator_range(boost::vertices(network.graph()))) {
                    _colors->emplace_back(boost::white_color);
                }
            }
            if (not _heap_index.get()) {
                _heap_index.reset(new std::vector<std::size_t>{});
                _heap_index->reserve(boost::num_vertices(network.graph()));
                for (const auto vertex: boost::make_iterator_range(boost::vertices(network.graph()))) {
                    _heap_index->emplace_back(0);
                }
            }
            if (not _queue.get()) {
                _queue.reset(new std::vector<std::size_t>{});
                _queue->reserve(boost::num_vertices(network.graph()));
            }
        }

    };

}

#endif //MAP_MATCHING_2_MATCHER_HPP
