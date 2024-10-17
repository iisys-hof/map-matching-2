// Copyright (C) 2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MATCHING_ALGORITHM_ROUTE_HPP
#define MAP_MATCHING_2_MATCHING_ALGORITHM_ROUTE_HPP

#include <vector>
#include <list>

#include <boost/unordered/unordered_flat_set.hpp>

#include "matching/traits/network.hpp"

#include "graph/algorithm/dijkstra_shortest_path.hpp"

#include "geometry/algorithm/distance.hpp"
#include "geometry/algorithm/substring.hpp"

#include "util/time_helper.hpp"

namespace map_matching_2::matching {

    template<typename Network>
    class router {

    public:
        using network_type = Network;
        using vertex_descriptor = typename network_traits<network_type>::vertex_descriptor;
        using edge_descriptor = typename network_traits<network_type>::edge_descriptor;
        using edge_data_type = typename network_traits<network_type>::edge_data_type;
        using coordinate_system_type = typename network_traits<network_type>::coordinate_system_type;
        using point_type = typename network_traits<network_type>::point_type;
        using candidate_type = typename network_traits<network_type>::candidate_type;
        using distance_type = typename network_traits<network_type>::distance_type;
        using length_type = typename network_traits<network_type>::length_type;
        using route_type = typename network_traits<network_type>::route_type;
        using lazy_rich_line_type = typename network_traits<network_type>::lazy_rich_line_type;
        using lazy_multi_rich_line_type = typename network_traits<network_type>::lazy_multi_rich_line_type;
        using policy_type = typename network_traits<network_type>::policy_type;

        constexpr explicit router(const network_type &network, util::time_helper &time_helper)
            : _time_helper{time_helper}, _network{network} {}

        [[nodiscard]] std::vector<std::list<vertex_descriptor>> shortest_paths(
                const std::vector<candidate_type> &candidates,
                std::size_t from, std::size_t source, std::size_t to, double max_distance_factor = 10.0) const {
            const auto &candidate_edge_start = candidates[from].edges[source];
            const auto &vertex_start = _network.graph().target(candidate_edge_start.edge_desc);
            const auto &node_start = _network.vertex(vertex_start);

            distance_type max_distance = geometry::default_float_type<distance_type>::v0;
            boost::unordered::unordered_flat_set<vertex_descriptor> goals;
            for (const auto &candidate_node : candidates[to].nodes) {
                const auto &vertex_goal = candidate_node.vertex_desc;
                goals.emplace(vertex_goal);

                const auto &node_goal = _network.vertex(vertex_goal);
                const auto distance = geometry::point_distance(node_start.point, node_goal.point);
                if (distance > max_distance) {
                    max_distance = distance;
                }
            }

            return _network.dijkstra_shortest_paths(
                    vertex_start, goals, max_distance, max_distance_factor,
                    _predecessors, _distances, _colors, _queue);
        }

        [[nodiscard]] const route_type &route(
                const std::vector<candidate_type> &candidates,
                const std::size_t from, const std::size_t source, const std::size_t to, const std::size_t target,
                const double max_distance_factor = 10.0) const {
            const auto from_target = _network.graph().target(candidates[from].edges[source].edge_desc);
            const auto to_source = _network.graph().source(candidates[to].edges[target].edge_desc);

            route_cache_key_type key{from_target, to_source};

            auto search = _route_cache.find(key);
            bool exists = search != _route_cache.end();
            const route_type *return_route;

            if (exists) {
                return_route = &(*(search->second));
            } else {
                bool found = false;

                const auto shortest_paths = this->shortest_paths(candidates, from, source, to, max_distance_factor);
                for (const auto &shortest_path : shortest_paths) {
                    route_cache_key_type new_key{shortest_path.front(), shortest_path.back()};

                    auto new_search = _route_cache.find(new_key);
                    exists = new_search != _route_cache.end();

                    if (exists) {
                        if (key == new_key) {
                            return_route = &(*(new_search->second));
                            found = true;
                        }
                    } else {
                        auto new_route = _network.extract_route(shortest_path);

                        auto inserted = _route_cache.emplace(
                                new_key, std::make_unique<const route_type>(std::move(new_route)));

                        if (key == new_key) {
                            return_route = &(*(inserted.first->second));
                            found = true;
                        }
                    }
                }

                if (not found) {
                    route_type no_route{true};
                    auto inserted = _route_cache.emplace(
                            key, std::make_unique<const route_type>(std::move(no_route)));
                    return_route = &(*(inserted.first->second));
                }
            }

            return std::cref(*return_route);
        }

        [[nodiscard]] route_type candidate_route(
                const std::vector<candidate_type> &candidates,
                const std::size_t from, const std::size_t source, const std::size_t to, const std::size_t target,
                const bool within_edge_turns = false, const double max_distance_factor = 10.0) const {
            const auto &candidate_from = candidates[from].edges[source];
            const auto &candidate_to = candidates[to].edges[target];

            const auto &start = candidate_from.to;
            const auto &end = candidate_to.from;

            if (geometry::equals_points(candidate_from.projection_point, candidate_to.projection_point)) {
                // when both measurements point to same candidate, no route is needed as we are already there
                return route_type{false};
            }

            if (candidate_from.edge_desc == candidate_to.edge_desc) {
                // when both measurements point to the same edge
                if (start->has_length() and end->has_length()) {
                    // when both partial routes exist on the same edge
                    const auto &edge = _network.edge(candidate_from.edge_desc);
                    if (edge.rich_line.has_length() and start->length() + end->length() >= edge.rich_line.length()) {
                        // if both lines overlap (which is only possible when sum of lengths is larger or equal than edge length)
                        if (candidate_from.from->has_length() and candidate_to.from->has_length() and
                            candidate_from.from->length() <= candidate_to.from->length()) {
                            // only continue if substring is possible, else routing is needed
                            // std::cout << edge << "\n" << candidate_from << "\n" << candidate_to << std::endl;
                            return route_type{
                                    geometry::substring<lazy_rich_line_type>(
                                            edge.rich_line, candidate_from.from->length(), candidate_to.from->length(),
                                            candidate_from.projection_point, candidate_to.projection_point, true, true)
                            };
                        }
                    }
                }
            }

            const route_type &route = this->route(candidates, from, source, to, target, max_distance_factor);

            if (not route.is_invalid()) {
                route_type start_route{std::cref(*start)};
                route_type end_route{std::cref(*end)};

                try {
                    if (within_edge_turns) {
                        // trim route to allow turns within the edge
                        route_type start_return = start_route.template trim_merge<lazy_rich_line_type>(route);
                        if (not start_return.is_invalid()) {
                            // start trim worked
                            route_type end_return = start_return.template trim_merge<lazy_rich_line_type>(end_route);
                            if (not end_return.is_invalid()) {
                                // end trim also worked, return combination
                                return end_return;
                            } else {
                                // end trim did not work, merge
                                return route_type::template merge<lazy_rich_line_type>(
                                        {start_return, end_route}, false, true);
                            }
                        } else {
                            // start trim did not work, try end
                            route_type end_return = route.template trim_merge<lazy_rich_line_type>(end_route);
                            if (not end_return.is_invalid()) {
                                // end trim worked, start did not, merge
                                return route_type::template merge<lazy_rich_line_type>(
                                        {start_route, end_return}, false, true);
                            } else {
                                // end trim also did not work, merge all
                                return route_type::template merge<lazy_rich_line_type>(
                                        {start_route, route, end_route}, false, true);
                            }
                        }
                    } else {
                        // only turn at graph nodes, never within edges, so regular merge
                        return route_type::template merge<lazy_rich_line_type>(
                                {start_route, route, end_route}, false, true);
                    }
                } catch (geometry::network::route_merge_exception &) {
                    // could not merge route, return invalid route
                    return route_type{true};
                } catch (geometry::rich_line_merge_exception &) {
                    // could not merge rich lines in route, return invalid route
                    return route_type{true};
                }
            }

            // did not find a route, return invalid route
            return route_type{true};
        }

        [[nodiscard]] route_type candidates_route(
                const std::vector<candidate_type> &candidates, const policy_type &policy,
                const bool within_edge_turns = false, const bool join_merges = true,
                const double max_distance_factor = 10.0) const {
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
                    if (not prev_route.is_invalid() and not prev_route.empty()) {
                        const auto &prev_route_back_ref_var = prev_route.rich_lines().back();
                        std::visit([this, &prev_route, &route](auto &&prev_route_back_ref) {
                            const auto &prev_route_back = prev_route_back_ref.get();
                            if (not prev_route_back.empty()) {
                                const point_type &prev_route_last_point = prev_route_back.back();
                                const auto point_search = _network.point_in_edges(prev_route_last_point);
                                if (not point_search.first) {
                                    route_type::template join<lazy_rich_line_type>(prev_route, route, true);
                                }
                            }
                        }, prev_route_back_ref_var);
                    }
                }

                if (not route.is_invalid() and route.has_length()) {
                    skipped = false;
                    routes.emplace_back(std::move(route));
                }
            }

            return route_type::template merge<lazy_rich_line_type>({routes.begin(), routes.end()});
        }

        [[nodiscard]] std::vector<std::reference_wrapper<const edge_data_type>> edges_list(
                const route_type &route) const {
            std::vector<std::reference_wrapper<const edge_data_type>> edges;

            const auto multi_rich_line = route.template get_multi_rich_line<lazy_multi_rich_line_type>();
            for (const auto &rich_line : multi_rich_line.rich_lines()) {
                if (not rich_line.empty()) {
                    for (std::size_t i = 0; i < rich_line.size() - 1; ++i) {
                        const auto rich_segment = rich_line.rich_segment(i);
                        if (rich_segment.has_length() and rich_segment.has_azimuth()) {
                            const auto edge_search = _network.fitting_edge(rich_segment.segment());
                            if (edge_search.first) {
                                const auto &edge = _network.edge(edge_search.second);
                                if (edges.empty()) {
                                    edges.emplace_back(std::cref(edge));
                                } else {
                                    const auto &back_edge = edges.back().get();
                                    if (reinterpret_cast<const std::uintptr_t>(&back_edge.rich_line) !=
                                        reinterpret_cast<const std::uintptr_t>(&edge.rich_line)) {
                                        edges.emplace_back(std::cref(edge));
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return edges;
        }

        [[nodiscard]] static route_type export_edges_route(
                const std::vector<std::reference_wrapper<const edge_data_type>> &edges_list) {
            std::vector<typename route_type::rich_line_reference_variant_type> edges;

            for (const auto &edge_ref : edges_list) {
                const auto &edge = edge_ref.get();
                edges.emplace_back(std::cref(edge.rich_line));
            }

            return route_type{std::move(edges)};
        }

        [[nodiscard]] static std::vector<osmium::object_id_type> edge_ids(
                const std::vector<std::reference_wrapper<const edge_data_type>> &edges_list) {
            std::vector<osmium::object_id_type> ids;

            for (const auto &edge_ref : edges_list) {
                const auto &edge = edge_ref.get();
                if (ids.empty()) {
                    ids.emplace_back(edge.id);
                } else {
                    const auto &back_id = ids.back();
                    if (back_id != edge.id) {
                        ids.emplace_back(edge.id);
                    }
                }
            }

            return ids;
        }

        [[nodiscard]] static std::vector<candidate_type> candidates_policy(
                const std::vector<candidate_type> &candidates, const policy_type &policy) {
            std::vector<candidate_type> candidates_policy;
            candidates_policy.reserve(candidates.size());
            for (const auto &policy_pair : policy) {
                const auto candidate_index = policy_pair.first;
                const auto edge_index = policy_pair.second;

                const auto &candidate = candidates[candidate_index];
                if (not candidate.edges.empty()) {
                    candidates_policy.emplace_back(candidate_type{
                            *candidate.track, candidate.index, candidate.next_equal, {}, {candidate.edges[edge_index]}
                    });
                }
            }

            return candidates_policy;
        }

    private:
        using route_cache_key_type = std::pair<vertex_descriptor, vertex_descriptor>;
        using priority_type = graph::priority_type<length_type, vertex_descriptor>;
        using route_cache_type = boost::unordered::unordered_flat_map<
            route_cache_key_type, std::unique_ptr<const route_type>>;

        util::time_helper &_time_helper;
        const network_type &_network;

        mutable graph::predecessors_type _predecessors{};
        mutable graph::distances_type<length_type> _distances{};
        mutable graph::colors_type _colors{};
        mutable graph::priority_queue_type<priority_type> _queue{};

        mutable route_cache_type _route_cache{};

    };

}

#endif //MAP_MATCHING_2_MATCHING_ALGORITHM_ROUTE_HPP
