// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MATCHING_ALGORITHM_CANDIDATE_SEARCH_HPP
#define MAP_MATCHING_2_MATCHING_ALGORITHM_CANDIDATE_SEARCH_HPP

#include <limits>
#include <algorithm>
#include <vector>

#include <boost/container/set.hpp>

#include "types/geometry/index/matcher.hpp"
#include "types/geometry/rich_type/lazy_rich_line.hpp"

#include "matching/traits/network.hpp"

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/distance.hpp"
#include "geometry/algorithm/buffer.hpp"
#include "geometry/algorithm/substring.hpp"

#include "util/benchmark.hpp"
#include "util/time_helper.hpp"

#include "../settings.hpp"

namespace map_matching_2::matching {

    template<typename Network>
    class candidate_searcher {

    public:
        using network_type = Network;
        using vertex_descriptor = typename network_traits<network_type>::vertex_descriptor;
        using edge_descriptor = typename network_traits<network_type>::edge_descriptor;
        using coordinate_system_type = typename network_traits<network_type>::coordinate_system_type;
        using track_type = typename network_traits<network_type>::track_type;
        using point_type = typename network_traits<network_type>::point_type;
        using time_point_type = typename network_traits<network_type>::time_point_type;
        using candidate_type = typename network_traits<network_type>::candidate_type;
        using candidate_node_type = typename network_traits<network_type>::candidate_node_type;
        using candidate_edge_type = typename network_traits<network_type>::candidate_edge_type;
        using distance_type = typename network_traits<network_type>::distance_type;
        using length_type = typename network_traits<network_type>::length_type;

        constexpr explicit candidate_searcher(const network_type &network, util::time_helper &time_helper)
            : _time_helper{time_helper}, _network{network} {}

        [[nodiscard]] std::vector<candidate_type> candidate_search(
                const track_type &track, const matching::settings &match_settings) const {
            std::vector<boost::container::multiset<candidate_edge_type>> candidate_edge_sets(track.rich_line.size());
            for (std::size_t i = 0; i < track.rich_line.size(); ++i) {
                _candidate_search_individual(candidate_edge_sets[i], track, i, match_settings);

                if (_time_helper.update_and_has_reached()) {
                    return {};
                }
            }

            return _finalize_candidates(candidate_edge_sets, track, match_settings);
        }

    private:
        util::time_helper &_time_helper;
        const network_type &_network;
        mutable std::size_t _edge_size_switch{16};
        mutable double _edge_duration{std::numeric_limits<double>::signaling_NaN()},
                _index_duration{std::numeric_limits<double>::signaling_NaN()};

        void _candidate_search_individual(
                boost::container::multiset<candidate_edge_type> &candidate_edge_set, const track_type &track,
                const std::size_t index, const matching::settings &match_settings) const {
            assert(index < track.rich_line.size());

            const time_point_type &point = track.rich_line.at(index);

            boost::container::set<edge_descriptor> edge_result;

            if (match_settings.candidate_search == settings::CANDIDATE_SEARCH::COMBINED or
                match_settings.candidate_search == settings::CANDIDATE_SEARCH::NEAREST or
                match_settings.candidate_search == settings::CANDIDATE_SEARCH::NEXT) {
                edge_result = _network.query_segments_unique(
                        boost::geometry::index::nearest(point, match_settings.k_nearest));
            }

            if (match_settings.candidate_search == settings::CANDIDATE_SEARCH::COMBINED or
                match_settings.candidate_search == settings::CANDIDATE_SEARCH::CIRCLE) {
                bool found = false;
                bool adaptive_radius = match_settings.adaptive_radius;
                double current_radius = match_settings.radius;
                while (not found) {
                    if (adaptive_radius) {
                        if (index > 0) {
                            current_radius = std::min(current_radius,
                                    static_cast<double>(track.rich_line.length(index - 1)));
                        }
                        if (index < track.rich_line.size() - 1) {
                            current_radius = std::min(current_radius,
                                    static_cast<double>(track.rich_line.length(index)));
                        }

                        if (current_radius < match_settings.radius_lower_limit) {
                            // needs to be at least minimum radius
                            current_radius = match_settings.radius_lower_limit;
                        }
                    }

                    current_radius = std::min(current_radius, match_settings.radius_upper_limit);

                    if (match_settings.candidate_search == settings::CANDIDATE_SEARCH::CIRCLE) {
                        // create bounding box with at least buffer_distance
                        const auto buffer = geometry::buffer_box(point, current_radius);
                        edge_result = _network.query_segments_unique(boost::geometry::index::intersects(buffer));
                    }

                    for (auto edge_it = edge_result.begin(); edge_it != edge_result.end();) {
                        const auto &edge = _network.edge(*edge_it);
                        const auto &edge_line = edge.rich_line.line();
                        const auto distance = geometry::distance(point, edge_line);
                        if (distance > current_radius) {
                            edge_it = edge_result.erase(edge_it);
                        } else {
                            ++edge_it;
                        }
                    }

                    if (edge_result.empty() and current_radius < match_settings.radius_upper_limit) {
                        current_radius *= 2;
                        adaptive_radius = false;
                    } else {
                        found = true;
                    }
                }
            }

            if (match_settings.candidate_search == settings::CANDIDATE_SEARCH::NEXT) {
                _additions_next(edge_result, point);
            }

            _process_candidate_query(candidate_edge_set, track, index, edge_result);
        }

        void _additions_next(boost::container::set<edge_descriptor> &edge_result, const time_point_type &point) const {
            // adjacent candidates
            std::list<edge_descriptor> additions;

            for (const auto &edge_descriptor : edge_result) {
                const auto &edge = _network.edge(edge_descriptor);
                const auto &edge_line = edge.rich_line.line();
                const auto edge_distance = geometry::distance(point, edge_line);

                const auto source = _network.graph().source(edge_descriptor);
                const auto target = _network.graph().target(edge_descriptor);

                for (const auto vertex : std::array{source, target}) {
                    const auto out_edges_v = _network.graph().out_edges_view(vertex);
                    for (auto it = out_edges_v.begin(); it != out_edges_v.end(); it = out_edges_v.next(it)) {
                        const auto &adj_edge = out_edges_v[it];
                        const auto &adj_edge_line = edge.rich_line.line();
                        const auto adj_distance = geometry::distance(point, adj_edge_line);
                        if (adj_distance <= edge_distance) {
                            additions.emplace_back(it);
                        }
                    }
                }
            }

            while (not additions.empty()) {
                edge_result.emplace(std::move(additions.front()));
                additions.pop_front();
            }

            // reverse candidates
            for (const auto edge_descriptor : edge_result) {
                const auto source = _network.graph().source(edge_descriptor);
                const auto target = _network.graph().target(edge_descriptor);

                const auto out_edges_v = _network.graph().out_edges_view(target);
                for (auto it = out_edges_v.begin(); it != out_edges_v.end(); it = out_edges_v.next(it)) {
                    if (_network.graph().target(it) == source) {
                        additions.emplace_back(it);
                    }
                }
            }

            while (not additions.empty()) {
                edge_result.emplace(std::move(additions.front()));
                additions.pop_front();
            }
        }

        void _process_candidate_query(
                boost::container::multiset<candidate_edge_type> &candidate_edge_set, const track_type &track,
                const std::size_t index, boost::container::set<edge_descriptor> &edge_result) const {
            assert(index < track.rich_line.size());
            const time_point_type &point = track.rich_line.at(index);

            // remove edges from result that were already processed in candidate_edge_set
            for (const auto &candidate_edge : candidate_edge_set) {
                edge_result.erase(candidate_edge.edge_desc);
            }

            using segment_type = typename geometry::models<point_type>::segment_type;

            for (const auto &edge_descriptor : edge_result) {
                const auto &edge = _network.edge(edge_descriptor);
                if (edge.rich_line.has_length()) {
                    segment_type projection;
                    const auto &edge_line = edge.rich_line.line();
                    boost::geometry::closest_points(point, edge_line, projection);
                    auto projection_distance = geometry::point_distance(projection.first, projection.second);
                    if (projection_distance == geometry::default_float_type<distance_type>::v0) {
                        // when distance is zero, replace closest point with original point as it is already fine
                        projection.second = projection.first;
                    }

                    // adapt the switch size dynamically based on measured times
                    if (not std::isnan(_edge_duration) and not std::isnan(_index_duration)) {
                        if (_edge_duration < _index_duration) {
                            const double factor = _index_duration / _edge_duration;
                            _edge_size_switch = static_cast<std::size_t>(_edge_size_switch * factor);
                        } else {
                            const double factor = _edge_duration / _index_duration;
                            _edge_size_switch = static_cast<std::size_t>(_edge_size_switch / factor);
                        }
                        // set edge_size_switch to at least 2 since no real edge size is below 2 (exactly one segment)
                        _edge_size_switch = std::max(static_cast<std::size_t>(2), _edge_size_switch);
                    }
                    // the next both cases are equivalent, but with small edges, using the index is slower
                    if (edge_line.size() <= _edge_size_switch) {
                        // check if point is near an existing edge line point, if so, replace with it
                        const point_type *edge_point = nullptr;
                        distance_type edge_point_distance = geometry::default_float_type<distance_type>::v1;
                        double duration = util::benchmark([&]() {
                            for (const auto &current_edge_point : edge_line) {
                                auto tmp_distance = geometry::point_distance(current_edge_point, projection.second);
                                if (tmp_distance < edge_point_distance) {
                                    edge_point_distance = tmp_distance;
                                    edge_point = &current_edge_point;
                                }
                            }
                        });
                        if (edge_point != nullptr and
                            not geometry::equals_points(*edge_point, projection.second) and
                            edge_point_distance < 1e-4) {
                            // if within one millimeter, replace point with found point, so "snap to original edge point"
                            projection.second = *edge_point;
                            projection_distance = geometry::point_distance(projection.first, projection.second);
                        }
                        const double edge_factor = static_cast<double>(_edge_size_switch) / edge_line.size();
                        _edge_duration = duration * edge_factor;
                    } else {
                        std::vector<typename network_type::index_helper_type::rtree_segments_value_type>
                                nearest_segments;
                        const point_type *segment_point = nullptr;
                        distance_type segment_point_distance = geometry::default_float_type<distance_type>::v1;
                        double duration = util::benchmark([&]() {
                            // check if point is near a segment point first, if so, replace with it
                            _network.query_segments(
                                    boost::geometry::index::nearest(projection.second, 1), nearest_segments);
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
                        });
                        if (segment_point != nullptr and
                            not geometry::equals_points(*segment_point, projection.second)
                            and segment_point_distance < 1e-4) {
                            // if within one millimeter, replace point with found point, so "snap to network"
                            projection.second = *segment_point;
                            projection_distance = geometry::point_distance(projection.first, projection.second);
                        }
                        _index_duration = duration;
                    }

                    using _rich_line_type = geometry::lazy_rich_line_type<point_type>;
                    _rich_line_type from_line, to_line;

                    auto _set_from_to_edge = [&]() {
                        from_line = edge.rich_line;
                        projection.second = edge.rich_line.back();
                        projection_distance = geometry::point_distance(projection.first, projection.second);
                    };

                    auto _set_to_to_edge = [&]() {
                        to_line = edge.rich_line;
                        projection.second = edge.rich_line.front();
                        projection_distance = geometry::point_distance(projection.first, projection.second);
                    };

                    if (geometry::equals_points(edge.rich_line.front(), projection.second)) {
                        // projection is equal to front, from is empty, to is edge
                        to_line = edge.rich_line;
                    } else if (geometry::equals_points(edge.rich_line.back(), projection.second)) {
                        // projection is equal to back, from is edge, to is empty
                        from_line = edge.rich_line;
                    } else {
                        // projection is not equal, check distances
                        const auto from_distance = geometry::point_distance(edge.rich_line.front(), projection.second);
                        if (from_distance == geometry::default_float_type<distance_type>::v0) {
                            // from distance is zero, from is empty, set to to edge
                            _set_to_to_edge();
                        } else {
                            const auto to_distance = geometry::point_distance(edge.rich_line.back(), projection.second);
                            if (to_distance == geometry::default_float_type<distance_type>::v0) {
                                // to distance is zero, to is empty, set from to edge
                                _set_from_to_edge();
                            } else {
                                // projection lies within edge, so we need to cut substrings
                                from_line = geometry::substring_point_to<_rich_line_type>(
                                        edge.rich_line, projection.second, true);
                                if (not from_line.has_length() or
                                    from_line.length() == geometry::default_float_type<length_type>::v0) {
                                    // from line is still empty, set to to edge
                                    _set_to_to_edge();
                                } else if (from_line.length() == edge.rich_line.length()) {
                                    // from line has same length as edge, replace with original edge again, to_line is still empty
                                    _set_from_to_edge();
                                } else {
                                    to_line = geometry::substring_point_from<_rich_line_type>(
                                            edge.rich_line, projection.second, true);
                                    if (not to_line.has_length() or
                                        to_line.length() == geometry::default_float_type<length_type>::v0) {
                                        // to line is still empty, set from to edge
                                        _set_from_to_edge();
                                    } else if (
                                        to_line.length() == edge.rich_line.length()) {
                                        // to line has same length as edge, replace with edge again, replace from to empty
                                        from_line = _rich_line_type{};
                                        _set_to_to_edge();
                                    }
                                    // else we use the two substrings that we set
                                }
                            }
                        }
                    }

                    candidate_edge_set.emplace(candidate_edge_type{
                            false, edge_descriptor,
                            std::move(projection.second), projection_distance,
                            std::make_shared<_rich_line_type>(std::move(from_line)),
                            std::make_shared<_rich_line_type>(std::move(to_line))
                    });
                }
            }
        }

        std::vector<candidate_type> _finalize_candidates(
                std::vector<boost::container::multiset<candidate_edge_type>> &candidate_edge_sets,
                const track_type &track, const matching::settings &match_settings) const {
            const auto add_candidate = [](const auto &point, const auto &candidate_edge, auto &edge_set) {
                const auto distance = geometry::point_distance(point, candidate_edge.projection_point);

                candidate_edge_type new_candidate_edge{
                        true, candidate_edge.edge_desc,
                        candidate_edge.projection_point, distance,
                        candidate_edge.from, candidate_edge.to
                };

                const auto range = edge_set.equal_range(new_candidate_edge);

                if (range.first == edge_set.cend()) {
                    edge_set.emplace(std::move(new_candidate_edge));
                } else if (range.first != edge_set.cend()) {
                    bool emplace = true;
                    for (auto it = range.first; it != range.second; ++it) {
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

            if (match_settings.candidate_adoption_nearby) {
                using segment_type = typename geometry::models<point_type>::segment_type;

                using index_type = geometry::matcher_rtree_type<candidate_edge_type>;
                using index_value_type = typename index_type::value_type;

                // create candidates index
                std::size_t candidates_number = 0;
                for (const auto &edge_set : candidate_edge_sets) {
                    for (const auto &candidate : edge_set) {
                        if (not candidate.adopted) {
                            candidates_number++;
                        }
                    }

                    if (_time_helper.update_and_has_reached()) {
                        return {};
                    }
                }

                std::vector<index_value_type> candidates_index_values;
                candidates_index_values.reserve(candidates_number);
                for (std::size_t index = 0; index < candidate_edge_sets.size(); ++index) {
                    const time_point_type &point = track.rich_line.at(index);
                    auto &edge_set = candidate_edge_sets[index];
                    for (const auto &candidate : edge_set) {
                        if (not candidate.adopted) {
                            candidates_index_values.emplace_back(
                                    index_value_type{
                                            segment_type{point, candidate.projection_point},
                                            std::pair{index, std::cref(candidate)}
                                    });
                        }
                    }

                    if (_time_helper.update_and_has_reached()) {
                        return {};
                    }
                }

                index_type candidates_index{std::move(candidates_index_values)};

                if (_time_helper.update_and_has_reached()) {
                    return {};
                }

                // search in index for adopting nearby candidates not directly attached
                for (std::size_t index = 0; index < candidate_edge_sets.size(); ++index) {
                    const time_point_type &point = track.rich_line.at(index);
                    auto &edge_set = candidate_edge_sets[index];
                    auto edge_set_it = edge_set.crbegin();
                    if (edge_set_it != edge_set.crend()) {
                        const auto longest_distance = edge_set_it->distance;
                        if (longest_distance > 0.0) {
                            const auto buffer = geometry::buffer_box(point, longest_distance);

                            std::vector<index_value_type> results;
                            candidates_index.query(boost::geometry::index::intersects(buffer),
                                    std::back_inserter(results));

                            for (const auto &index_value : results) {
                                if (geometry::distance(point, index_value.first) <= longest_distance) {
                                    const auto &candidate_edge_pair = index_value.second;
                                    std::size_t result_index = candidate_edge_pair.first;
                                    if (index != result_index) {
                                        // skip self references
                                        const candidate_edge_type &candidate_edge = candidate_edge_pair.second;
                                        add_candidate(point, candidate_edge, edge_set);

                                        if (match_settings.candidate_adoption_reverse) {
                                            auto &reverse_candidate_edge_set = candidate_edge_sets[result_index];
                                            for (const auto &edge : edge_set) {
                                                if (not edge.adopted) {
                                                    const time_point_type &reverse_point = track.rich_line.at(
                                                            result_index);
                                                    add_candidate(reverse_point, edge, reverse_candidate_edge_set);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (_time_helper.update_and_has_reached()) {
                        return {};
                    }
                }
            }

            if (match_settings.candidate_adoption_siblings) {
                // adopt from previous and next candidate set the directly attached candidates
                for (std::size_t index = 0; index < candidate_edge_sets.size(); ++index) {
                    const time_point_type &point = track.rich_line.at(index);
                    auto &edge_set = candidate_edge_sets[index];

                    // we have a previous candidate set
                    if (index >= 1) {
                        for (const auto &candidate_edge : candidate_edge_sets[index - 1]) {
                            if (not candidate_edge.adopted) {
                                add_candidate(point, candidate_edge, edge_set);
                            }
                        }
                    }

                    // we have a next candidate set
                    if (index + 1 < candidate_edge_sets.size()) {
                        for (const auto &candidate_edge : candidate_edge_sets[index + 1]) {
                            if (not candidate_edge.adopted) {
                                add_candidate(point, candidate_edge, edge_set);
                            }
                        }
                    }

                    if (_time_helper.update_and_has_reached()) {
                        return {};
                    }
                }
            }

            // preparing node set and finalizing candidates
            std::vector<candidate_type> candidates;
            candidates.reserve(candidate_edge_sets.size());

            for (std::size_t index = 0; index < candidate_edge_sets.size(); ++index) {
                const time_point_type &point = track.rich_line.at(index);
                auto &edge_set = candidate_edge_sets[index];

                // node set
                boost::container::multiset<candidate_node_type> node_set{};
                boost::container::set<vertex_descriptor> node_check;
                for (const auto &candidate_edge : edge_set) {
                    const auto edge_descriptor = candidate_edge.edge_desc;

                    const auto source = _network.graph().source(edge_descriptor);
                    const auto target = _network.graph().target(edge_descriptor);

                    if (not node_check.contains(source)) {
                        const auto &node = _network.vertex(source);
                        const auto distance = geometry::point_distance(point, node.point);
                        node_set.emplace(candidate_node_type{source, distance});
                        node_check.emplace(source);
                    }

                    if (not node_check.contains(target)) {
                        const auto &node = _network.vertex(target);
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
                    if (index + 1 < track.rich_line.size()) {
                        next_equal = geometry::equals_points(point, track.rich_line.at(index + 1));
                    }

                    candidates.emplace_back(candidate_type{
                            track, index, next_equal, std::move(nodes), std::move(edges)
                    });
                } else {
                    auto &candidate = candidates[index];
                    candidate.nodes = std::move(nodes);
                    candidate.edges = std::move(edges);
                }

                if (_time_helper.update_and_has_reached()) {
                    return {};
                }
            }

            return candidates;
        }

    };

}

#endif //MAP_MATCHING_2_MATCHING_ALGORITHM_CANDIDATE_SEARCH_HPP
