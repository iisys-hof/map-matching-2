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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_MEDIAN_MERGE_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_MEDIAN_MERGE_HPP

#include <concepts>
#include <functional>
#include <list>
#include <memory_resource>

#include "types/geometry/index/median_merge.hpp"

#include "geometry/point.hpp"

#include "distance.hpp"
#include "buffer.hpp"
#include "median.hpp"

namespace map_matching_2::geometry {

    template<typename Line, typename ToleranceAdjuster>
    [[nodiscard]] Line median_merge(const Line &line, const ToleranceAdjuster &tolerance_adjuster,
            const double tolerance = 1e-3) requires
        (std::default_initializable<Line>) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));

        using line_type = Line;
        using point_type = point_type_t<line_type>;

        using index_type = median_merge_rtree_type<point_type>;
        using index_key_type = typename index_type::value_type;

        std::pmr::monotonic_buffer_resource _buffer{1024};

        std::pmr::list<point_type> points{line.cbegin(), line.cend(), &_buffer};

        std::vector<index_key_type> index_points;
        index_points.reserve(points.size());
        for (auto it = points.begin(); it != points.end(); ++it) {
            index_points.emplace_back(index_key_type{*it, it});
        }

        index_type points_index{index_points};

        auto it = points.begin();
        while (it != points.end()) {
            const point_type &point = *it;

            double current_tolerance = tolerance_adjuster(point, tolerance);

            const auto buffer = geometry::buffer_box(point, current_tolerance);
            std::pmr::list<index_key_type> results{&_buffer};
            points_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(results));

            // remove results outside current tolerance
            for (auto results_it = results.begin(); results_it != results.end();) {
                if (geometry::point_distance(point, results_it->first) > current_tolerance) {
                    results_it = results.erase(results_it);
                } else {
                    ++results_it;
                }
            }

            // only retain connected points
            if (not results.empty()) {
                // std::cout << geometry::to_wkt(buffer) << std::endl;

                // only continue if we found at least one other point next to our current point
                std::vector<index_key_type> connected_results;
                connected_results.reserve(results.size());
                auto current_it = it;
                bool search = true;
                auto result_it = results.begin();
                while (search) {
                    auto &result = *result_it;
                    if (current_it == result.second) {
                        // found current point
                        connected_results.emplace_back(std::move(result));
                        results.erase(result_it);
                        // start again
                        result_it = results.begin();
                        ++current_it;
                    } else {
                        ++result_it;
                    }

                    if (result_it == results.end() or current_it == points.end()) {
                        search = false;
                    }
                }

                if (not connected_results.empty()) {
                    std::vector<point_type> result_points;
                    result_points.reserve(connected_results.size());
                    for (const auto &result : connected_results) {
                        result_points.emplace_back(result.first);
                    }

                    point_type median_point;
                    if (result_points.size() == 1) {
                        median_point = result_points.front();
                    } else {
                        median_point = geometry::median_point_dispatcher(result_points);
                    }

                    for (const auto &result : connected_results) {
                        points_index.remove(result);
                        points.erase(result.second);
                    }

                    it = current_it;
                    if (it != points.end()) {
                        points.insert(it, median_point);
                    } else {
                        points.emplace_back(median_point);
                    }
                }
            } else {
                ++it;
            }
        }

        return line_type{points.cbegin(), points.cend()};
    }

    template<typename Line>
    [[nodiscard]] Line median_merge(const Line &line, const double tolerance = 1e-3) requires
        (std::default_initializable<Line>) {
        const auto &tolerance_adjuster =
                [](const point_type_t<Line> &_point, const double _tolerance) constexpr -> double {
            return _tolerance;
        };

        return median_merge(line, tolerance_adjuster, tolerance);
    }

    template<typename Line, typename Network>
    [[nodiscard]] Line median_merge(const Line &line, const double tolerance = 1e-3, const bool adaptive = true,
            const Network *network = nullptr) {
        using line_type = Line;
        using point_type = point_type_t<line_type>;

        const auto &adaptive_tolerance_adjuster =
                [adaptive, network](const point_type &_point, const double _tolerance) -> double {
            double current_tolerance = _tolerance;
            if (adaptive and network != nullptr) {
                const auto edge_result = network->query_segments_unique(boost::geometry::index::nearest(_point, 1));
                if (not edge_result.empty()) {
                    const auto &edge = network->edge(*edge_result.cbegin());
                    auto distance = geometry::distance(_point, edge.rich_line.line());
                    current_tolerance = std::max(_tolerance, distance * 0.5);
                }
            }
            return current_tolerance;
        };

        return median_merge(line, adaptive_tolerance_adjuster, tolerance);
    }

}

#endif //MEDIAN_MERGE_HPP
