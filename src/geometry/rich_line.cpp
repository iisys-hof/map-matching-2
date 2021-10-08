// Copyright (C) 2021-2021 Adrian Wöltche
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

#include "rich_line.hpp"

#include <numeric>

#include <boost/geometry/index/rtree.hpp>

#include "network/types.hpp"
#include "util.hpp"
#include "types.hpp"

namespace map_matching_2::geometry {

    template<typename Line>
    rich_line<Line>::rich_line()
            : line{}, rich_segments{} {
        _complete();
    }

    template<typename Line>
    rich_line<Line>::rich_line(Line line)
            : line{std::move(line)} {
        rich_segments = rich_segment_type::template line2rich_segments<std::vector>(this->line);
        _complete();
    }

    template<typename Line>
    rich_line<Line>::rich_line(std::vector<rich_segment_type> rich_segments)
            : rich_segments{std::move(rich_segments)} {
        line = rich_segment_type::rich_segments2line(this->rich_segments);
        _complete();
    }

    template<typename Line>
    rich_line<Line>::rich_line(Line line, std::vector<rich_segment_type> rich_segments)
            : line{std::move(line)}, rich_segments{std::move(rich_segments)} {
        _complete();
    }

    template<typename Line>
    void rich_line<Line>::_complete() {
        assert((line.size() < 2 and rich_segments.empty()) or
               (rich_segments.size() == line.size() - 1));

        has_length = false;
        has_azimuth = false;
        has_directions = false;
        if (line.size() >= 2) {
            length = default_float_type<length_type>::v0;
            for (const auto &rich_segment: rich_segments) {
                if (rich_segment.has_length) {
                    has_length = true;
                    length += rich_segment.length;
                }
            }
            if (not geometry::equals_points(line.front(), line.back())) {
                azimuth = geometry::azimuth_line_deg(line);
                has_azimuth = true;
            }
        }
        if (line.size() >= 3) {
            segments_directions = rich_segment_type::template rich_segments_directions_deg<std::vector>(rich_segments);
            segments_absolute_directions = std::vector<angle_type>{};
            segments_absolute_directions.reserve(segments_directions.size());
            directions = default_float_type<angle_type>::v0;
            absolute_directions = default_float_type<angle_type>::v0;
            for (const auto segment_direction: segments_directions) {
                const auto segment_absolute_direction = std::fabs(segment_direction);
                segments_absolute_directions.emplace_back(segment_absolute_direction);
                directions += segment_direction;
                absolute_directions += segment_absolute_direction;
            }
            has_directions = true;
        }
    }

    template<typename Line>
    void
    rich_line<Line>::simplify(const bool retain_reversals, const double tolerance, const double reverse_tolerance) {
        std::list<rich_segment_type> rich_segments_list;
        std::move(rich_segments.begin(), rich_segments.end(), std::back_inserter(rich_segments_list));
        rich_segments.clear();

        rich_segment_type::simplify(rich_segments_list, retain_reversals, tolerance, reverse_tolerance);

        rich_segments.reserve(rich_segments_list.size());
        std::move(rich_segments_list.begin(), rich_segments_list.end(), std::back_inserter(rich_segments));

        line = rich_segment_type::rich_segments2line(rich_segments);
        _complete();
    }

    template<typename Line>
    template<typename Network>
    void rich_line<Line>::median_merge(const double tolerance, const bool adaptive, const Network *network) {
        using coordinate_type = typename boost::geometry::coordinate_type<point_type>::type;
        using index_key_type = std::pair<point_type, typename std::list<point_type>::iterator>;
        using index_type = boost::geometry::index::rtree<index_key_type, boost::geometry::index::rstar<16>>;

        std::list<point_type> points;
        std::move(line.begin(), line.end(), std::back_inserter(points));
        line.clear();

        std::vector<index_key_type> index_points;
        index_points.reserve(points.size());
        for (auto it = points.begin(); it != points.end(); it++) {
            index_points.emplace_back(index_key_type{*it, it});
        }

        index_type points_index{index_points};

        std::size_t min_buffer_points = 8;

        auto it = points.begin();
        while (it != points.end()) {
            const auto &point = *it;

            double current_tolerance = tolerance;
            if (adaptive and network != nullptr) {
                const auto edge_result = network->query_segments_unique(boost::geometry::index::nearest(point, 1));
                if (not edge_result.empty()) {
                    const auto &edge = network->graph[*edge_result.cbegin()];
                    auto distance = geometry::distance(point, edge.line);
                    current_tolerance = std::max(tolerance, distance * 0.5);
                }
            }

            std::size_t automatic_buffer_points = geometry::next_pow2((std::uint64_t) (current_tolerance / 4));
            std::size_t buffer_points = std::max(min_buffer_points, automatic_buffer_points);

            const auto buffer = geometry::buffer(point, buffer_points, current_tolerance);
            std::list<index_key_type> results;
            points_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(results));

            // only retain connected points
            if (results.size() > 1) {
//                std::cout << geometry::to_wkt(buffer) << std::endl;

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
                        current_it++;
                    } else {
                        result_it++;
                    }

                    if (result_it == results.end() or current_it == points.end()) {
                        search = false;
                    }
                }

                if (not connected_results.empty()) {
                    std::vector<point_type> result_points;
                    result_points.reserve(connected_results.size());
                    for (const auto &result: connected_results) {
                        result_points.emplace_back(result.first);
                    }

                    point_type median_point = geometry::median_point(result_points);

                    for (const auto &result: connected_results) {
                        points.erase(result.second);
                        points_index.remove(result);
                    }

                    it = current_it;
                    if (it != points.end()) {
                        points.insert(it, median_point);
                    } else {
                        points.emplace_back(median_point);
                    }
                }
            } else {
                it++;
            }
        }

        line.reserve(points.size());
        std::move(points.begin(), points.end(), std::back_inserter(line));

        rich_segments = rich_segment_type::template line2rich_segments<std::vector>(line);
        _complete();
    }

    template<typename Line>
    rich_line<Line> rich_line<Line>::merge(const std::vector<std::reference_wrapper<rich_line>> &rich_lines) {
        // calculate new size
        std::size_t new_size = 0;
        for (const rich_line &rich_line: rich_lines) {
            new_size += rich_line.line.size();
        }

        std::vector<rich_segment_type> new_rich_segments;
        new_rich_segments.reserve(new_size);

        for (rich_line &next_rich_line: rich_lines) {
            if (next_rich_line.has_length) {
                auto next_rich_segments_it = next_rich_line.rich_segments.begin();
                if (not new_rich_segments.empty()) {
                    const auto &prev_point = new_rich_segments.back().segment.second;
                    const auto &next_point = (*next_rich_segments_it).segment.first;
                    if (not geometry::equals_points(prev_point, next_point)) {
//                        std::clog << "Prev: " << geometry::to_wkt(prev_point) << "\n"
//                                  << "Next: " << geometry::to_wkt(next_point) << std::endl;
                        throw rich_line_merge_exception{};
                    }
                }

                std::move(next_rich_segments_it, next_rich_line.rich_segments.end(),
                          std::back_inserter(new_rich_segments));
            }
        }

        return {std::move(new_rich_segments)};
    }

    template<typename Line>
    std::string rich_line<Line>::wkt() const {
        return geometry::to_wkt(line);
    }

    template<typename Line>
    std::string rich_line<Line>::str() const {
        std::string str = "RichLine(";
        str.append(wkt());
        str.append(",");
        str.append(std::to_string(length));
        str.append(")");
        return str;
    }

    template<typename Line>
    bool operator==(const rich_line<Line> &left, const rich_line<Line> &right) {
        // if lines are equal, everything is equal
        return geometry::equals(left.line, right.line);
    }

    template<typename Line>
    bool operator!=(const rich_line<Line> &left, const rich_line<Line> &right) {
        return not(left == right);
    }

    template<typename Line>
    std::ostream &operator<<(std::ostream &out, const rich_line<Line> &rich_line) {
        out << rich_line.str();
        return out;
    }

    template
    class rich_line<geometry::types_geographic::line_type>;

    template
    class rich_line<geometry::types_cartesian::line_type>;

    template
    void rich_line<geometry::types_geographic::line_type>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_geographic::network_static *network);

    template
    void rich_line<geometry::types_cartesian::line_type>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_cartesian::network_static *network);

    template
    bool operator==(const rich_line <geometry::types_geographic::line_type> &left,
                    const rich_line <geometry::types_geographic::line_type> &right);

    template
    bool operator==(const rich_line <geometry::types_cartesian::line_type> &left,
                    const rich_line <geometry::types_cartesian::line_type> &right);

    template
    bool operator!=(const rich_line <geometry::types_geographic::line_type> &left,
                    const rich_line <geometry::types_geographic::line_type> &right);

    template
    bool operator!=(const rich_line <geometry::types_cartesian::line_type> &left,
                    const rich_line <geometry::types_cartesian::line_type> &right);

    template
    std::ostream &operator<<(std::ostream &out, const rich_line <geometry::types_geographic::line_type> &rich_line);

    template
    std::ostream &operator<<(std::ostream &out, const rich_line <geometry::types_cartesian::line_type> &rich_line);

}
