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
#include <limits>

#include <boost/geometry/index/rtree.hpp>

#include "network/types.hpp"
#include "util.hpp"
#include "types.hpp"

namespace map_matching_2::geometry {

    template<typename Line>
    rich_line<Line>::rich_line()
            : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false},
              _computed_rich_segments{false}, _line{} {}

    template<typename Line>
    rich_line<Line>::rich_line(Line line)
            : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false},
              _computed_rich_segments{false}, _line{std::move(line)} {}

    template<typename Line>
    rich_line<Line>::rich_line(std::vector<rich_segment_type> rich_segments)
            : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false},
              _computed_rich_segments{true}, _rich_segments{std::move(rich_segments)},
              _line{rich_segment_type::rich_segments2line(this->_rich_segments)} {}

    template<typename Line>
    rich_line<Line>::rich_line(const rich_line &other)
            : _line{other._line} {
        _copy(other);
    }

    template<typename Line>
    rich_line<Line>::rich_line(rich_line &&other) noexcept
            : _line{std::move(other._line)} {
        _move(std::move(other));
    }

    template<typename Line>
    rich_line<Line> &rich_line<Line>::operator=(const rich_line &other) {
        _line = other._line;
        _copy(other);
        return *this;
    }

    template<typename Line>
    rich_line<Line> &rich_line<Line>::operator=(rich_line &&other) noexcept {
        _line = std::move(other._line);
        _move(std::move(other));
        return *this;
    }

    template<typename Line>
    bool rich_line<Line>::has_length() const {
        _compute_length();
        return _has_length;
    }

    template<typename Line>
    typename rich_line<Line>::length_type rich_line<Line>::length() const {
        _compute_length();
        return _length;
    }

    template<typename Line>
    bool rich_line<Line>::has_azimuth() const {
        _compute_azimuth();
        return _has_azimuth;
    }

    template<typename Line>
    typename rich_line<Line>::angle_type rich_line<Line>::azimuth() const {
        _compute_azimuth();
        return _azimuth;
    }

    template<typename Line>
    bool rich_line<Line>::has_directions() const {
        _compute_directions();
        return _has_directions;
    }

    template<typename Line>
    typename rich_line<Line>::angle_type rich_line<Line>::directions() const {
        _compute_directions();
        return _directions;
    }

    template<typename Line>
    typename rich_line<Line>::angle_type rich_line<Line>::absolute_directions() const {
        _compute_directions();
        return _absolute_directions;
    }

    template<typename Line>
    const std::vector<typename rich_line<Line>::rich_segment_type> &rich_line<Line>::rich_segments() const {
        _compute_rich_segments();
        return _rich_segments;
    }

    template<typename Line>
    const typename rich_line<Line>::line_type &rich_line<Line>::line() const {
        return _line;
    }

    template<typename Line>
    typename rich_line<Line>::angle_type rich_line<Line>::segments_direction(std::size_t index) const {
        _compute_rich_segments();

        if (index >= _rich_segments.size() - 1) {
            throw std::out_of_range("index is out of range");
        }

        const auto &a = _rich_segments[index];
        const auto &b = _rich_segments[index + 1];
        if (geometry::equals_points(a.segment().second, b.segment().first)) {
            if (a.has_azimuth() and b.has_azimuth()) {
                return direction_deg(a.azimuth(), b.azimuth());
            }
        }

        return std::numeric_limits<angle_type>::signaling_NaN();
    }

    template<typename Line>
    void
    rich_line<Line>::simplify(const bool retain_reversals, const double tolerance, const double reverse_tolerance) {
        _compute_rich_segments();

        std::list<rich_segment_type> rich_segments_list;
        std::move(_rich_segments.begin(), _rich_segments.end(), std::back_inserter(rich_segments_list));
        _rich_segments.clear();

        rich_segment_type::simplify(rich_segments_list, retain_reversals, tolerance, reverse_tolerance);

        _rich_segments.reserve(rich_segments_list.size());
        std::move(rich_segments_list.begin(), rich_segments_list.end(), std::back_inserter(_rich_segments));

        _line = rich_segment_type::rich_segments2line(_rich_segments);
        _computed_length.store(false, std::memory_order_release);
        _computed_azimuth.store(false, std::memory_order_release);
        _computed_directions.store(false, std::memory_order_release);
    }

    template<typename Line>
    template<typename Network>
    void rich_line<Line>::median_merge(const double tolerance, const bool adaptive, const Network *network) {
        using coordinate_type = typename boost::geometry::coordinate_type<point_type>::type;
        using index_key_type = std::pair<point_type, typename std::list<point_type>::iterator>;
        using index_type = boost::geometry::index::rtree<index_key_type, boost::geometry::index::rstar<16>>;

        std::list<point_type> points{_line.cbegin(), _line.cend()};

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
                    auto distance = geometry::distance(point, edge.line());
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

        _line = line_type{points.begin(), points.end()};
        _rich_segments = std::vector<rich_segment_type>{};
        _computed_length.store(false, std::memory_order_release);
        _computed_azimuth.store(false, std::memory_order_release);
        _computed_directions.store(false, std::memory_order_release);
        _computed_rich_segments.store(false, std::memory_order_release);
    }

    template<typename Line>
    rich_line<Line> rich_line<Line>::merge(const std::vector<std::reference_wrapper<rich_line>> &rich_lines) {
        // calculate new size
        std::size_t new_size = 0;
        for (const rich_line &rich_line: rich_lines) {
            new_size += rich_line.line().size();
        }

        line_type new_line;
        new_line.reserve(new_size);

        for (rich_line &next_rich_line: rich_lines) {
            auto &next_line = next_rich_line._line;
            if (not next_line.empty()) {
                if (not new_line.empty()) {
                    const auto &prev_point = new_line.back();
                    const auto &next_point = next_line.front();
                    if (not geometry::equals_points(prev_point, next_point)) {
//                    std::clog << "Prev: " << geometry::to_wkt(prev_point) << "\n"
//                              << "Next: " << geometry::to_wkt(next_point) << std::endl;
                        throw rich_line_merge_exception{};
                    }
                }

                std::move(new_line.empty() ? next_line.begin() : std::next(next_line.begin()),
                          next_line.end(), std::back_inserter(new_line));
            }
        }

        return rich_line{std::move(new_line)};
    }

    template<typename Line>
    std::string rich_line<Line>::wkt() const {
        return geometry::to_wkt(_line);
    }

    template<typename Line>
    std::string rich_line<Line>::str() const {
        std::string str = "RichLine(";
        str.append(wkt());
        if (has_length()) {
            str.append(",");
            str.append(std::to_string(length()));
        }
        str.append(")");
        return str;
    }

    template<typename Line>
    bool operator==(const rich_line<Line> &left, const rich_line<Line> &right) {
        // if lines are equal, everything is equal
        return geometry::equals(left._line, right._line);
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

    template<typename Line>
    void rich_line<Line>::_copy(const rich_line &other) {
        _transfer(other);

        _computed_rich_segments.store(false, std::memory_order_release);
        if (other._computed_rich_segments.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_rich_segments);
            std::lock_guard<std::mutex> other_lock(other._mutex_rich_segments);
            if (other._computed_rich_segments.load(std::memory_order_relaxed)) {
                _rich_segments = other._rich_segments;
                _computed_rich_segments.store(true, std::memory_order_release);
            }
        }
    }

    template<typename Line>
    void rich_line<Line>::_move(rich_line &&other) noexcept {
        _transfer(other);

        _computed_rich_segments.store(false, std::memory_order_release);
        if (other._computed_rich_segments.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_rich_segments);
            std::lock_guard<std::mutex> other_lock(other._mutex_rich_segments);
            if (other._computed_rich_segments.load(std::memory_order_relaxed)) {
                _rich_segments = std::move(other._rich_segments);
                _computed_rich_segments.store(true, std::memory_order_release);
            }
        }
    }

    template<typename Line>
    void rich_line<Line>::_transfer(const rich_line &other) {
        _computed_length.store(false, std::memory_order_release);
        if (other._computed_length.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_length);
            std::lock_guard<std::mutex> other_lock(other._mutex_length);
            if (other._computed_length.load(std::memory_order_relaxed)) {
                _length = other._length;
                _has_length = other._has_length;
                _computed_length.store(true, std::memory_order_release);
            }
        }

        _computed_azimuth.store(false, std::memory_order_release);
        if (other._computed_azimuth.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_azimuth);
            std::lock_guard<std::mutex> other_lock(other._mutex_azimuth);
            if (other._computed_azimuth.load(std::memory_order_relaxed)) {
                _azimuth = other._azimuth;
                _has_azimuth = other._has_azimuth;
                _computed_azimuth.store(true, std::memory_order_release);
            }
        }

        _computed_directions.store(false, std::memory_order_release);
        if (other._computed_directions.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_directions);
            std::lock_guard<std::mutex> other_lock(other._mutex_directions);
            if (other._computed_directions.load(std::memory_order_relaxed)) {
                _directions = other._azimuth;
                _absolute_directions = other._absolute_directions;
                _has_directions = other._has_directions;
                _computed_directions.store(true, std::memory_order_release);
            }
        }
    }

    template<typename Line>
    void rich_line<Line>::_compute_length() const {
        if (not _computed_length.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_length);
            if (not _computed_length.load(std::memory_order_relaxed)) {
                _has_length = false;
                _compute_rich_segments();
                if (not _rich_segments.empty()) {
                    _length = default_float_type<length_type>::v0;
                    for (const auto &rich_segment: _rich_segments) {
                        if (rich_segment.has_length()) {
                            _length += rich_segment.length();
                            _has_length = true;
                        }
                    }
                }
                _computed_length.store(true, std::memory_order_release);
            }
        }
    }

    template<typename Line>
    void rich_line<Line>::_compute_azimuth() const {
        if (not _computed_azimuth.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_azimuth);
            if (not _computed_azimuth.load(std::memory_order_relaxed)) {
                _has_azimuth = false;
                if (not _line.empty()) {
                    const auto &start = _line.front();
                    const auto &end = _line.back();
                    if (not geometry::equals_points(start, end)) {
                        _azimuth = geometry::azimuth_deg(start, end);
                        _has_azimuth = true;
                    }
                }
                _computed_azimuth.store(true, std::memory_order_release);
            }
        }
    }

    template<typename Line>
    void rich_line<Line>::_compute_directions() const {
        if (not _computed_directions.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_directions);
            if (not _computed_directions.load(std::memory_order_relaxed)) {
                _has_directions = false;
                _compute_rich_segments();
                if (_rich_segments.size() >= 2) {
                    _directions = default_float_type<angle_type>::v0;
                    _absolute_directions = default_float_type<angle_type>::v0;
                    for (std::size_t i = 0; i < _rich_segments.size() - 1; ++i) {
                        angle_type segment_direction = segments_direction(i);
                        if (not std::isnan(segment_direction)) {
                            _directions += segment_direction;
                            _absolute_directions += std::fabs(segment_direction);
                            _has_directions = true;
                        }
                    }
                }
                _computed_directions.store(true, std::memory_order_release);
            }
        }
    }

    template<typename Line>
    void rich_line<Line>::_compute_rich_segments() const {
        if (not _computed_rich_segments.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(_mutex_rich_segments);
            if (not _computed_rich_segments.load(std::memory_order_relaxed)) {
                _rich_segments = rich_segment_type::template line2rich_segments<std::vector>(_line);
                _computed_rich_segments.store(true, std::memory_order_release);
            }
        }
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
