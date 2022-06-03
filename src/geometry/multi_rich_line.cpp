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

#include "multi_rich_line.hpp"

#include "network/types.hpp"
#include "util.hpp"
#include "types.hpp"

namespace map_matching_2::geometry {

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line()
            : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false},
              _computed_rich_lines{false}, _multi_line{} {}

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(multi_line_type multi_line)
            : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false},
              _computed_rich_lines{false}, _multi_line{std::move(multi_line)} {}

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(rich_line_type rich_line)
            : multi_rich_line{std::vector{std::move(rich_line)}} {}

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(std::vector<line_type> lines)
            : multi_rich_line{lines2multi_line(lines)} {}

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(std::vector<rich_line_type> rich_lines)
            : _computed_length{false}, _computed_azimuth{false}, _computed_directions{false},
              _computed_rich_lines{true}, _rich_lines{std::move(rich_lines)},
              _multi_line{rich_lines2multi_line(this->_rich_lines)} {}

    template<typename Line>
    bool multi_rich_line<Line>::has_length() const {
        _compute_length();
        return _has_length;
    }

    template<typename Line>
    typename multi_rich_line<Line>::length_type multi_rich_line<Line>::length() const {
        _compute_length();
        return _length;
    }

    template<typename Line>
    bool multi_rich_line<Line>::has_azimuth() const {
        _compute_azimuth();
        return _has_azimuth;
    }

    template<typename Line>
    typename multi_rich_line<Line>::angle_type multi_rich_line<Line>::azimuth() const {
        _compute_azimuth();
        return _azimuth;
    }

    template<typename Line>
    bool multi_rich_line<Line>::has_directions() const {
        _compute_directions();
        return _has_directions;
    }

    template<typename Line>
    typename multi_rich_line<Line>::angle_type multi_rich_line<Line>::directions() const {
        _compute_directions();
        return _directions;
    }

    template<typename Line>
    typename multi_rich_line<Line>::angle_type multi_rich_line<Line>::absolute_directions() const {
        _compute_directions();
        return _absolute_directions;
    }

    template<typename Line>
    const std::vector<typename multi_rich_line<Line>::rich_line_type> &multi_rich_line<Line>::rich_lines() const {
        _compute_rich_lines();
        return _rich_lines;
    }

    template<typename Line>
    const typename multi_rich_line<Line>::multi_line_type &multi_rich_line<Line>::multi_line() const {
        return _multi_line;
    }

    template<typename Line>
    std::size_t multi_rich_line<Line>::sizes() const {
        std::size_t size = 0;
        for (const line_type &line: _multi_line) {
            size += line.size();
        }
        return size;
    }

    template<typename Line>
    void multi_rich_line<Line>::simplify(const bool retain_reversals, const double tolerance,
                                         const double reverse_tolerance) {
        _compute_rich_lines();

        for (auto &rich_line: _rich_lines) {
            rich_line.simplify(retain_reversals, tolerance, reverse_tolerance);
        }

        _multi_line.clear();
        _multi_line.reserve(_rich_lines.size());
        for (const rich_line_type &rich_line: _rich_lines) {
            _multi_line.emplace_back(rich_line.line());
        }

        _computed_length = false;
        _computed_azimuth = false;
        _computed_directions = false;
    }

    template<typename Line>
    template<typename Network>
    void multi_rich_line<Line>::median_merge(const double tolerance, const bool adaptive, const Network *network) {
        _compute_rich_lines();

        for (auto &rich_line: _rich_lines) {
            rich_line.median_merge(tolerance, adaptive, network);
        }

        _multi_line = rich_lines2multi_line(_rich_lines);
        _computed_length = false;
        _computed_azimuth = false;
        _computed_directions = false;
    }

    template<typename Line>
    multi_rich_line<Line>
    multi_rich_line<Line>::merge(const std::vector<std::reference_wrapper<rich_line_type>> &rich_lines) {
        std::vector<rich_line_type> new_rich_lines;

        auto it = rich_lines.cbegin();
        auto start_it = it;
        while (it != rich_lines.cend()) {
            if (it != rich_lines.cbegin()) {
                const rich_line_type &prev_rich_line = *std::prev(it);
                const rich_line_type &next_rich_line = *it;
                if (not next_rich_line.line().empty()) {
                    if (not prev_rich_line.line().empty()) {
                        const auto &prev_point = prev_rich_line.line().back();
                        const auto &next_point = next_rich_line.line().front();
                        if (not geometry::equals_points(prev_point, next_point)) {
                            new_rich_lines.emplace_back(rich_line_type::merge({start_it, it}));
                            start_it = it;
                        }
                    }
                }
            }
            it++;
        }

        new_rich_lines.emplace_back(rich_line_type::merge({start_it, it}));

        return multi_rich_line{std::move(new_rich_lines)};
    }

    template<typename Line>
    typename multi_rich_line<Line>::multi_line_type
    multi_rich_line<Line>::lines2multi_line(const std::vector<line_type> &lines) {
        multi_line_type multi_line;
        multi_line.reserve(lines.size());
        for (const auto &line: lines) {
            multi_line.emplace_back(line);
        }
        return multi_line;
    }

    template<typename Line>
    typename multi_rich_line<Line>::multi_line_type
    multi_rich_line<Line>::rich_lines2multi_line(const std::vector<rich_line_type> &rich_lines) {
        multi_line_type multi_line;
        multi_line.reserve(rich_lines.size());
        for (const auto &rich_line: rich_lines) {
            multi_line.emplace_back(rich_line.line());
        }
        return multi_line;
    }

    template<typename Line>
    std::string multi_rich_line<Line>::wkt() const {
        if (_multi_line.empty()) {
            return geometry::to_wkt(line_type{});
        } else if (_multi_line.size() <= 1) {
            return geometry::to_wkt(_multi_line.front());
        } else {
            return geometry::to_wkt(_multi_line);
        }
    }

    template<typename Line>
    std::string multi_rich_line<Line>::str() const {
        std::string str = "MultiRichLine(";
        str.append(wkt());
        if (has_length()) {
            str.append(",");
            str.append(std::to_string(length()));
        }
        str.append(")");
        return str;
    }

    template<typename Line>
    bool operator==(const multi_rich_line<Line> &left, const multi_rich_line<Line> &right) {
        // if lines are equal, everything is equal
        return geometry::equals(left._multi_line, right._multi_line);
    }

    template<typename Line>
    bool operator!=(const multi_rich_line<Line> &left, const multi_rich_line<Line> &right) {
        return not(left == right);
    }

    template<typename Line>
    std::ostream &operator<<(std::ostream &out, const multi_rich_line<Line> &multi_rich_line) {
        out << multi_rich_line.str();
        return out;
    }

    template<typename Line>
    void multi_rich_line<Line>::_compute_length() const {
        if (not _computed_length) {
            _has_length = false;
            _compute_rich_lines();
            if (not _rich_lines.empty()) {
                _length = default_float_type<length_type>::v0;
                for (const auto &rich_line: _rich_lines) {
                    if (rich_line.has_length()) {
                        _length += rich_line.length();
                        _has_length = true;
                    }
                }
            }
            _computed_length = true;
        }
    }

    template<typename Line>
    void multi_rich_line<Line>::_compute_azimuth() const {
        if (not _computed_azimuth) {
            _has_azimuth = false;
            _compute_rich_lines();
            if (not _rich_lines.empty()) {
                if (_rich_lines.size() == 1) {
                    const auto &rich_line = _rich_lines.front();
                    if (rich_line.has_azimuth()) {
                        _azimuth = rich_line.azimuth();
                        _has_azimuth = true;
                    }
                } else {
                    const auto &a = _multi_line.front().front();
                    const auto &b = _multi_line.back().back();
                    if (not geometry::equals_points(a, b)) {
                        _azimuth = geometry::azimuth_deg(a, b);
                        _has_azimuth = true;
                    }
                }
            }
            _computed_azimuth = true;
        }
    }

    template<typename Line>
    void multi_rich_line<Line>::_compute_directions() const {
        if (not _computed_directions) {
            _has_directions = false;
            _compute_rich_lines();
            if (not _rich_lines.empty()) {
                _directions = default_float_type<angle_type>::v0;
                _absolute_directions = default_float_type<angle_type>::v0;
                for (const auto &rich_line: _rich_lines) {
                    if (rich_line.has_directions()) {
                        _directions += rich_line.directions();
                        _absolute_directions += rich_line.absolute_directions();
                        _has_directions = true;
                    }
                }
            }
            _computed_directions = true;
        }
    }

    template<typename Line>
    void multi_rich_line<Line>::_compute_rich_lines() const {
        if (not _computed_rich_lines) {
            _rich_lines.reserve(_multi_line.size());
            for (const line_type &line: _multi_line) {
                _rich_lines.emplace_back(rich_line_type{line});
            }
            _computed_rich_lines = true;
        }
    }

    template
    class multi_rich_line<geometry::types_geographic::line_type>;

    template
    class multi_rich_line<geometry::types_cartesian::line_type>;

    template
    void multi_rich_line<geometry::types_geographic::line_type>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_geographic::network_static *network);

    template
    void multi_rich_line<geometry::types_cartesian::line_type>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_cartesian::network_static *network);

    template
    bool operator==(const multi_rich_line <geometry::types_geographic::line_type> &left,
                    const multi_rich_line <geometry::types_geographic::line_type> &right);

    template
    bool operator==(const multi_rich_line <geometry::types_cartesian::line_type> &left,
                    const multi_rich_line <geometry::types_cartesian::line_type> &right);

    template
    bool operator!=(const multi_rich_line <geometry::types_geographic::line_type> &left,
                    const multi_rich_line <geometry::types_geographic::line_type> &right);

    template
    bool operator!=(const multi_rich_line <geometry::types_cartesian::line_type> &left,
                    const multi_rich_line <geometry::types_cartesian::line_type> &right);

    template
    std::ostream &
    operator<<(std::ostream &out, const multi_rich_line <geometry::types_geographic::line_type> &multi_rich_line);

    template
    std::ostream &
    operator<<(std::ostream &out, const multi_rich_line <geometry::types_cartesian::line_type> &multi_rich_line);

}