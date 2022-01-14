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
            : multi_line{}, rich_lines{} {
        _complete();
    }

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(multi_line_type multi_line)
            : multi_line{std::move(multi_line)} {
        rich_lines.reserve(this->multi_line.size());
        for (const Line &line: this->multi_line) {
            rich_lines.emplace_back(rich_line_type{line});
        }
        _complete();
    }

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(rich_line_type rich_line)
            : multi_rich_line{std::vector{std::move(rich_line)}} {}

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(std::vector<line_type> lines)
            : multi_rich_line{lines2multi_lines(lines)} {}

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(std::vector<rich_line_type> rich_lines)
            : rich_lines{std::move(rich_lines)} {
        multi_line.reserve(this->rich_lines.size());
        for (const rich_line_type &rich_line: this->rich_lines) {
            multi_line.emplace_back(rich_line.line);
        }
        _complete();
    }

    template<typename Line>
    multi_rich_line<Line>::multi_rich_line(multi_line_type multi_line, std::vector<rich_line_type> rich_lines)
            : multi_line{std::move(multi_line)}, rich_lines{std::move(rich_lines)} {
        _complete();
    }

    template<typename Line>
    void multi_rich_line<Line>::_complete() {
        assert(multi_line.size() == rich_lines.size());

        has_length = false;
        has_azimuth = false;
        has_directions = false;
        if (not rich_lines.empty()) {
            length = default_float_type<length_type>::v0;
            for (const auto &rich_line: rich_lines) {
                if (rich_line.has_length) {
                    has_length = true;
                    length += rich_line.length;
                }
            }
            if (rich_lines.size() == 1) {
                if (rich_lines.front().has_azimuth) {
                    azimuth = rich_lines.front().azimuth;
                    has_azimuth = true;
                }
            } else {
                if (not geometry::equals_points(multi_line.front().front(), multi_line.back().back())) {
                    azimuth = geometry::azimuth_deg(multi_line.front().front(), multi_line.back().back());
                    has_azimuth = true;
                }
            }
        }
        if (not rich_lines.empty()) {
            directions = default_float_type<angle_type>::v0;
            absolute_directions = default_float_type<angle_type>::v0;
            for (const auto &rich_line: rich_lines) {
                if (rich_line.has_directions) {
                    has_directions = true;
                    directions += rich_line.directions;
                    absolute_directions += rich_line.absolute_directions;
                }
            }
        }
    }

    template<typename Line>
    std::size_t multi_rich_line<Line>::sizes() const {
        std::size_t size = 0;
        for (const line_type &line: multi_line) {
            size += line.size();
        }
        return size;
    }

    template<typename Line>
    void multi_rich_line<Line>::simplify(const bool retain_reversals, const double tolerance,
                                         const double reverse_tolerance) {
        for (auto &rich_line: rich_lines) {
            rich_line.simplify(retain_reversals, tolerance, reverse_tolerance);
        }

        multi_line.clear();
        multi_line.reserve(rich_lines.size());
        for (const rich_line_type &rich_line: rich_lines) {
            multi_line.emplace_back(rich_line.line);
        }

        _complete();
    }

    template<typename Line>
    template<typename Network>
    void multi_rich_line<Line>::median_merge(const double tolerance, const bool adaptive, const Network *network) {
        for (auto &rich_line: rich_lines) {
            rich_line.median_merge(tolerance, adaptive, network);
        }

        multi_line.clear();
        multi_line.reserve(rich_lines.size());
        for (const rich_line_type &rich_line: rich_lines) {
            multi_line.emplace_back(rich_line.line);
        }

        _complete();
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
                if (not next_rich_line.rich_segments.empty()) {
                    if (not prev_rich_line.rich_segments.empty()) {
                        const auto &prev_point = prev_rich_line.rich_segments.back().segment.second;
                        const auto &next_point = next_rich_line.rich_segments.front().segment.first;
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
    multi_rich_line<Line>::lines2multi_lines(const std::vector<line_type> &lines) {
        multi_line_type multi_line;
        multi_line.reserve(lines.size());
        for (const auto &line: lines) {
            multi_line.emplace_back(line);
        }
        return multi_line;
    }

    template<typename Line>
    std::string multi_rich_line<Line>::wkt() const {
        if (multi_line.size() <= 1) {
            return geometry::to_wkt(multi_line.front());
        } else {
            return geometry::to_wkt(multi_line);
        }
    }

    template<typename Line>
    std::string multi_rich_line<Line>::str() const {
        std::string str = "MultiRichLine(";
        str.append(wkt());
        str.append(",");
        str.append(std::to_string(length));
        str.append(")");
        return str;
    }

    template<typename Line>
    bool operator==(const multi_rich_line<Line> &left, const multi_rich_line<Line> &right) {
        // if lines are equal, everything is equal
        return geometry::equals(left.multi_line, right.multi_line);
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