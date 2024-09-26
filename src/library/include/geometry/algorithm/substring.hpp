// Copyright (C) 2021-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_SUBSTRING_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_SUBSTRING_HPP

#include "geometry/rich_type/traits/rich_line.hpp"

#include "geometry/common.hpp"

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/distance.hpp"

namespace map_matching_2::geometry {

    template<is_rich_line RichLine>
    [[nodiscard]] RichLine substring(
            const RichLine &rich_line,
            typename rich_line_traits<RichLine>::length_type from_distance,
            typename rich_line_traits<RichLine>::length_type to_distance,
            const typename rich_line_traits<RichLine>::point_type &from_point = {},
            const typename rich_line_traits<RichLine>::point_type &to_point = {},
            const bool use_from_point = false, const bool use_to_point = false) {
        using rich_line_type = RichLine;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using line_type = typename rich_line_traits<rich_line_type>::memory_line_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;

        if (rich_line.empty()) {
            return rich_line_type{};
        }

        if (use_from_point and use_to_point and geometry::equals_points(from_point, to_point)) {
            return rich_line_type{};
        }

        if (rich_line.size() == 1) {
            return rich_line;
        }

        if (rich_line.has_length() and
            from_distance == geometry::default_float_type<length_type>::v0 and to_distance == rich_line.length()) {
            return rich_line;
        }

        const auto &line = rich_line.line();
        std::size_t from_index = 0, to_index = line.size();
        // only recombine when substring is in same direction as original line
        if (rich_line.has_length() and from_distance < to_distance) {
            point_type from, to;
            if (use_from_point) {
                from = from_point;
            } else {
                boost::geometry::line_interpolate(line, from_distance, from);
            }

            if (use_to_point) {
                to = to_point;
            } else {
                boost::geometry::line_interpolate(line, to_distance, to);
            }

            length_type current_distance = default_float_type<length_type>::v0;
            bool found{false};
            for (std::size_t i = 0; i < line.size() - 1; ++i) {
                const auto segment = rich_line.rich_referring_segment(i);
                current_distance += segment.length();

                if (not found and current_distance > from_distance) {
                    from_index = i + 1;
                    found = true;
                }

                if (current_distance >= to_distance) {
                    to_index = i + 1;
                    break;
                }
            }

            rich_line_type _from{}, _to{};
            rich_line_type _middle = rich_line.sub_rich_line(from_index, to_index);
            if (_middle.empty()) {
                _from = rich_line_type{line_type{from, to}};
            } else {
                if (not geometry::equals_points(from, _middle.front())) {
                    _from = rich_line_type{line_type{from, _middle.front()}};
                }
                if (not geometry::equals_points(_middle.back(), to)) {
                    _to = rich_line_type{line_type{_middle.back(), to}};
                }
            }
            return _from.merge(std::move(_middle), std::move(_to));
        }

        return rich_line_type{};
    }

    template<is_rich_line RichLine>
    [[nodiscard]] RichLine substring_from(
            const RichLine &rich_line,
            typename rich_line_traits<RichLine>::length_type from_distance,
            const typename rich_line_traits<RichLine>::point_type &from_point = {},
            const bool use_from_point = false) {
        return substring(rich_line, from_distance, rich_line.length(),
                from_point, rich_line.back(), use_from_point, true);
    }

    template<is_rich_line RichLine>
    [[nodiscard]] RichLine substring_to(
            const RichLine &rich_line,
            typename rich_line_traits<RichLine>::length_type to_distance,
            const typename rich_line_traits<RichLine>::point_type &to_point = {},
            const bool use_to_point = false) {
        using rich_line_type = RichLine;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;

        return substring(rich_line, geometry::default_float_type<length_type>::v0, to_distance,
                rich_line.front(), to_point, true, use_to_point);
    }

    template<is_rich_line RichLine>
    [[nodiscard]] std::pair<typename rich_line_traits<RichLine>::length_type,
        typename rich_line_traits<RichLine>::length_type>
    distances(const RichLine &rich_line,
            const typename rich_line_traits<RichLine>::point_type &from,
            const typename rich_line_traits<RichLine>::point_type &to,
            const bool calculate_from = true, const bool calculate_to = true,
            const bool use_from_point = false, const bool use_to_point = false) {
        using rich_line_type = RichLine;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;

        length_type from_distance = default_float_type<length_type>::v0;
        length_type to_distance = default_float_type<length_type>::v0;

        if (rich_line.size() <= 1) {
            return std::pair{from_distance, to_distance};
        }

        // find nearest segments with projections
        std::size_t from_nearest_segment, to_nearest_segment;
        distance_type from_smallest_distance = std::numeric_limits<distance_type>::infinity(),
                to_smallest_distance = std::numeric_limits<distance_type>::infinity();
        point_type from_projection, to_projection;

        if (calculate_from) {
            // search from_projection from start
            for (std::size_t i = 0; i < rich_line.size() - 1; ++i) {
                const auto rich_segment = rich_line.rich_referring_segment(i);
                if (geometry::equals_points(from, rich_segment.first())) {
                    from_smallest_distance = geometry::default_float_type<distance_type>::v0;
                    from_nearest_segment = i;
                    from_projection = from;
                    break;
                } else {
                    segment_type projection;
                    boost::geometry::closest_points(from, rich_segment.segment(), projection);
                    const auto distance = geometry::point_distance(projection.first, projection.second);

                    if (distance < from_smallest_distance) {
                        from_smallest_distance = distance;
                        from_nearest_segment = i;
                        if (use_from_point) {
                            from_projection = from;
                        } else {
                            from_projection = projection.second;
                        }
                    }
                }
            }

            const auto rich_segment = rich_line.rich_referring_segment(from_nearest_segment);
            const auto distance_from = geometry::distance(rich_segment.first(), from);
            if (distance_from > rich_segment.length() and from_nearest_segment + 1 < rich_line.size() - 1) {
                from_nearest_segment++;
            }
        }

        if (calculate_to) {
            // search to_projection from end
            for (std::int64_t i = rich_line.size() - 2; i >= 0; --i) {
                const auto rich_segment = rich_line.rich_referring_segment(i);
                if (geometry::equals_points(to, rich_segment.second())) {
                    to_smallest_distance = geometry::default_float_type<distance_type>::v0;
                    to_nearest_segment = i;
                    to_projection = to;
                    break;
                } else {
                    segment_type projection;
                    boost::geometry::closest_points(to, rich_segment.segment(), projection);
                    const auto distance = geometry::point_distance(projection.first, projection.second);

                    if (distance < to_smallest_distance) {
                        to_smallest_distance = distance;
                        to_nearest_segment = i;
                        if (use_to_point) {
                            to_projection = to;
                        } else {
                            to_projection = projection.second;
                        }
                    }
                }
            }

            const auto rich_segment = rich_line.rich_referring_segment(to_nearest_segment);
            const auto distance_to = geometry::distance(rich_segment.second(), to);
            if (distance_to > rich_segment.length() and to_nearest_segment > 0) {
                to_nearest_segment--;
            }
        }

        // calculate distances along line
        bool found_from_distance = !calculate_from;
        bool found_to_distance = !calculate_to;
        for (std::size_t i = 0; i < rich_line.size() - 1; ++i) {
            const auto rich_segment = rich_line.rich_referring_segment(i);
            const auto distance = rich_segment.length();

            if (calculate_from) {
                if (i < from_nearest_segment) {
                    from_distance += distance;
                } else if (i == from_nearest_segment) {
                    if (geometry::equals_points(rich_segment.second(), from_projection)) {
                        from_distance += distance;
                        found_from_distance = true;
                    } else {
                        const auto partial_distance = geometry::distance(rich_segment.first(), from_projection);
                        from_distance += partial_distance;
                        found_from_distance = true;
                    }
                }
            }

            if (calculate_to) {
                if (i < to_nearest_segment) {
                    to_distance += distance;
                } else if (i == to_nearest_segment) {
                    if (geometry::equals_points(rich_segment.second(), to_projection)) {
                        to_distance += distance;
                        found_to_distance = true;
                    } else {
                        const auto partial_distance = geometry::distance(rich_segment.first(), to_projection);
                        to_distance += partial_distance;
                        found_to_distance = true;
                    }
                }
            }

            if (found_from_distance and found_to_distance) {
                break;
            }
        }

        // std::cout << std::setprecision(10)
        //           << "from_i: " << from_nearest_segment << ", " << boost::geometry::wkt(from_projection) << ", "
        //           << from_distance << ", to_i: " << to_nearest_segment << ", " << boost::geometry::wkt(to_projection)
        //           << ", " << to_distance << std::endl;

        return std::pair{from_distance, to_distance};
    }

    template<is_rich_line RichLine>
    [[nodiscard]] typename rich_line_traits<RichLine>::length_type
    distance_from(const RichLine &rich_line,
            const typename rich_line_traits<RichLine>::point_type &from,
            const bool use_from_point = false) {
        const auto distances_pair = distances(rich_line, from, {}, true, false, use_from_point, false);
        return distances_pair.first;
    }

    template<is_rich_line RichLine>
    [[nodiscard]] typename rich_line_traits<RichLine>::length_type
    distance_to(const RichLine &rich_line,
            const typename rich_line_traits<RichLine>::point_type &to,
            const bool use_to_point = false) {
        const auto distances_pair = distances(rich_line, {}, to, false, true, false, use_to_point);
        return distances_pair.second;
    }

    template<is_rich_line RichLine>
    [[nodiscard]] RichLine substring_points(
            const RichLine &rich_line,
            const typename rich_line_traits<RichLine>::point_type &from,
            const typename rich_line_traits<RichLine>::point_type &to,
            const bool use_from_point = false, const bool use_to_point = false) {
        const auto distances_pair = distances(rich_line, from, to, true, true, use_from_point, use_to_point);
        return substring(rich_line, distances_pair.first, distances_pair.second,
                from, to, use_from_point, use_to_point);
    }

    template<is_rich_line RichLine>
    [[nodiscard]] RichLine substring_point_from(
            const RichLine &rich_line,
            const typename rich_line_traits<RichLine>::point_type &from,
            const bool use_from_point = false) {
        const auto distance = distance_from(rich_line, from, use_from_point);
        return substring_from(rich_line, distance, from, use_from_point);
    }

    template<is_rich_line RichLine>
    [[nodiscard]] RichLine substring_point_to(
            const RichLine &rich_line,
            const typename rich_line_traits<RichLine>::point_type &to,
            const bool use_to_point = false) {
        const auto distance = distance_to(rich_line, to, use_to_point);
        return substring_to(rich_line, distance, to, use_to_point);
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_SUBSTRING_HPP
