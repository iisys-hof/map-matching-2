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

#ifndef MAP_MATCHING_2_SUBSTRING_HPP
#define MAP_MATCHING_2_SUBSTRING_HPP

#include "rich_line.hpp"

namespace map_matching_2::geometry {

    template<typename RichLine>
    RichLine substring(const RichLine &rich_line,
                       typename rich_line_traits<RichLine>::length_type from_distance,
                       typename rich_line_traits<RichLine>::length_type to_distance,
                       const typename rich_line_traits<RichLine>::point_type &from_point = {},
                       const typename rich_line_traits<RichLine>::point_type &to_point = {},
                       bool use_from_point = false, bool use_to_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;
        using rich_segment_type = typename rich_line_traits<RichLine>::rich_segment_type;
        using segment_type = typename rich_line_traits<RichLine>::segment_type;
        using length_type = typename rich_line_traits<RichLine>::length_type;
        using angle_type = typename rich_line_traits<RichLine>::angle_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        if (rich_line.rich_segments().empty()) {
            return RichLine{};
        }

        if (use_from_point and use_to_point and geometry::equals_points(from_point, to_point)) {
            return RichLine{};
        }

        if (rich_line.has_length() and
            from_distance == geometry::default_float_type<length_type>::v0 and to_distance == rich_line.length()) {
            return rich_line;
        }

        line_type line = rich_line.line();
        const auto &rich_segments = rich_line.rich_segments();
        std::size_t from_index = 0, to_index = line.size() - 1;
        std::size_t from_custom_index = 0, to_custom_index = line.size() - 1;
        line_type new_line;
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

//            std::cout << std::setprecision(10)
//                      << "from_point: " << boost::geometry::wkt(from)
//                      << ", to_point: " << boost::geometry::wkt(to) << std::endl;

            bool from_added = false, to_added = false;
            new_line.reserve(line.size());
            length_type current_distance = default_float_type<length_type>::v0;
            for (std::size_t i = 0, j = 1; j < line.size(); ++i, ++j) {
                const auto segment_length = rich_segments[i].length();
                current_distance += segment_length;

                if (from_distance < current_distance) {
                    if (new_line.empty()) {
                        new_line.emplace_back(from);
                        from_custom_index = i;
                        from_index = i;
                        if (not geometry::equals_points(line[from_index], from)) {
                            from_index = i + 1;
                        }
                        from_added = true;
                    } else if (from_added) {
                        new_line.emplace_back(line[i]);
                    }
                }

                if (from_added and to_distance <= current_distance) {
                    new_line.emplace_back(to);
                    to_index = j;
                    to_custom_index = j;
                    if (not geometry::equals_points(line[to_index], to)) {
                        to_index = j - 1;
                    }
                    to_added = true;
                    break;
                }
            }

            if (not from_added) {
                new_line.emplace_back(from);
                if (not geometry::equals_points(line[from_index], from)) {
                    from_index++;
                }
                if (line.size() > 2) {
                    new_line.insert(new_line.end(), std::next(line.cbegin()), std::prev(line.cend()));
                }
            }
            if (not to_added) {
                new_line.emplace_back(to);
                if (not geometry::equals_points(line[to_index], to)) {
                    to_index--;
                }
            }

            // if line is still no line, delete it
            if (new_line.size() < 2) {
//                std::cout << std::setprecision(20)
//                          << "line: " << boost::geometry::wkt(line)
//                          << ", from_distance: " << from_distance << ", to_distance: " << to_distance
//                          << ", current_distance: " << current_distance << std::endl;
                new_line.clear();
            }

            new_line.shrink_to_fit();
        }

        // hammer for correcting edge cases
        if (new_line.empty() and use_from_point and use_to_point and
            not geometry::equals_points(from_point, to_point)) {
            if (from_distance == to_distance or from_distance > to_distance) {
                // distances were too close at start or end
                new_line.reserve(2);
                new_line.emplace_back(from_point);
                new_line.emplace_back(to_point);
                return RichLine{std::move(new_line)};
            }
        }

        // copy remaining data
        std::vector<rich_segment_type> new_rich_segments;

        if (new_line.size() >= 2 and to_index >= 0) {
            new_rich_segments.reserve(new_line.size() - 1);

            if (from_custom_index < from_index) {
                new_rich_segments.emplace_back(rich_segment_type{segment_type{new_line[0], new_line[1]}});
            }

            if (from_index < to_index) {
                std::copy(std::next(rich_segments.cbegin(), from_index),
                          std::next(rich_segments.cbegin(), to_index),
                          std::back_inserter(new_rich_segments));
            }

            if (to_custom_index > to_index and from_index <= to_index) {
                new_rich_segments.emplace_back(
                        rich_segment_type{segment_type{new_line[new_line.size() - 2], new_line[new_line.size() - 1]}});
            }
        }

        return RichLine{std::move(new_rich_segments)};
    }

    template<typename RichLine>
    RichLine substring_from(const RichLine &rich_line,
                            typename rich_line_traits<RichLine>::length_type from_distance,
                            const typename rich_line_traits<RichLine>::point_type &from_point = {},
                            bool use_from_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        if (rich_line.rich_segments().size() >= 1) {
            return substring(rich_line, from_distance, rich_line.length(),
                             from_point, rich_line.rich_segments().back().second(), use_from_point, true);
        } else {
            return RichLine{};
        }
    }

    template<typename RichLine>
    RichLine substring_to(const RichLine &rich_line,
                          typename rich_line_traits<RichLine>::length_type to_distance,
                          const typename rich_line_traits<RichLine>::point_type &to_point = {},
                          bool use_to_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;
        using length_type = typename rich_line_traits<RichLine>::length_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        if (rich_line.rich_segments().size() >= 1) {
            return substring(rich_line, geometry::default_float_type<length_type>::v0, to_distance,
                             rich_line.rich_segments().front().first(), to_point, true, use_to_point);
        } else {
            return RichLine{};
        }
    }

    template<typename RichLine>
    std::pair<typename rich_line_traits<RichLine>::length_type, typename rich_line_traits<RichLine>::length_type>
    distances(const RichLine &rich_line,
              const typename rich_line_traits<RichLine>::point_type &from,
              const typename rich_line_traits<RichLine>::point_type &to,
              bool calculate_from = true, bool calculate_to = true,
              bool use_from_point = false, bool use_to_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;
        using segment_type = typename rich_line_traits<RichLine>::segment_type;
        using length_type = typename rich_line_traits<RichLine>::length_type;
        using distance_type = typename rich_line_traits<RichLine>::distance_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        // find nearest segments with projections
        std::size_t from_nearest_segment, to_nearest_segment;
        distance_type from_smallest_distance = std::numeric_limits<distance_type>::infinity(),
                to_smallest_distance = std::numeric_limits<distance_type>::infinity();
        point_type from_projection, to_projection;

        const auto &rich_segments = rich_line.rich_segments();

        if (calculate_from) {
            // search from_projection from start
            for (std::size_t i = 0; i < rich_segments.size(); ++i) {
                const auto &line_segment = rich_segments[i];
                if (geometry::equals_points(from, line_segment.first())) {
                    from_smallest_distance = geometry::default_float_type<distance_type>::v0;
                    from_nearest_segment = i;
                    from_projection = from;
                    break;
                } else {
                    segment_type projection;
                    boost::geometry::closest_points(from, line_segment.segment(), projection);
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

            const auto &line_segment = rich_segments[from_nearest_segment];
            const auto distance_from = geometry::distance(line_segment.first(), from);
            if (distance_from > line_segment.length() and from_nearest_segment + 1 < rich_segments.size()) {
                from_nearest_segment++;
            }
        }

        if (calculate_to) {
            // search to_projection from end
            for (std::int64_t i = rich_segments.size() - 1; i >= 0; --i) {
                const auto &line_segment = rich_segments[i];
                if (geometry::equals_points(to, line_segment.second())) {
                    to_smallest_distance = geometry::default_float_type<distance_type>::v0;
                    to_nearest_segment = i;
                    to_projection = to;
                    break;
                } else {
                    segment_type projection;
                    boost::geometry::closest_points(to, line_segment.segment(), projection);
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

            const auto &line_segment = rich_segments[to_nearest_segment];
            const auto distance_to = geometry::distance(line_segment.second(), to);
            if (distance_to > line_segment.length() and to_nearest_segment > 0) {
                to_nearest_segment--;
            }
        }

        // calculate distances along line
        bool found_from_distance = !calculate_from;
        bool found_to_distance = !calculate_to;
        length_type from_distance = default_float_type<length_type>::v0;
        length_type to_distance = default_float_type<length_type>::v0;
        for (std::size_t i = 0; i < rich_segments.size(); ++i) {
            const auto distance = rich_segments[i].length();

            if (calculate_from) {
                if (i < from_nearest_segment) {
                    from_distance += distance;
                } else if (i == from_nearest_segment) {
                    if (geometry::equals_points(rich_segments[i].second(), from_projection)) {
                        from_distance += distance;
                        found_from_distance = true;
                    } else {
                        const auto partial_distance = geometry::distance(rich_segments[i].first(), from_projection);
                        from_distance += partial_distance;
                        found_from_distance = true;
                    }
                }
            }

            if (calculate_to) {
                if (i < to_nearest_segment) {
                    to_distance += distance;
                } else if (i == to_nearest_segment) {
                    if (geometry::equals_points(rich_segments[i].second(), to_projection)) {
                        to_distance += distance;
                        found_to_distance = true;
                    } else {
                        const auto partial_distance = geometry::distance(rich_segments[i].first(), to_projection);
                        to_distance += partial_distance;
                        found_to_distance = true;
                    }
                }
            }

            if (found_from_distance and found_to_distance) {
                break;
            }
        }

//        std::cout << std::setprecision(10)
//                  << "from_i: " << from_nearest_segment << ", " << boost::geometry::wkt(from_projection) << ", "
//                  << from_distance << ", to_i: " << to_nearest_segment << ", " << boost::geometry::wkt(to_projection)
//                  << ", " << to_distance << std::endl;

        return std::pair{from_distance, to_distance};
    }

    template<typename RichLine>
    typename rich_line_traits<RichLine>::length_type
    distance_from(const RichLine &rich_line, const typename rich_line_traits<RichLine>::point_type &from,
                  bool use_from_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        const auto distances_pair = distances(rich_line, from, {}, true, false, use_from_point, false);
        return distances_pair.first;
    }

    template<typename RichLine>
    typename rich_line_traits<RichLine>::length_type
    distance_to(const RichLine &rich_line, const typename rich_line_traits<RichLine>::point_type &to,
                bool use_to_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        const auto distances_pair = distances(rich_line, {}, to, false, true, false, use_to_point);
        return distances_pair.second;
    }

    template<typename RichLine>
    RichLine substring_points(const RichLine &rich_line,
                              const typename rich_line_traits<RichLine>::point_type &from,
                              const typename rich_line_traits<RichLine>::point_type &to,
                              bool use_from_point = false, bool use_to_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        const auto distances_pair = distances(rich_line, from, to, true, true, use_from_point, use_to_point);
        return substring(rich_line, distances_pair.first, distances_pair.second,
                         from, to, use_from_point, use_to_point);
    }

    template<typename RichLine>
    RichLine
    substring_point_from(const RichLine &rich_line, const typename rich_line_traits<RichLine>::point_type &from,
                         bool use_from_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        const auto distance = distance_from(rich_line, from, use_from_point);
        return substring_from(rich_line, distance, from, use_from_point);
    }

    template<typename RichLine>
    RichLine substring_point_to(const RichLine &rich_line, const typename rich_line_traits<RichLine>::point_type &to,
                                bool use_to_point = false) {
        using point_type = typename rich_line_traits<RichLine>::point_type;
        using line_type = typename rich_line_traits<RichLine>::line_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<point_type>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        const auto distance = distance_to(rich_line, to, use_to_point);
        return substring_to(rich_line, distance, to, use_to_point);
    }

}

#endif //MAP_MATCHING_2_SUBSTRING_HPP
