// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_CONVERT_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_CONVERT_HPP

#include "util/concepts.hpp"

#include "geometry/point.hpp"
#include "geometry/types.hpp"
#include "geometry/traits.hpp"

namespace map_matching_2::geometry {

    template<typename Point, typename PointNew>
    [[nodiscard]] constexpr PointNew convert_point(const Point &point) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<PointNew>));

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            return PointNew{
                    static_cast<typename data<PointNew>::coordinate_type>(boost::geometry::get<0>(point))
            };
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            return PointNew{
                    static_cast<typename data<PointNew>::coordinate_type>(boost::geometry::get<0>(point)),
                    static_cast<typename data<PointNew>::coordinate_type>(boost::geometry::get<1>(point))
            };
        } else {
            return PointNew{
                    static_cast<typename data<PointNew>::coordinate_type>(boost::geometry::get<0>(point)),
                    static_cast<typename data<PointNew>::coordinate_type>(boost::geometry::get<1>(point)),
                    static_cast<typename data<PointNew>::coordinate_type>(boost::geometry::get<2>(point))
            };
        }
    }

    template<typename Line, typename LineNew>
    [[nodiscard]] LineNew convert_line(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<LineNew>));

        using point_type = point_type_t<Line>;
        using point_type_new = point_type_t<LineNew>;

        LineNew line_new;
        line_new.reserve(line.size());
        for (const auto &point : line) {
            line_new.emplace_back(convert_point<point_type, point_type_new>(point));
        }
        return line_new;
    }

    template<typename ContainerOut, typename ContainerIn>
    [[nodiscard]] ContainerOut convert_container_types(
            const ContainerIn &input, const typename ContainerOut::allocator_type &allocator) requires
        (not std::same_as<ContainerOut, ContainerIn>) {
        ContainerOut output(allocator);
        if constexpr (util::has_reserve<ContainerOut>) {
            output.reserve(input.size());
        }
        for (const typename ContainerIn::value_type &in : input) {
            if constexpr (std::default_initializable<typename ContainerOut::allocator_type>) {
                output.emplace_back(typename ContainerOut::value_type{in});
            } else {
                output.emplace_back(typename ContainerOut::value_type{in, allocator});
            }
        }
        output.shrink_to_fit();
        return output;
    }

    template<typename ContainerOut, typename ContainerIn>
    [[nodiscard]] ContainerOut convert_container_types(const ContainerIn &input) requires
        (std::default_initializable<typename ContainerOut::allocator_type>) {
        return convert_container_types<ContainerOut, ContainerIn>(input, typename ContainerOut::allocator_type{});
    }

    template<template<typename, typename> typename Container,
        typename Line,
        template<typename> typename Allocator>
    [[nodiscard]] Container<typename models<point_type_t<Line>>::segment_type,
        Allocator<typename models<point_type_t<Line>>::segment_type>>
    line2segments(const Line &line) {
        using segment_type = typename models<point_type_t<Line>>::segment_type;
        using allocator_type = Allocator<segment_type>;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<segment_type>));

        Container<segment_type, allocator_type> segments;
        if (line.size() >= 2) {
            if constexpr (util::has_reserve<Container<segment_type, allocator_type>>) {
                segments.reserve(line.size() - 1);
            }
            for (std::size_t i = 0; i < line.size() - 1; ++i) {
                segments.emplace_back(segment_type{line[i], line[i + 1]});
            }
        }
        return segments;
    }

    template<typename Segment,
        template<typename, typename> typename Container,
        template<typename> typename Allocator>
    [[nodiscard]] typename models<point_type_t<Segment>>::line_type
    segments2line(const Container<Segment, Allocator<Segment>> &segments) {
        using line_type = typename models<point_type_t<Segment>>::line_type;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        line_type line;
        if (not segments.empty()) {
            line.reserve(segments.size() + 1);
            for (const Segment &segment : segments) {
                if (line.empty()) {
                    line.emplace_back(segment.first);
                }
                line.emplace_back(segment.second);
            }
        }
        return line;
    }

    template<typename Measurement,
        typename Line = typename models<typename Measurement::point_type>::line_type>
    [[nodiscard]] Line measurements2line(const std::vector<Measurement> &measurements) {
        using measurement_type = Measurement;
        using line_type = Line;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        line_type line;
        line.reserve(measurements.size());
        for (const measurement_type &measurement : measurements) {
            line.emplace_back(measurement.point);
        }
        return line;
    }

    template<typename Measurement,
        typename Line = typename models<typename Measurement::point_type>::line_type>
    [[nodiscard]] std::vector<Measurement> line2measurements(const Line &line) {
        using measurement_type = Measurement;
        using line_type = Line;
        using point_type = point_type_t<line_type>;

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        std::vector<measurement_type> measurements;
        measurements.reserve(line.size());
        std::uint64_t _timestamp = 0;
        for (const point_type &_point : line) {
            measurements.emplace_back(measurement_type{_timestamp++, _point});
        }
        return measurements;
    }

    // template<typename MultiTrack>
    // [[nodiscard]] static typename MultiTrack::multi_line_type
    // tracks2multi_line(const std::vector<typename MultiTrack::track_type> &tracks) {
    //     using multi_line_type = typename MultiTrack::multi_line_type;
    //     using track_type = typename MultiTrack::track_type;
    //
    //     multi_line_type multi_line;
    //     multi_line.reserve(tracks.size());
    //     for (const track_type &track : tracks) {
    //         if (not track.empty()) {
    //             multi_line.emplace_back(track.line());
    //         }
    //     }
    //     return multi_line;
    // }
    //
    // template<typename MultiTrack, template<typename> class Container>
    // [[nodiscard]] static Container<typename MultiTrack::rich_line_type>
    // tracks2rich_lines(const std::vector<typename MultiTrack::track_type> &tracks) {
    //     using rich_line_type = typename MultiTrack::rich_line_type;
    //
    //     Container<rich_line_type> rich_lines;
    //     if (not tracks.empty()) {
    //         if constexpr (util::has_reserve<Container<rich_line_type>>) {
    //             rich_lines.reserve(tracks.size());
    //         }
    //         for (const auto &track : tracks) {
    //             if (not track.empty()) {
    //                 rich_lines.emplace_back(rich_line_type{track});
    //             }
    //         }
    //     }
    //     return rich_lines;
    // }
    //
    // template<typename MultiTrack, template<typename> class Container>
    // [[nodiscard]] static std::vector<typename MultiTrack::track_type>
    // multi_line2tracks(const std::string &id, const typename MultiTrack::multi_line_type &multi_line) {
    //     using track_type = typename MultiTrack::track_type;
    //
    //     Container<track_type> tracks;
    //     if (not multi_line.empty()) {
    //         if constexpr (util::has_reserve<Container<track_type>>) {
    //             tracks.reserve(multi_line.size());
    //         }
    //         for (const auto &line : multi_line) {
    //             if (not line.empty()) {
    //                 tracks.emplace_back(track_type{id, line});
    //             }
    //         }
    //     }
    //     return tracks;
    // }


    // template<typename RichSegment,
    //     template<typename, typename> class ContainerA = std::vector,
    //     template<typename, typename> class ContainerB = std::vector,
    //     template<typename> class AllocatorA = std::allocator,
    //     template<typename> class AllocatorB = std::allocator>
    // [[nodiscard]] static ContainerA<RichSegment, AllocatorA<RichSegment>>
    // points2rich_segments(const ContainerB<typename rich_segment_traits<RichSegment>::point_type,
    //     AllocatorB<typename rich_segment_traits<RichSegment>::point_type>> &points) {
    //     using segment_type = typename rich_segment_traits<RichSegment>::segment_type;
    //
    //     ContainerA<RichSegment, AllocatorA<RichSegment>> rich_segments;
    //     if (points.size() >= 2) {
    //         if constexpr (util::has_reserve<ContainerA<RichSegment, AllocatorA<RichSegment>>>) {
    //             rich_segments.reserve(points.size() - 1);
    //         }
    //         for (auto it = points.cbegin(); std::next(it) != points.cend(); it++) {
    //             rich_segments.emplace_back(RichSegment{segment_type{*it, *std::next(it)}});
    //         }
    //     }
    //     return rich_segments;
    // }
    //
    // template<typename RichSegment,
    //     template<typename, typename> class ContainerA = std::vector,
    //     template<typename, typename> class ContainerB = std::vector,
    //     template<typename> class AllocatorA = std::allocator,
    //     template<typename> class AllocatorB = std::allocator>
    // [[nodiscard]] static ContainerA<typename rich_segment_traits<RichSegment>::point_type,
    //     AllocatorA<typename rich_segment_traits<RichSegment>::point_type>>
    // rich_segments2points(const ContainerB<RichSegment, AllocatorB<RichSegment>> &rich_segments) {
    //     using point_type = typename rich_segment_traits<RichSegment>::point_type;
    //     using segment_type = typename rich_segment_traits<RichSegment>::segment_type;
    //
    //     ContainerA<point_type, AllocatorA<point_type>> points;
    //     if (not rich_segments.empty()) {
    //         if constexpr (util::has_reserve<ContainerA<point_type, AllocatorA<point_type>>>) {
    //             points.reserve(rich_segments.size() + 1);
    //         }
    //         for (const auto &rich_segment : rich_segments) {
    //             if (points.empty()) {
    //                 points.emplace_back(rich_segment.first());
    //             }
    //             points.emplace_back(rich_segment.second());
    //         }
    //     }
    //     return points;
    // }

    // template<typename MultiRichLine,
    //     template<typename, typename> class Container = std::vector,
    //     template<typename> class Allocator = std::allocator>
    // [[nodiscard]] static typename multi_rich_line_traits<MultiRichLine>::template multi_line_type<
    //     Container, Container, Allocator, Allocator>
    // lines2multi_line(const std::vector<typename multi_rich_line_traits<MultiRichLine>::template line_type<
    //     Container, Allocator>> &lines) {
    //     using multi_line_type = typename multi_rich_line_traits<MultiRichLine>::template multi_line_type<
    //         Container, Container, Allocator, Allocator>;
    //
    //     multi_line_type multi_line;
    //     multi_line.reserve(lines.size());
    //     for (const auto &line : lines) {
    //         if (not line.empty()) {
    //             multi_line.emplace_back(line);
    //         }
    //     }
    //     return multi_line;
    // }
    //
    // template<typename MultiRichLine,
    //     template<typename, typename> class Container = std::vector,
    //     template<typename> class Allocator = std::allocator>
    // [[nodiscard]] static typename multi_rich_line_traits<MultiRichLine>::template multi_line_type<
    //     Container, Container, Allocator, Allocator>
    // rich_lines2multi_line(
    //         const std::vector<typename multi_rich_line_traits<MultiRichLine>::rich_line_type> &rich_lines) {
    //     using multi_line_type = typename multi_rich_line_traits<MultiRichLine>::template multi_line_type<
    //         Container, Container, Allocator, Allocator>;
    //
    //     multi_line_type multi_line;
    //     multi_line.reserve(rich_lines.size());
    //     for (const auto &rich_line : rich_lines) {
    //         if (not rich_line.empty()) {
    //             multi_line.emplace_back(rich_line.line());
    //         }
    //     }
    //     return multi_line;
    // }
    //
    // template<typename MultiRichLine,
    //     template<typename, typename> class Container = std::vector,
    //     template<typename> class Allocator = std::allocator>
    // [[nodiscard]] static Container<typename multi_rich_line_traits<MultiRichLine>::rich_line_type,
    //     Allocator<typename multi_rich_line_traits<MultiRichLine>::rich_line_type>>
    // multi_line2rich_lines(const typename multi_rich_line_traits<MultiRichLine>::template multi_line_type<
    //     Container, Container, Allocator, Allocator> &multi_lines) {
    //     using rich_line_type = typename multi_rich_line_traits<MultiRichLine>::rich_line_type;
    //
    //     Container<rich_line_type, Allocator<rich_line_type>> rich_lines;
    //     if (not multi_lines.empty()) {
    //         if constexpr (util::has_reserve<Container<rich_line_type, Allocator<rich_line_type>>>) {
    //             rich_lines.reserve(multi_lines.size());
    //         }
    //         for (const auto &line : multi_lines) {
    //             if (not line.empty()) {
    //                 rich_lines.emplace_back(rich_line_type{line});
    //             }
    //         }
    //     }
    //     return rich_lines;
    // }

    template<typename RichSegmentContainer, typename Line>
    [[nodiscard]] RichSegmentContainer line2rich_segments(
            const Line &line, const typename RichSegmentContainer::allocator_type &allocator) {
        using rich_segment_type = typename RichSegmentContainer::value_type;

        RichSegmentContainer rich_segments(allocator);
        if (line.size() >= 2) {
            if constexpr (util::has_reserve<RichSegmentContainer>) {
                rich_segments.reserve(line.size() - 1);
            }
            for (std::size_t i = 0; i < line.size() - 1; ++i) {
                rich_segments.emplace_back(rich_segment_type{line[i], line[i + 1]});
            }
            if constexpr (util::has_capacity<RichSegmentContainer>) {
                assert(rich_segments.capacity() == line.size() - 1);
            }
        }
        return rich_segments;
    }

    template<typename RichSegmentContainer, typename Line>
    [[nodiscard]] RichSegmentContainer line2rich_segments(const Line &line) requires
        (std::default_initializable<typename RichSegmentContainer::allocator_type>) {
        return line2rich_segments<RichSegmentContainer, Line>(line, typename RichSegmentContainer::allocator_type{});
    }

    template<typename Line, typename RichSegmentContainer>
    [[nodiscard]] Line rich_segments2line(const RichSegmentContainer &rich_segments) {
        Line line;
        if (not rich_segments.empty()) {
            line.reserve(rich_segments.size() + 1);
            for (const auto &rich_segment : rich_segments) {
                if (line.empty()) {
                    line.emplace_back(rich_segment.first());
                }
                line.emplace_back(rich_segment.second());
            }
            assert(line.capacity() == rich_segments.size() + 1);
        }
        return line;
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_CONVERT_HPP
