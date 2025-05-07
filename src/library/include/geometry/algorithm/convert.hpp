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
