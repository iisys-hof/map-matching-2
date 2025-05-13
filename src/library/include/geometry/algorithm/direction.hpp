// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_DIRECTION_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_DIRECTION_HPP

#include "util/concepts.hpp"

#include "geometry/common.hpp"
#include "geometry/traits.hpp"

#include "equals.hpp"
#include "azimuth.hpp"

namespace map_matching_2::geometry {

    template<typename Float>
    [[nodiscard]] constexpr Float direction_deg(const Float azimuth_deg_a, const Float azimuth_deg_b) {
        return angle_diff(azimuth_deg_a, azimuth_deg_b);
    }

    template<typename Point>
    [[nodiscard]] typename data<Point>::angle_type
    direction_deg(const Point &a, const Point &b, const Point &c) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        return angle_diff(azimuth_deg(a, b), azimuth_deg(b, c));
    }

    template<typename Line>
    [[nodiscard]] typename data<Line>::angle_type
    directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename data<Line>::angle_type>::v0;
        for (std::size_t i = 0, j = 1, k = 2; k < line.size(); ++i, ++j, ++k) {
            directions += direction_deg(line[i], line[j], line[k]);
        }
        return directions;
    }

    template<typename Line>
    [[nodiscard]] typename data<Line>::angle_type
    absolute_directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename data<Line>::angle_type>::v0;
        for (std::size_t i = 0, j = 1, k = 2; k < line.size(); ++i, ++j, ++k) {
            directions += std::fabs(direction_deg(line[i], line[j], line[k]));
        }
        return directions;
    }

    template<typename Line>
    [[nodiscard]] std::pair<typename data<Line>::angle_type, typename data<Line>::angle_type>
    combined_directions_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        auto directions = default_float_type<typename data<Line>::angle_type>::v0;
        auto absolute_directions = default_float_type<typename data<Line>::angle_type>::v0;
        for (std::size_t i = 0, j = 1, k = 2; k < line.size(); ++i, ++j, ++k) {
            const auto direction = direction_deg(line[i], line[j], line[k]);
            directions += direction;
            absolute_directions += std::fabs(direction);
        }
        return std::pair{directions, absolute_directions};
    }

    template<template<typename, typename> typename Container,
        typename Segment,
        template<typename> typename Allocator>
    [[nodiscard]] Container<typename data<Segment>::angle_type,
        Allocator<typename data<Segment>::angle_type>>
    segments_directions_deg(const Container<Segment, Allocator<Segment>> &segments,
            const Container<typename data<Segment>::angle_type,
                Allocator<typename data<Segment>::angle_type>> &segments_azimuths = {}) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename data<Segment>::angle_type;
        using allocator_type = Allocator<angle_type>;

        Container<angle_type, allocator_type> directions;
        if constexpr (util::has_reserve<Container<angle_type, allocator_type>>) {
            directions.reserve(segments.size() - 1);
        }
        if (segments_azimuths.empty()) {
            for (std::size_t i = 0, j = 1; j < segments.size(); ++i, ++j) {
                const auto &a = segments[i];
                const auto &b = segments[j];
                assert(geometry::equals_points(a.second, b.first));
                directions.emplace_back(direction_deg(a.first, a.second, b.second));
            }
        } else {
            for (std::size_t i = 0, j = 1; j < segments_azimuths.size(); ++i, ++j) {
                directions.emplace_back(direction_deg(segments_azimuths[i], segments_azimuths[j]));
            }
        }
        return directions;
    }

    template<template<typename, typename> typename Container,
        typename Segment,
        template<typename> typename Allocator>
    [[nodiscard]] Container<typename data<Segment>::angle_type,
        Allocator<typename data<Segment>::angle_type>>
    segments_absolute_directions_deg(const Container<Segment, Allocator<Segment>> &segments,
            const Container<typename data<Segment>::angle_type,
                Allocator<typename data<Segment>::angle_type>> &segments_azimuths = {}) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename data<Segment>::angle_type;
        using allocator_type = Allocator<angle_type>;

        Container<angle_type, allocator_type> directions;
        if constexpr (util::has_reserve<Container<angle_type, allocator_type>>) {
            directions.reserve(segments.size() - 1);
        }
        if (segments_azimuths.empty()) {
            for (std::size_t i = 0, j = 1; j < segments.size(); ++i, ++j) {
                const auto &a = segments[i];
                const auto &b = segments[j];
                assert(geometry::equals_points(a.second, b.first));
                directions.emplace_back(std::fabs(direction_deg(a.first, a.second, b.second)));
            }
        } else {
            for (std::size_t i = 0, j = 1; j < segments_azimuths.size(); ++i, ++j) {
                directions.emplace_back(std::fabs(direction_deg(segments_azimuths[i], segments_azimuths[j])));
            }
        }
        return directions;
    }

    template<template<typename, typename> typename Container,
        typename Segment,
        template<typename> typename Allocator>
    [[nodiscard]] std::pair<Container<typename data<Segment>::angle_type,
            Allocator<typename data<Segment>::angle_type>>,
        Container<typename data<Segment>::angle_type,
            Allocator<typename data<Segment>::angle_type>>>
    segments_combined_directions_deg(const Container<Segment, Allocator<Segment>> &segments,
            const Container<typename data<Segment>::angle_type,
                Allocator<typename data<Segment>::angle_type>> &segments_azimuths = {}) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));
        assert(segments_azimuths.empty() or segments_azimuths.size() == segments.size());

        using angle_type = typename data<Segment>::angle_type;
        using allocator_type = Allocator<angle_type>;

        Container<angle_type, allocator_type> directions;
        Container<angle_type, allocator_type> absolute_directions;
        if constexpr (util::has_reserve<Container<angle_type, allocator_type>>) {
            directions.reserve(segments.size() - 1);
            absolute_directions.reserve(segments.size() - 1);
        }
        if (segments_azimuths.empty()) {
            for (std::size_t i = 0, j = 1; j < segments.size(); ++i, ++j) {
                const auto &a = segments[i];
                const auto &b = segments[j];
                assert(geometry::equals_points(a.second, b.first));
                const auto direction = direction_deg(a.first, a.second, b.second);
                directions.emplace_back(direction);
                absolute_directions.emplace_back(std::fabs(direction));
            }
        } else {
            for (std::size_t i = 0, j = 1; j < segments_azimuths.size(); ++i, ++j) {
                const auto direction = direction_deg(segments_azimuths[i], segments_azimuths[j]);
                directions.emplace_back(direction);
                absolute_directions.emplace_back(std::fabs(direction));
            }
        }
        return std::pair{std::move(directions), std::move(absolute_directions)};
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_DIRECTION_HPP
