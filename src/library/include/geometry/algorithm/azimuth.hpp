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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_AZIMUTH_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_AZIMUTH_HPP

#include "util/concepts.hpp"

#include "geometry/common.hpp"
#include "geometry/concepts.hpp"
#include "geometry/cache.hpp"
#include "geometry/traits.hpp"

namespace map_matching_2::geometry {

    template<typename Point>
    [[nodiscard]] typename data<Point>::angle_type
    azimuth_rad(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        if (use_azimuth_cache) {
            if (not azimuth_cache) {
                reset_azimuth_cache();
            }

            const auto key = _cache_key_type{
                    boost::geometry::get<0>(a), boost::geometry::get<1>(a),
                    boost::geometry::get<0>(b), boost::geometry::get<1>(b)
            };
            const auto search = azimuth_cache->find(key);
            const bool exists = search != azimuth_cache->end();
            if (exists) {
                return search->second;
            } else {
                const auto azimuth = boost::geometry::azimuth(a, b);
                azimuth_cache->emplace(key, azimuth);
                return azimuth;
            }
        } else {
            return boost::geometry::azimuth(a, b);
        }
    }

    template<typename Point>
    [[nodiscard]] typename data<Point>::angle_type
    azimuth_deg(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        return rad2deg(azimuth_rad(a, b));
    }

    template<typename Line>
    [[nodiscard]] typename data<Line>::angle_type
    azimuth_line_deg(const Line &line) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));
        if (line.size() >= 2) {
            return azimuth_deg(line.front(), line.back());
        }
        return default_float_type<typename data<Line>::angle_type>::v0;
    }

    template<typename Segment>
    [[nodiscard]] typename data<Segment>::angle_type
    azimuth_segment_deg(const Segment &segment) {
        if constexpr (geometry::is_segment<Segment> or geometry::is_referring_segment<Segment>) {
            return azimuth_deg(segment.first, segment.second);
        } else if constexpr (geometry::is_pointing_segment<Segment>) {
            return azimuth_deg(*(segment.first), *(segment.second));
        } else {
            static_assert(util::dependent_false_v<Segment>, "invalid segment type");
            throw std::invalid_argument{"invalid segment type"};
        }
    }

    template<template<typename, typename> typename Container,
        typename Segment,
        template<typename> typename Allocator>
    [[nodiscard]] Container<typename data<Segment>::angle_type,
        Allocator<typename data<Segment>::angle_type>>
    segments_azimuths_deg(const Container<Segment, Allocator<Segment>> &segments) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using angle_type = typename data<Segment>::angle_type;
        using allocator_type = Allocator<angle_type>;

        Container<angle_type, allocator_type> azimuths;
        if constexpr (util::has_reserve<Container<angle_type, allocator_type>>) {
            azimuths.reserve(segments.size());
        }
        for (const auto &segment : segments) {
            azimuths.emplace_back(azimuth_segment_deg(segment));
        }
        return azimuths;
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_AZIMUTH_HPP
