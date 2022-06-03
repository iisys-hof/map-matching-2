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

#ifndef MAP_MATCHING_2_RICH_SEGMENT_HPP
#define MAP_MATCHING_2_RICH_SEGMENT_HPP

#include <atomic>
#include <mutex>
#include <utility>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/utility.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace map_matching_2::geometry {

    template<typename Segment>
    class rich_segment {

    public:
        using point_type = typename boost::geometry::point_type<Segment>::type;
        using line_type = boost::geometry::model::linestring<point_type>;
        using segment_type = boost::geometry::model::segment<point_type>;
        using distance_type = typename boost::geometry::default_distance_result<point_type, point_type>::type;
        using length_type = typename boost::geometry::default_length_result<Segment>::type;
        using angle_type = typename boost::geometry::coordinate_type<Segment>::type;

        rich_segment();

        explicit rich_segment(Segment segment);

        rich_segment(point_type first, point_type second);

        rich_segment(Segment segment, length_type length, angle_type azimuth);

        ~rich_segment() = default;

        rich_segment(const rich_segment &other);

        rich_segment(rich_segment &&other) noexcept;

        rich_segment &operator=(const rich_segment &other);

        rich_segment &operator=(rich_segment &&other) noexcept;

        [[nodiscard]] bool has_length() const;

        [[nodiscard]] length_type length() const;

        [[nodiscard]] bool has_azimuth() const;

        [[nodiscard]] angle_type azimuth() const;

        [[nodiscard]] const segment_type &segment() const;

        [[nodiscard]] line_type line() const;

        template<template<typename> class ContainerA,
                template<typename> class ContainerB>
        [[nodiscard]] static ContainerA<rich_segment>
        points2rich_segments(const ContainerB<point_type> &points);

        template<template<typename> class ContainerA,
                template<typename> class ContainerB>
        [[nodiscard]] static ContainerA<point_type>
        rich_segments2points(const ContainerB<rich_segment> &rich_segments);

        template<template<typename> class Container>
        [[nodiscard]] static Container<rich_segment>
        line2rich_segments(const line_type &line);

        template<template<typename> class Container>
        [[nodiscard]] static line_type
        rich_segments2line(const Container<rich_segment> &rich_segments);

        static void simplify(std::list<rich_segment> &segments, bool retain_reversals = true, double tolerance = 1e-3,
                             double reverse_tolerance = 1e-3);

        [[nodiscard]] std::string wkt() const;

        [[nodiscard]] std::string str() const;

        template<typename SegmentT>
        friend bool operator==(const rich_segment<SegmentT> &left, const rich_segment<SegmentT> &right);

        template<typename SegmentT>
        friend bool operator!=(const rich_segment<SegmentT> &left, const rich_segment<SegmentT> &right);

        template<typename SegmentT>
        friend std::ostream &operator<<(std::ostream &out, const rich_segment<SegmentT> &rich_segment);

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _segment;
        }

    private:
        mutable std::mutex _mutex_length, _mutex_azimuth;
        mutable std::atomic<bool> _computed_length, _computed_azimuth;
        mutable bool _has_length, _has_azimuth;
        mutable length_type _length;
        mutable angle_type _azimuth;
        segment_type _segment;

        void _transfer(const rich_segment &other);

        void _compute_length() const;

        void _compute_azimuth() const;

        static void _simplify(std::list<rich_segment> &segments, typename std::list<rich_segment>::iterator begin,
                              typename std::list<rich_segment>::iterator end, double tolerance);

    };

}

namespace boost::serialization {

    template<typename Archive, typename Point>
    void serialize(Archive &ar, boost::geometry::model::segment<Point> &segment, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP((static_cast<std::pair<Point, Point> &>(segment)));
    }

}

#endif //MAP_MATCHING_2_RICH_SEGMENT_HPP
