// Copyright (C) 2022 Adrian Wöltche
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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE map_matching_2_tests

#include <boost/test/unit_test.hpp>

#include <geometry/rich_segment.hpp>
#include <geometry/rich_line.hpp>
#include <geometry/multi_rich_line.hpp>

using point_type = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;

namespace map_matching_2::geometry {
    boost::thread_specific_ptr<map_matching_2::geometry::distance_cache_type> distance_cache;
    boost::thread_specific_ptr<map_matching_2::geometry::azimuth_cache_type> azimuth_cache;
}

template<typename RichSegment>
void test_rich_segment(const RichSegment &rich_segment, const point_type &a, const point_type &b, bool reference) {
    BOOST_TEST_MESSAGE("sizeof: " << sizeof(rich_segment));
    BOOST_CHECK(boost::geometry::equals(rich_segment.first(), a));
    BOOST_CHECK(boost::geometry::equals(rich_segment.second(), b));

    if (reference) {
        BOOST_CHECK_EQUAL(&rich_segment.first(), &a);
        BOOST_CHECK_EQUAL(&rich_segment.second(), &b);
    } else {
        BOOST_CHECK_NE(&rich_segment.first(), &a);
        BOOST_CHECK_NE(&rich_segment.second(), &b);
    }

    if (not boost::geometry::equals(a, b)) {
        BOOST_CHECK(rich_segment.has_length());
        BOOST_CHECK(rich_segment.has_azimuth());
        BOOST_CHECK_CLOSE_FRACTION(rich_segment.length(), boost::geometry::distance(a, b), 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(rich_segment.azimuth(), boost::geometry::azimuth(a, b), 1e-6);
    } else {
        BOOST_CHECK(not rich_segment.has_length());
        BOOST_CHECK(not rich_segment.has_azimuth());
    }
}

template<typename RichSegmentA, typename RichSegmentB>
void test_rich_segment_equality(const RichSegmentA &a, const RichSegmentB &b) {
    if constexpr (std::is_same_v<RichSegmentA, RichSegmentB>) {
        BOOST_CHECK_EQUAL(a, b);
    }
    BOOST_CHECK(map_matching_2::geometry::equals_points(a.first(), b.first()));
    BOOST_CHECK(map_matching_2::geometry::equals_points(a.second(), b.second()));
    BOOST_CHECK_EQUAL(a.has_length(), b.has_length());
    BOOST_CHECK_EQUAL(a.has_azimuth(), b.has_azimuth());
    BOOST_CHECK_EQUAL(a.length(), b.length());
    BOOST_CHECK_EQUAL(a.azimuth(), b.azimuth());
}

template<typename RichSegment>
void test_rich_segment_type(bool reference) {
    using rich_segment_type = RichSegment;
    using segment_type = typename map_matching_2::geometry::rich_segment_traits<rich_segment_type>::segment_type;

    point_type point_1{0.0, 0.0};
    point_type point_2{0.0, 1.0};

    if constexpr (std::is_default_constructible_v<segment_type>) {
        rich_segment_type rich_segment_0;
    }

    rich_segment_type rich_segment_1{segment_type{point_1, point_2}};
    test_rich_segment(rich_segment_1, point_1, point_2, reference);

    if constexpr (std::is_copy_constructible_v<rich_segment_type>) {
        rich_segment_type rich_segment{segment_type{point_1, point_2}};
        rich_segment_type rich_segment_2{rich_segment};
        test_rich_segment_equality(rich_segment_2, rich_segment_1);
    }

    if constexpr (std::is_copy_assignable_v<rich_segment_type>) {
        rich_segment_type rich_segment{segment_type{point_1, point_2}};
        rich_segment_type rich_segment_3{segment_type{point_1, point_2}};
        rich_segment_3 = rich_segment;
        test_rich_segment_equality(rich_segment_3, rich_segment_1);
    }

    if constexpr (std::is_move_constructible_v<rich_segment_type>) {
        rich_segment_type rich_segment{segment_type{point_1, point_2}};
        rich_segment_type rich_segment_4{std::move(rich_segment)};
        test_rich_segment_equality(rich_segment_4, rich_segment_1);
    }

    if constexpr (std::is_move_assignable_v<rich_segment_type>) {
        rich_segment_type rich_segment{segment_type{point_1, point_2}};
        rich_segment_type rich_segment_5{segment_type{point_1, point_2}};
        rich_segment_5 = std::move(rich_segment);
        test_rich_segment_equality(rich_segment_5, rich_segment_1);
    }
}

template<typename RichSegmentA, typename RichSegmentB>
void test_rich_segment_conversion() {
    using rich_segment_type_a = RichSegmentA;
    using rich_segment_type_b = RichSegmentB;
    using segment_type_a = typename map_matching_2::geometry::rich_segment_traits<rich_segment_type_a>::segment_type;
    using segment_type_b = typename map_matching_2::geometry::rich_segment_traits<rich_segment_type_b>::segment_type;

    point_type point_1{0.0, 0.0};
    point_type point_2{0.0, 1.0};

    BOOST_TEST_MESSAGE("RichSegmentA: " << typeid(RichSegmentA).name() << ", "
                                        << "RichSegmentB: " << typeid(RichSegmentB).name());

    rich_segment_type_a rich_segment_1{segment_type_a{point_1, point_2}};

    if constexpr (std::is_copy_constructible_v<rich_segment_type_b>) {
        rich_segment_type_a rich_segment{segment_type_a{point_1, point_2}};
        rich_segment_type_b rich_segment_2{rich_segment};
        test_rich_segment_equality(rich_segment_2, rich_segment_1);
    }

    if constexpr (std::is_copy_assignable_v<rich_segment_type_b>) {
        rich_segment_type_a rich_segment{segment_type_a{point_1, point_2}};
        rich_segment_type_b rich_segment_3{segment_type_b{point_1, point_2}};
        rich_segment_3 = rich_segment;
        test_rich_segment_equality(rich_segment_3, rich_segment_1);
    }

    if constexpr (std::is_move_constructible_v<rich_segment_type_b>) {
        rich_segment_type_a rich_segment{segment_type_a{point_1, point_2}};
        rich_segment_type_b rich_segment_4{std::move(rich_segment)};
        test_rich_segment_equality(rich_segment_4, rich_segment_1);
    }

    if constexpr (std::is_move_assignable_v<rich_segment_type_b>) {
        rich_segment_type_a rich_segment{segment_type_a{point_1, point_2}};
        rich_segment_type_b rich_segment_5{segment_type_b{point_1, point_2}};
        rich_segment_5 = std::move(rich_segment);
        test_rich_segment_equality(rich_segment_5, rich_segment_1);
    }
}

template<template<typename> typename RichSegmentA,
        template<typename> typename RichSegmentB>
void test_rich_segment_conversion_types() {
    using segment_type = typename map_matching_2::geometry::models<point_type>::segment_type;
    using referring_segment_type = typename map_matching_2::geometry::models<point_type>::referring_segment_type;
    using pointing_segment_type = typename map_matching_2::geometry::models<point_type>::pointing_segment_type;

    test_rich_segment_conversion<RichSegmentA<segment_type>, RichSegmentB<segment_type>>();
    test_rich_segment_conversion<RichSegmentA<segment_type>, RichSegmentB<referring_segment_type>>();
    test_rich_segment_conversion<RichSegmentA<segment_type>, RichSegmentB<pointing_segment_type>>();
    test_rich_segment_conversion<RichSegmentA<referring_segment_type>, RichSegmentB<segment_type>>();
    test_rich_segment_conversion<RichSegmentA<referring_segment_type>, RichSegmentB<referring_segment_type>>();
    test_rich_segment_conversion<RichSegmentA<referring_segment_type>, RichSegmentB<pointing_segment_type>>();
    test_rich_segment_conversion<RichSegmentA<pointing_segment_type>, RichSegmentB<segment_type>>();
    test_rich_segment_conversion<RichSegmentA<pointing_segment_type>, RichSegmentB<referring_segment_type>>();
    test_rich_segment_conversion<RichSegmentA<pointing_segment_type>, RichSegmentB<pointing_segment_type>>();
}

template<typename RichLine>
void test_rich_line(const RichLine &rich_line, const std::vector<point_type> &points, bool references) {
    using line_type = typename map_matching_2::geometry::rich_line_traits<RichLine>::line_type;

    line_type line{std::cbegin(points), std::cend(points)};

    BOOST_TEST_MESSAGE("sizeof: " << sizeof(rich_line));
    BOOST_CHECK_EQUAL(rich_line.size(), points.size());
    BOOST_CHECK(boost::geometry::equals(rich_line.line(), line));

    for (std::size_t i = 0; i < points.size() - 1; ++i) {
        if (references) {
            BOOST_CHECK_EQUAL(&rich_line.rich_segments()[i].first(), &points[i]);
            BOOST_CHECK_EQUAL(&rich_line.rich_segments()[i].second(), &points[i + 1]);
        } else {
            BOOST_CHECK_NE(&rich_line.rich_segments()[i].first(), &points[i]);
            BOOST_CHECK_NE(&rich_line.rich_segments()[i].second(), &points[i + 1]);
        }
    }

    if (points.size() >= 2 and not boost::geometry::equals(line.front(), line.back())) {
        BOOST_CHECK(rich_line.has_length());
        BOOST_CHECK(rich_line.has_azimuth());
        BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), boost::geometry::length(line), 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), map_matching_2::geometry::azimuth_line_deg(line), 1e-6);
    } else {
        BOOST_CHECK(not rich_line.has_length());
        BOOST_CHECK(not rich_line.has_azimuth());
    }

    if (points.size() >= 3) {
        BOOST_CHECK(rich_line.has_directions());
        BOOST_CHECK_CLOSE_FRACTION(rich_line.directions(), map_matching_2::geometry::directions_deg(line), 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(rich_line.absolute_directions(),
                                   map_matching_2::geometry::absolute_directions_deg(line), 1e-6);
    }
}

template<typename RichLineA, typename RichLineB>
void test_rich_line_equality(const RichLineA &a, const RichLineB &b) {
    if constexpr (std::is_same_v<RichLineA, RichLineB>) {
        BOOST_CHECK_EQUAL(a, b);
    }
    BOOST_CHECK_EQUAL(a.size(), b.size());
    for (std::size_t i = 0; i < a.rich_segments().size() and i < b.rich_segments().size(); ++i) {
        test_rich_segment_equality(a.rich_segments()[i], b.rich_segments()[i]);
    }
    BOOST_CHECK_EQUAL(a.has_length(), b.has_length());
    BOOST_CHECK_EQUAL(a.has_azimuth(), b.has_azimuth());
    BOOST_CHECK_EQUAL(a.has_directions(), b.has_directions());
    BOOST_CHECK_EQUAL(a.length(), b.length());
    BOOST_CHECK_EQUAL(a.azimuth(), b.azimuth());
    BOOST_CHECK_EQUAL(a.directions(), b.directions());
    BOOST_CHECK_EQUAL(a.absolute_directions(), b.absolute_directions());
}

template<typename RichLine>
void test_rich_line_type(bool references) {
    using rich_line_type = RichLine;
    using rich_segment_type = typename map_matching_2::geometry::rich_line_traits<rich_line_type>::rich_segment_type;
    using line_type = typename map_matching_2::geometry::rich_line_traits<rich_line_type>::line_type;
    using segment_type = typename map_matching_2::geometry::rich_line_traits<rich_line_type>::segment_type;

    point_type point_1{0.0, 0.0};
    point_type point_2{0.0, 1.0};
    point_type point_3{1.0, 1.0};
    std::vector<point_type> points{point_1, point_2, point_3};

    if constexpr (std::is_default_constructible_v<segment_type>) {
        rich_line_type rich_line_0;
    }

    rich_segment_type rich_segment_1{segment_type{points[0], points[1]}};
    rich_segment_type rich_segment_2{segment_type{points[1], points[2]}};
    rich_line_type rich_line_1{std::vector{rich_segment_1, rich_segment_2}};
    test_rich_line(rich_line_1, points, references);

    if constexpr (std::is_copy_constructible_v<rich_segment_type>) {
        rich_line_type rich_line_2{rich_line_1};
        test_rich_line_equality(rich_line_2, rich_line_1);
    }

    if constexpr (std::is_copy_assignable_v<rich_segment_type>) {
        rich_line_type rich_line_3{std::vector{rich_segment_1, rich_segment_2}};
        rich_line_3 = rich_line_1;
        test_rich_line_equality(rich_line_3, rich_line_1);
    }

    if constexpr (std::is_move_constructible_v<rich_segment_type>) {
        rich_line_type rich_line_2{std::vector{rich_segment_1, rich_segment_2}};
        rich_line_type rich_line_4{std::move(rich_line_2)};
        test_rich_line_equality(rich_line_4, rich_line_1);
    }

    if constexpr (std::is_move_assignable_v<rich_segment_type>) {
        rich_line_type rich_line_3{std::vector{rich_segment_1, rich_segment_2}};
        rich_line_type rich_line_5{std::vector{rich_segment_1, rich_segment_2}};
        rich_line_5 = std::move(rich_line_3);
        test_rich_line_equality(rich_line_5, rich_line_1);
    }
}

template<typename RichLineA, typename RichLineB>
void test_rich_line_conversion() {
    using rich_line_type_a = RichLineA;
    using rich_line_type_b = RichLineB;
    using rich_segment_type_a = typename map_matching_2::geometry::rich_line_traits<rich_line_type_a>::rich_segment_type;
    using rich_segment_type_b = typename map_matching_2::geometry::rich_line_traits<rich_line_type_b>::rich_segment_type;
    using line_type_a = typename map_matching_2::geometry::rich_line_traits<rich_line_type_a>::line_type;
    using line_type_b = typename map_matching_2::geometry::rich_line_traits<rich_line_type_b>::line_type;
    using segment_type_a = typename map_matching_2::geometry::rich_line_traits<rich_line_type_a>::segment_type;
    using segment_type_b = typename map_matching_2::geometry::rich_line_traits<rich_line_type_b>::segment_type;

    point_type point_1{0.0, 0.0};
    point_type point_2{0.0, 1.0};
    point_type point_3{1.0, 1.0};
    std::vector<point_type> points{point_1, point_2, point_3};

    BOOST_TEST_MESSAGE("RichLineA: " << typeid(RichLineA).name() << ", "
                                     << "RichLineB: " << typeid(RichLineB).name());

    rich_segment_type_a rich_segment_1{segment_type_a{points[0], points[1]}};
    rich_segment_type_a rich_segment_2{segment_type_a{points[1], points[2]}};
    rich_line_type_a rich_line_1{std::vector{rich_segment_1, rich_segment_2}};

    if constexpr (std::is_copy_constructible_v<rich_segment_type_b>) {
        rich_line_type_b rich_line_2{rich_line_1};
        test_rich_line_equality(rich_line_2, rich_line_1);
    }

    if constexpr (std::is_copy_assignable_v<rich_segment_type_b>) {
        rich_line_type_b rich_line_3{std::vector{rich_segment_1, rich_segment_2}};
        rich_line_3 = rich_line_1;
        test_rich_line_equality(rich_line_3, rich_line_1);
    }

    if constexpr (std::is_move_constructible_v<rich_segment_type_b>) {
        rich_line_type_a rich_line_2{std::vector{rich_segment_1, rich_segment_2}};
        rich_line_type_b rich_line_4{std::move(rich_line_2)};
        test_rich_line_equality(rich_line_4, rich_line_1);
    }

    if constexpr (std::is_move_assignable_v<rich_segment_type_b>) {
        rich_line_type_a rich_line_3{std::vector{rich_segment_1, rich_segment_2}};
        rich_line_type_b rich_line_5{std::vector{rich_segment_1, rich_segment_2}};
        rich_line_5 = std::move(rich_line_3);
        test_rich_line_equality(rich_line_5, rich_line_1);
    }
}

template<template<typename> typename RichLineA,
        template<typename> typename RichLineB>
void test_rich_line_conversion_types() {
    using rich_segment_type = map_matching_2::geometry::rich_segment_type<point_type>;
    using rich_referring_segment_type = map_matching_2::geometry::rich_referring_segment_type<point_type>;
    using rich_pointing_segment_type = map_matching_2::geometry::rich_pointing_segment_type<point_type>;

    test_rich_line_conversion<RichLineA<rich_segment_type>, RichLineB<rich_segment_type>>();
    test_rich_line_conversion<RichLineA<rich_segment_type>, RichLineB<rich_referring_segment_type>>();
    test_rich_line_conversion<RichLineA<rich_segment_type>, RichLineB<rich_pointing_segment_type>>();
    test_rich_line_conversion<RichLineA<rich_referring_segment_type>, RichLineB<rich_segment_type>>();
    test_rich_line_conversion<RichLineA<rich_referring_segment_type>, RichLineB<rich_referring_segment_type>>();
    test_rich_line_conversion<RichLineA<rich_referring_segment_type>, RichLineB<rich_pointing_segment_type>>();
    test_rich_line_conversion<RichLineA<rich_pointing_segment_type>, RichLineB<rich_segment_type>>();
    test_rich_line_conversion<RichLineA<rich_pointing_segment_type>, RichLineB<rich_referring_segment_type>>();
    test_rich_line_conversion<RichLineA<rich_pointing_segment_type>, RichLineB<rich_pointing_segment_type>>();
}

template<typename MultiRichLine>
void test_multi_rich_line(const MultiRichLine &multi_rich_line, const std::vector<std::vector<point_type>> &points,
                          bool references) {
    using multi_line_type = typename map_matching_2::geometry::rich_line_traits<MultiRichLine>::multi_line_type;
    using angle_type = typename map_matching_2::geometry::rich_line_traits<MultiRichLine>::angle_type;

    multi_line_type multi_line;

    std::size_t points_sizes = std::accumulate(std::cbegin(points), std::cend(points), 0L,
                                               [](const auto v, const auto &vec) { return v + vec.size(); });

    BOOST_TEST_MESSAGE("sizeof: " << sizeof(multi_rich_line));
    BOOST_CHECK_EQUAL(multi_rich_line.rich_lines().size(), points.size());
    BOOST_CHECK_EQUAL(multi_rich_line.sizes(), points_sizes);

    bool has = false, has_directions = false;
    angle_type directions = map_matching_2::geometry::default_float_type<angle_type>::v0,
            absolute_directions = map_matching_2::geometry::default_float_type<angle_type>::v0;
    for (std::size_t l = 0; l < points.size(); ++l) {
        using line_type = typename map_matching_2::geometry::rich_line_traits<MultiRichLine>::line_type;

        line_type line{std::cbegin(points[l]), std::cend(points[l])};
        multi_line.emplace_back(line);

        if (not boost::geometry::equals(line.front(), line.back())) {
            has = true;
        }

        if (points[l].size() >= 3) {
            has_directions = true;
            directions += map_matching_2::geometry::directions_deg(line);
            absolute_directions += map_matching_2::geometry::absolute_directions_deg(line);
        }

        BOOST_CHECK(boost::geometry::equals(multi_rich_line.rich_lines()[l].line(), line));

        for (std::size_t i = 0; i < points[l].size() - 1; ++i) {
            if (references) {
                BOOST_CHECK_EQUAL(&multi_rich_line.rich_lines()[l].rich_segments()[i].first(), &points[l][i]);
                BOOST_CHECK_EQUAL(&multi_rich_line.rich_lines()[l].rich_segments()[i].second(), &points[l][i + 1]);
            } else {
                BOOST_CHECK_NE(&multi_rich_line.rich_lines()[l].rich_segments()[i].first(), &points[l][i]);
                BOOST_CHECK_NE(&multi_rich_line.rich_lines()[l].rich_segments()[i].second(), &points[l][i + 1]);
            }
        }
    }

    BOOST_CHECK(boost::geometry::equals(multi_rich_line.multi_line(), multi_line));

    if (points_sizes >= 2 and has) {
        BOOST_CHECK(multi_rich_line.has_length());
        BOOST_CHECK(multi_rich_line.has_azimuth());
        BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), boost::geometry::length(multi_line), 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(),
                                   map_matching_2::geometry::azimuth_deg(multi_line.front().front(),
                                                                         multi_line.back().back()), 1e-6);
    } else {
        BOOST_CHECK(not multi_rich_line.has_length());
        BOOST_CHECK(not multi_rich_line.has_azimuth());
    }

    if (has_directions) {
        BOOST_CHECK(multi_rich_line.has_directions());
        BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.directions(), directions, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.absolute_directions(), absolute_directions, 1e-6);
    }
}

template<typename MultiRichLineA, typename MultiRichLineB>
void test_multi_rich_line_equality(const MultiRichLineA &a, const MultiRichLineB &b) {
    if constexpr (std::is_same_v<MultiRichLineA, MultiRichLineB>) {
        BOOST_CHECK_EQUAL(a, b);
    }
    BOOST_CHECK_EQUAL(a.sizes(), b.sizes());
    for (std::size_t i = 0; i < a.rich_lines().size() and i < b.rich_lines().size(); ++i) {
        test_rich_line_equality(a.rich_lines()[i], b.rich_lines()[i]);
    }
    BOOST_CHECK_EQUAL(a.has_length(), b.has_length());
    BOOST_CHECK_EQUAL(a.has_azimuth(), b.has_azimuth());
    BOOST_CHECK_EQUAL(a.has_directions(), b.has_directions());
    BOOST_CHECK_EQUAL(a.length(), b.length());
    BOOST_CHECK_EQUAL(a.azimuth(), b.azimuth());
    BOOST_CHECK_EQUAL(a.directions(), b.directions());
    BOOST_CHECK_EQUAL(a.absolute_directions(), b.absolute_directions());
}

template<typename MultiRichLine>
void test_multi_rich_line_type(bool references) {
    using multi_rich_line_type = MultiRichLine;
    using rich_line_type = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type>::rich_line_type;
    using rich_segment_type = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type>::rich_segment_type;
    using line_type = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type>::line_type;
    using segment_type = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type>::segment_type;

    if constexpr (std::is_default_constructible_v<segment_type>) {
        multi_rich_line_type multi_rich_line_0;
    }

    point_type point_1{0.0, 0.0};
    point_type point_2{0.0, 1.0};
    point_type point_3{1.0, 1.0};
    point_type point_4{0.0, 0.0};
    point_type point_5{0.0, 1.0};
    point_type point_6{1.0, 1.0};
    std::vector<std::vector<point_type>> points{{point_1, point_2, point_3},
                                                {point_4, point_5, point_6}};

    rich_segment_type rich_segment_1{segment_type{points[0][0], points[0][1]}};
    rich_segment_type rich_segment_2{segment_type{points[0][1], points[0][2]}};
    rich_line_type rich_line_1{std::vector{rich_segment_1, rich_segment_2}};

    rich_segment_type rich_segment_3{segment_type{points[1][0], points[1][1]}};
    rich_segment_type rich_segment_4{segment_type{points[1][1], points[1][2]}};
    rich_line_type rich_line_2{std::vector{rich_segment_3, rich_segment_4}};

    multi_rich_line_type multi_rich_line_1{std::vector{rich_line_1, rich_line_2}};
    test_multi_rich_line(multi_rich_line_1, points, references);

    if constexpr (std::is_copy_constructible_v<rich_segment_type>) {
        multi_rich_line_type multi_rich_line_2{multi_rich_line_1};
        test_multi_rich_line_equality(multi_rich_line_2, multi_rich_line_1);
    }

    if constexpr (std::is_copy_assignable_v<rich_segment_type>) {
        multi_rich_line_type multi_rich_line_3{std::vector{rich_line_1, rich_line_2}};
        multi_rich_line_3 = multi_rich_line_1;
        test_multi_rich_line_equality(multi_rich_line_3, multi_rich_line_1);
    }

    if constexpr (std::is_move_constructible_v<rich_segment_type>) {
        multi_rich_line_type multi_rich_line_2{std::vector{rich_line_1, rich_line_2}};
        multi_rich_line_type multi_rich_line_4{std::move(multi_rich_line_2)};
        test_multi_rich_line_equality(multi_rich_line_4, multi_rich_line_1);
    }

    if constexpr (std::is_move_assignable_v<rich_segment_type>) {
        multi_rich_line_type multi_rich_line_3{std::vector{rich_line_1, rich_line_2}};
        multi_rich_line_type multi_rich_line_5{std::vector{rich_line_1, rich_line_2}};
        multi_rich_line_5 = std::move(multi_rich_line_3);
        test_multi_rich_line_equality(multi_rich_line_5, multi_rich_line_1);
    }
}

template<typename MultiRichLineA, typename MultiRichLineB>
void test_multi_rich_line_conversion() {
    using multi_rich_line_type_a = MultiRichLineA;
    using multi_rich_line_type_b = MultiRichLineB;
    using rich_line_type_a = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type_a>::rich_line_type;
    using rich_line_type_b = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type_b>::rich_line_type;
    using rich_segment_type_a = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type_a>::rich_segment_type;
    using rich_segment_type_b = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type_b>::rich_segment_type;
    using line_type_a = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type_a>::line_type;
    using line_type_b = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type_b>::line_type;
    using segment_type_a = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type_a>::segment_type;
    using segment_type_b = typename map_matching_2::geometry::multi_rich_line_traits<multi_rich_line_type_b>::segment_type;

    point_type point_1{0.0, 0.0};
    point_type point_2{0.0, 1.0};
    point_type point_3{1.0, 1.0};
    point_type point_4{0.0, 0.0};
    point_type point_5{0.0, 1.0};
    point_type point_6{1.0, 1.0};
    std::vector<std::vector<point_type>> points{{point_1, point_2, point_3},
                                                {point_4, point_5, point_6}};

    rich_segment_type_a rich_segment_1{segment_type_a{points[0][0], points[0][1]}};
    rich_segment_type_a rich_segment_2{segment_type_a{points[0][1], points[0][2]}};
    rich_line_type_a rich_line_1{std::vector{rich_segment_1, rich_segment_2}};

    rich_segment_type_a rich_segment_3{segment_type_a{points[1][0], points[1][1]}};
    rich_segment_type_a rich_segment_4{segment_type_a{points[1][1], points[1][2]}};
    rich_line_type_a rich_line_2{std::vector{rich_segment_3, rich_segment_4}};

    multi_rich_line_type_a multi_rich_line_1{std::vector{rich_line_1, rich_line_2}};

    if constexpr (std::is_copy_constructible_v<rich_segment_type_b>) {
        multi_rich_line_type_b multi_rich_line_2{multi_rich_line_1};
        test_multi_rich_line_equality(multi_rich_line_2, multi_rich_line_1);
    }

    if constexpr (std::is_copy_assignable_v<rich_segment_type_b>) {
        multi_rich_line_type_b multi_rich_line_3{std::vector{rich_line_1, rich_line_2}};
        multi_rich_line_3 = multi_rich_line_1;
        test_multi_rich_line_equality(multi_rich_line_3, multi_rich_line_1);
    }

    if constexpr (std::is_move_constructible_v<rich_segment_type_b>) {
        multi_rich_line_type_a multi_rich_line_2{std::vector{rich_line_1, rich_line_2}};
        multi_rich_line_type_b multi_rich_line_4{std::move(multi_rich_line_2)};
        test_multi_rich_line_equality(multi_rich_line_4, multi_rich_line_1);
    }

    if constexpr (std::is_move_assignable_v<rich_segment_type_b>) {
        multi_rich_line_type_a multi_rich_line_3{std::vector{rich_line_1, rich_line_2}};
        multi_rich_line_type_b multi_rich_line_5{std::vector{rich_line_1, rich_line_2}};
        multi_rich_line_5 = std::move(multi_rich_line_3);
        test_multi_rich_line_equality(multi_rich_line_5, multi_rich_line_1);
    }
}

template<template<typename> typename MultiRichLineA,
        template<typename> typename MultiRichLineB>
void test_multi_rich_line_conversion_types() {
    using rich_line_type = map_matching_2::geometry::rich_line_type<point_type>;
    using rich_referring_line_type = map_matching_2::geometry::rich_referring_line_type<point_type>;
    using rich_pointing_line_type = map_matching_2::geometry::rich_pointing_line_type<point_type>;

    test_multi_rich_line_conversion<MultiRichLineA<rich_line_type>, MultiRichLineB<rich_line_type>>();
    test_multi_rich_line_conversion<MultiRichLineA<rich_line_type>, MultiRichLineB<rich_referring_line_type>>();
    test_multi_rich_line_conversion<MultiRichLineA<rich_line_type>, MultiRichLineB<rich_pointing_line_type>>();
    test_multi_rich_line_conversion<MultiRichLineA<rich_referring_line_type>, MultiRichLineB<rich_line_type>>();
    test_multi_rich_line_conversion<MultiRichLineA<rich_referring_line_type>, MultiRichLineB<rich_referring_line_type>>();
    test_multi_rich_line_conversion<MultiRichLineA<rich_referring_line_type>, MultiRichLineB<rich_pointing_line_type>>();
    test_multi_rich_line_conversion<MultiRichLineA<rich_pointing_line_type>, MultiRichLineB<rich_line_type>>();
    test_multi_rich_line_conversion<MultiRichLineA<rich_pointing_line_type>, MultiRichLineB<rich_referring_line_type>>();
    test_multi_rich_line_conversion<MultiRichLineA<rich_pointing_line_type>, MultiRichLineB<rich_pointing_line_type>>();
}

BOOST_AUTO_TEST_SUITE(model_tests)

    BOOST_AUTO_TEST_CASE(rich_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::rich_segment_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(rich_referring_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::rich_referring_segment_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(rich_pointing_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::rich_pointing_segment_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(eager_rich_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::eager_rich_segment_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(eager_rich_referring_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::eager_rich_referring_segment_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(eager_rich_pointing_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::eager_rich_pointing_segment_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::lazy_rich_segment_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_referring_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::lazy_rich_referring_segment_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_pointing_segment_test) {
        test_rich_segment_type<map_matching_2::geometry::lazy_rich_pointing_segment_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(rich_segment_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::rich_segment,
                map_matching_2::geometry::rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(rich_segment_eager_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::rich_segment,
                map_matching_2::geometry::eager_rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(rich_segment_lazy_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::rich_segment,
                map_matching_2::geometry::lazy_rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(eager_rich_segment_eager_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::eager_rich_segment,
                map_matching_2::geometry::eager_rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(eager_rich_segment_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::eager_rich_segment,
                map_matching_2::geometry::rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(eager_rich_segment_lazy_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::eager_rich_segment,
                map_matching_2::geometry::lazy_rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_segment_lazy_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::lazy_rich_segment,
                map_matching_2::geometry::lazy_rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_segment_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::lazy_rich_segment,
                map_matching_2::geometry::rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_segment_eager_rich_segment_conversion_test) {
        test_rich_segment_conversion_types<
                map_matching_2::geometry::lazy_rich_segment,
                map_matching_2::geometry::eager_rich_segment>();
    }

    BOOST_AUTO_TEST_CASE(rich_line_test) {
        test_rich_line_type<map_matching_2::geometry::rich_line_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(rich_referring_line_test) {
        test_rich_line_type<map_matching_2::geometry::rich_referring_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(rich_pointing_line_test) {
        test_rich_line_type<map_matching_2::geometry::rich_pointing_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(eager_rich_line_test) {
        test_rich_line_type<map_matching_2::geometry::eager_rich_line_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(eager_rich_referring_line_test) {
        test_rich_line_type<map_matching_2::geometry::eager_rich_referring_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(eager_rich_pointing_line_test) {
        test_rich_line_type<map_matching_2::geometry::eager_rich_pointing_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_line_test) {
        test_rich_line_type<map_matching_2::geometry::lazy_rich_line_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_referring_line_test) {
        test_rich_line_type<map_matching_2::geometry::lazy_rich_referring_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_pointing_line_test) {
        test_rich_line_type<map_matching_2::geometry::lazy_rich_pointing_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(rich_line_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::rich_line,
                map_matching_2::geometry::rich_line>();
    }

    BOOST_AUTO_TEST_CASE(rich_line_eager_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::rich_line,
                map_matching_2::geometry::eager_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(rich_line_lazy_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::rich_line,
                map_matching_2::geometry::lazy_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(eager_rich_line_eager_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::eager_rich_line,
                map_matching_2::geometry::eager_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(eager_rich_line_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::eager_rich_line,
                map_matching_2::geometry::rich_line>();
    }

    BOOST_AUTO_TEST_CASE(eager_rich_line_lazy_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::eager_rich_line,
                map_matching_2::geometry::lazy_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_line_lazy_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::lazy_rich_line,
                map_matching_2::geometry::lazy_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_line_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::lazy_rich_line,
                map_matching_2::geometry::rich_line>();
    }

    BOOST_AUTO_TEST_CASE(lazy_rich_line_eager_rich_line_conversion_test) {
        test_rich_line_conversion_types<
                map_matching_2::geometry::lazy_rich_line,
                map_matching_2::geometry::eager_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(multi_rich_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::multi_rich_line_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(multi_rich_referring_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::multi_rich_referring_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(multi_rich_pointing_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::multi_rich_pointing_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(eager_multi_rich_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::eager_multi_rich_line_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(eager_multi_rich_referring_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::eager_multi_rich_referring_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(eager_multi_rich_pointing_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::eager_multi_rich_pointing_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(lazy_multi_rich_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::lazy_multi_rich_line_type<point_type>>(false);
    }

    BOOST_AUTO_TEST_CASE(lazy_multi_rich_referring_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::lazy_multi_rich_referring_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(lazy_multi_rich_pointing_line_test) {
        test_multi_rich_line_type<map_matching_2::geometry::lazy_multi_rich_pointing_line_type<point_type>>(true);
    }

    BOOST_AUTO_TEST_CASE(multi_rich_line_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::multi_rich_line,
                map_matching_2::geometry::multi_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(multi_rich_line_eager_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::multi_rich_line,
                map_matching_2::geometry::eager_multi_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(multi_rich_line_lazy_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::multi_rich_line,
                map_matching_2::geometry::lazy_multi_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(eager_multi_rich_line_eager_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::eager_multi_rich_line,
                map_matching_2::geometry::eager_multi_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(eager_multi_rich_line_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::eager_multi_rich_line,
                map_matching_2::geometry::multi_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(eager_multi_rich_line_lazy_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::eager_multi_rich_line,
                map_matching_2::geometry::lazy_multi_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(lazy_multi_rich_line_lazy_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::lazy_multi_rich_line,
                map_matching_2::geometry::lazy_multi_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(lazy_multi_rich_line_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::lazy_multi_rich_line,
                map_matching_2::geometry::multi_rich_line>();
    }

    BOOST_AUTO_TEST_CASE(lazy_multi_rich_line_eager_multi_rich_line_conversion_test) {
        test_multi_rich_line_conversion_types<
                map_matching_2::geometry::lazy_multi_rich_line,
                map_matching_2::geometry::eager_multi_rich_line>();
    }

BOOST_AUTO_TEST_SUITE_END()
