// Copyright (C) 2024 Adrian Wöltche
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

#include <boost/test/unit_test.hpp>

#include "types/geometry/rich_type/rich_line.hpp"
#include "types/geometry/rich_type/eager_rich_line.hpp"
#include "types/geometry/rich_type/lazy_rich_line.hpp"
#include "types/geometry/rich_type/multi_rich_line.hpp"
#include "types/geometry/rich_type/eager_multi_rich_line.hpp"
#include "types/geometry/rich_type/lazy_multi_rich_line.hpp"

#include "geometry/rich_type/rich_segment.hpp"
#include "geometry/rich_type/traits/rich_segment.hpp"

#include "geometry/algorithm/simplify.hpp"
#include "geometry/algorithm/substring.hpp"

#include "helper/geometry_helper.hpp"

using point_type = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
using line_type = typename map_matching_2::geometry::models<point_type>::line_type<>;
using multi_line_type = typename map_matching_2::geometry::models<point_type>::multi_line_type<line_type>;

using rich_segment = map_matching_2::geometry::rich_segment<
    typename map_matching_2::geometry::models<point_type>::segment_type>;
using eager_rich_segment = map_matching_2::geometry::eager_rich_segment<
    typename map_matching_2::geometry::models<point_type>::segment_type>;
using lazy_rich_segment = map_matching_2::geometry::lazy_rich_segment<
    typename map_matching_2::geometry::models<point_type>::segment_type>;

using rich_line_type = map_matching_2::geometry::rich_line<point_type>;
using eager_rich_line_type = map_matching_2::geometry::eager_rich_line<point_type>;
using lazy_rich_line_type = map_matching_2::geometry::lazy_rich_line<point_type>;

using multi_rich_line_type = map_matching_2::geometry::multi_rich_line<rich_line_type>;
using eager_multi_rich_line_type = map_matching_2::geometry::eager_multi_rich_line<eager_rich_line_type>;
using lazy_multi_rich_line_type = map_matching_2::geometry::lazy_multi_rich_line<lazy_rich_line_type>;

void test_rich_line_erase(line_type line, const std::vector<std::size_t> &indices, const auto &check) {
    auto test_rich_line = [&]<typename RichLine>() {
        auto rich_line = RichLine{line};
        rich_line.erase(indices);
        check(rich_line);
    };

    test_rich_line.template operator()<rich_line_type>();
    test_rich_line.template operator()<eager_rich_line_type>();
    test_rich_line.template operator()<lazy_rich_line_type>();
}

void test_sub_rich_line(line_type line, std::size_t from, std::size_t to, const auto &check) {
    auto test_rich_line = [&]<typename RichLine>() {
        auto input = RichLine{line};
        auto output = input.sub_rich_line(from, to);
        check(output);
    };

    test_rich_line.template operator()<rich_line_type>();
    test_rich_line.template operator()<eager_rich_line_type>();
    test_rich_line.template operator()<lazy_rich_line_type>();
}

void test_sub_multi_rich_line(multi_line_type multi_line, std::size_t from, std::size_t to, const auto &check) {
    auto test_multi_rich_line = [&]<typename MultiRichLine>() {
        auto input = MultiRichLine{multi_line};
        auto output = input.sub_multi_rich_line(from, to);
        check(output);
    };

    test_multi_rich_line.template operator()<multi_rich_line_type>();
    test_multi_rich_line.template operator()<eager_multi_rich_line_type>();
    test_multi_rich_line.template operator()<lazy_multi_rich_line_type>();
}

void test_extract_multi_rich_line(multi_line_type multi_line, std::size_t outer_from, std::size_t inner_from,
        std::size_t inner_to, std::size_t outer_to, const auto &check) {
    auto test_multi_rich_line = [&]<typename MultiRichLine>() {
        auto input = MultiRichLine{multi_line};
        auto output = input.template extract<MultiRichLine>(outer_from, inner_from, inner_to, outer_to);
        check(output);
    };

    test_multi_rich_line.template operator()<multi_rich_line_type>();
    test_multi_rich_line.template operator()<eager_multi_rich_line_type>();
    test_multi_rich_line.template operator()<lazy_multi_rich_line_type>();
}

void test_simplify_rich_lines(line_type input, line_type output, bool retain_reversals = true) {
    auto test_rich_line = [&]<typename RichLine>() {
        auto rich_line_input = RichLine{input};
        auto rich_line_output = RichLine{output};

        map_matching_2::geometry::simplify_rich_line(rich_line_input, retain_reversals);

        BOOST_CHECK_EQUAL(rich_line_input, rich_line_output);
    };

    test_rich_line.template operator()<rich_line_type>();
    test_rich_line.template operator()<eager_rich_line_type>();
    test_rich_line.template operator()<lazy_rich_line_type>();
}

void test_simplify(line_type input, line_type output, bool retain_reversals = true) {
    //    auto segments_input = map_matching_2::geometry::line2rich_segments<std::list<rich_segment>>(input);
    //    auto segments_output = map_matching_2::geometry::line2rich_segments<std::list<rich_segment>>(output);
    //
    //    map_matching_2::geometry::simplify_rich_segments(segments_input, retain_reversals);
    //
    //    BOOST_CHECK_EQUAL_COLLECTIONS(segments_input.begin(), segments_input.end(),
    //                                  segments_output.begin(), segments_output.end());

    test_simplify_rich_lines(input, output, retain_reversals);
}

BOOST_AUTO_TEST_SUITE(model_algorithm_tests)

    BOOST_AUTO_TEST_CASE(rich_line_erase_test) {
        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                {1, 2, 3, 4},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 2);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 5.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_EQUAL(rich_line.has_directions(), false);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                {0, 1, 2, 3, 4, 5},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 0);
                    BOOST_CHECK_EQUAL(rich_line.has_length(), false);
                    BOOST_CHECK_EQUAL(rich_line.has_azimuth(), false);
                    BOOST_CHECK_EQUAL(rich_line.has_directions(), false);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                {0},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 5);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 4.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(rich_line.absolute_directions(), 1e-6);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                {5},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 5);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 4.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(rich_line.absolute_directions(), 1e-6);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 2, 1, 2, 2}),
                {},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 5);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 4.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 45.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.directions(), 90, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.absolute_directions(), 90, 1e-6);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 2, 1, 2, 2}),
                {6},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 5);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 4.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 45.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.directions(), 90, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.absolute_directions(), 90, 1e-6);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 2, 1, 2, 2}),
                {1, 3},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 3);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 4.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 45.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.directions(), 90, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.absolute_directions(), 90, 1e-6);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 2, 1, 2, 2}),
                {0, 4},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 3);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 2.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 45.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.directions(), 90, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.absolute_directions(), 90, 1e-6);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 2, 1, 2, 2}),
                {2},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 4);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 2.0 + std::sqrt(2.0), 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 45.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.directions(), 90, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.absolute_directions(), 90, 1e-6);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 2, 1, 2, 2}),
                {0},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 4);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 3.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(),
                            map_matching_2::geometry::azimuth_deg(
                                point_type{1, 0}, point_type{2, 2}), 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.directions(), 90, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.absolute_directions(), 90, 1e-6);
                });

        test_rich_line_erase(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 2, 1, 2, 2}),
                {4},
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 4);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 3.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(),
                            map_matching_2::geometry::azimuth_deg(
                                point_type{0, 0}, point_type{2, 1}), 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.directions(), 90, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.absolute_directions(), 90, 1e-6);
                });
    }

    BOOST_AUTO_TEST_CASE(rich_line_sub_line_test) {
        test_sub_rich_line(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                1, 5,
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 4);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 3.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(rich_line.absolute_directions(), 1e-6);
                });

        test_sub_rich_line(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                0, 6,
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 5.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(rich_line.absolute_directions(), 1e-6);
                });

        test_sub_rich_line(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                0, 0,
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 0);
                    BOOST_CHECK_EQUAL(rich_line.has_length(), false);
                    BOOST_CHECK_EQUAL(rich_line.has_azimuth(), false);
                    BOOST_CHECK_EQUAL(rich_line.has_directions(), false);
                });

        test_sub_rich_line(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                5, 5,
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 0);
                    BOOST_CHECK_EQUAL(rich_line.has_length(), false);
                    BOOST_CHECK_EQUAL(rich_line.has_azimuth(), false);
                    BOOST_CHECK_EQUAL(rich_line.has_directions(), false);
                });

        test_sub_rich_line(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                1, 4,
                [](const auto &rich_line) {
                    BOOST_CHECK_EQUAL(rich_line.size(), 3);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.length(), 2.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(rich_line.absolute_directions(), 1e-6);
                });
    }

    BOOST_AUTO_TEST_CASE(multi_rich_line_sub_multi_rich_line_test) {
        test_sub_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                1, 4,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 3);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 15.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_sub_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                0, 3,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 3);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 15.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_sub_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                2, 5,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 3);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 15.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_sub_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                0, 5,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 5);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 25.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_sub_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                0, 0,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 0);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_length(), false);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_azimuth(), false);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_directions(), false);
                });

        test_sub_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                4, 4,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 0);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_length(), false);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_azimuth(), false);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_directions(), false);
                });
    }

    BOOST_AUTO_TEST_CASE(multi_rich_line_extract_test) {
        test_extract_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                1, 2, 3, 4,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 3);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 10.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_extract_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                1, 0, 6, 4,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 3);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 15.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_extract_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                1, 2, 5, 2,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 1);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 2.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_extract_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                1, 2, 5, 1,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 0);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_length(), false);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_azimuth(), false);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_directions(), false);
                });

        test_extract_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                1, 2, 2, 2,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 1);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_length(), false);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_azimuth(), false);
                    BOOST_CHECK_EQUAL(multi_rich_line.has_directions(), false);
                });

        test_extract_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                0, 0, 6, 2,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 2);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 10.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_extract_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                3, 3, 6, 5,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 2);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 7.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });

        test_extract_multi_rich_line(multi_line_type{
                        create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                        create_line<point_type, line_type>({5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0}),
                        create_line<point_type, line_type>({10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0}),
                        create_line<point_type, line_type>({15, 0, 16, 0, 17, 0, 18, 0, 19, 0, 20, 0}),
                        create_line<point_type, line_type>({20, 0, 21, 0, 22, 0, 23, 0, 24, 0, 25, 0})
                },
                0, 0, 6, 5,
                [](const auto &multi_rich_line) {
                    BOOST_CHECK_EQUAL(multi_rich_line.size(), 5);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.length(), 25.0, 1e-6);
                    BOOST_CHECK_CLOSE_FRACTION(multi_rich_line.azimuth(), 90.0, 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.directions(), 1e-6);
                    BOOST_CHECK_LE(multi_rich_line.absolute_directions(), 1e-6);
                });
    }

    BOOST_AUTO_TEST_CASE(simplify_test) {
        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 5, 0}));

        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 1, 3, 0, 4, 0, 5, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 1, 3, 0, 5, 0}));

        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 1, 0, 2, 0, 3, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 2, 0, 1, 0, 3, 0}));

        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 1, 0, 2, 0, 3, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 3, 0}), false);

        test_simplify(create_line<point_type, line_type>(
                        {0, 0}),
                create_line<point_type, line_type>(
                        {0, 0}));

        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 1, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 1, 0}));

        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 2, 0, 1, 0, 2, 0, 3, 0, 4, 0, 3, 0, 2, 0, 3, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 3, 0, 1, 0, 4, 0, 2, 0, 3, 0}));

        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 0, 2, 0, 1, 0, 2, 0, 3, 0, 4, 0, 3, 0, 2, 0, 3, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 4, 0, 2, 0, 3, 0}), false);

        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 3, 0, 0, 0, 3, 0, 0, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 3, 0, 0, 0, 3, 0, 0, 0}));

        test_simplify(create_line<point_type, line_type>(
                        {0, 0, 1, 0, 2, 0, 3, 1, 4, 2, 5, 3, 6, 2, 7, 1, 8, 0, 9, 0}),
                create_line<point_type, line_type>(
                        {0, 0, 2, 0, 5, 3, 8, 0, 9, 0}));
    }

    BOOST_AUTO_TEST_CASE(substring_test) {
        auto line_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 5, 0, 5, 5})};
        BOOST_CHECK_EQUAL(line_1.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(line_1.length(), 10.0, 1e-6);

        auto substring_points_1_1 = map_matching_2::geometry::substring_points(
                line_1, point_type{0, 0}, point_type{5, 0});
        BOOST_CHECK_EQUAL(substring_points_1_1.size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_1.length(), 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_1.azimuth(), 90.0, 1e-6);
        BOOST_CHECK_EQUAL(substring_points_1_1.has_directions(), false);

        auto substring_points_1_2 = map_matching_2::geometry::substring_points(
                line_1, point_type{-1, 0}, point_type{5, 0});
        BOOST_CHECK_EQUAL(substring_points_1_2.size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_2.length(), 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_2.azimuth(), 90.0, 1e-6);
        BOOST_CHECK_EQUAL(substring_points_1_2.has_directions(), false);

        auto substring_points_1_3 = map_matching_2::geometry::substring_points(
                line_1, point_type{-1, 0}, point_type{6, 0});
        BOOST_CHECK_EQUAL(substring_points_1_3.size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_3.length(), 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_3.azimuth(), 90.0, 1e-6);
        BOOST_CHECK_EQUAL(substring_points_1_3.has_directions(), false);

        auto substring_points_1_4 = map_matching_2::geometry::substring_points(
                line_1, point_type{-1, 0}, point_type{5, 6});
        BOOST_CHECK_EQUAL(substring_points_1_4.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_4.length(), 10.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_4.azimuth(), 45.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_4.directions(), 90.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_4.absolute_directions(), 90.0, 1e-6);

        auto substring_points_1_5 = map_matching_2::geometry::substring_points(
                line_1, point_type{2, 0}, point_type{3, 0});
        BOOST_CHECK_EQUAL(substring_points_1_5.size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_5.length(), 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_5.azimuth(), 90.0, 1e-6);
        BOOST_CHECK_EQUAL(substring_points_1_5.has_directions(), false);

        auto substring_points_1_6 = map_matching_2::geometry::substring_points(
                line_1, point_type{3, 0}, point_type{2, 0});
        BOOST_CHECK_EQUAL(substring_points_1_6.size(), 0);
        BOOST_CHECK_EQUAL(substring_points_1_6.has_length(), false);

        auto substring_points_1_7 = map_matching_2::geometry::substring_points(
                line_1, point_type{5, 4}, point_type{1, 0});
        BOOST_CHECK_EQUAL(substring_points_1_7.size(), 0);
        BOOST_CHECK_EQUAL(substring_points_1_7.has_length(), false);

        auto substring_points_1_8 = map_matching_2::geometry::substring_points(
                line_1, point_type{2, 1}, point_type{6, 4});
        BOOST_CHECK_EQUAL(substring_points_1_8.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_8.length(), 7.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_8.directions(), 90.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_8.absolute_directions(), 90.0, 1e-6);

        auto substring_points_1_9 = map_matching_2::geometry::substring_points(
                line_1, point_type{2, 1}, point_type{2.00000000000001, 1}, true, true);
        BOOST_CHECK_EQUAL(substring_points_1_9.size(), 2);
        BOOST_CHECK_LE(substring_points_1_9.length(), 1e-6);

        auto substring_points_1_1_ni = map_matching_2::geometry::substring_points(
                line_1, point_type{3, 0}, point_type{5, 3}, true, true);
        BOOST_CHECK_EQUAL(substring_points_1_1_ni.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_1_ni.length(), 5.0, 1e-6);

        auto substring_points_1_2_ni = map_matching_2::geometry::substring_points(
                line_1, point_type{3, 0}, point_type{5, 3}, false, true);
        BOOST_CHECK_EQUAL(substring_points_1_2_ni.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_2_ni.length(), 5.0, 1e-6);

        auto substring_points_1_3_ni = map_matching_2::geometry::substring_points(
                line_1, point_type{3, 0}, point_type{5, 3}, true, false);
        BOOST_CHECK_EQUAL(substring_points_1_3_ni.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_3_ni.length(), 5.0, 1e-6);

        auto substring_points_1_4_ni = map_matching_2::geometry::substring_points(
                line_1, point_type{3, 0}, point_type{5, 3}, false, false);
        BOOST_CHECK_EQUAL(substring_points_1_4_ni.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_4_ni.length(), 5.0, 1e-6);

        auto substring_points_1_1_d = map_matching_2::geometry::substring(line_1, 0.0L, 10.0L);
        BOOST_CHECK_EQUAL(substring_points_1_1_d.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_1_d.length(), 10.0, 1e-6);

        auto substring_points_1_2_d = map_matching_2::geometry::substring(line_1, 0.0L, 11.0L);
        BOOST_CHECK_EQUAL(substring_points_1_2_d.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_2_d.length(), 10.0, 1e-6);

        auto substring_points_1_3_d = map_matching_2::geometry::substring(line_1, -1.0L, 10.0L);
        BOOST_CHECK_EQUAL(substring_points_1_3_d.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_3_d.length(), 10.0, 1e-6);

        auto substring_points_1_4_d = map_matching_2::geometry::substring(line_1, -1.0L, 11.0L);
        BOOST_CHECK_EQUAL(substring_points_1_4_d.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_4_d.length(), 10.0, 1e-6);

        auto substring_points_1_5_d = map_matching_2::geometry::substring(line_1, 1.0L, 9.0L);
        BOOST_CHECK_EQUAL(substring_points_1_5_d.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_5_d.length(), 8.0, 1e-6);

        auto substring_points_1_6_d = map_matching_2::geometry::substring(line_1, 9.0L, 1.0L);
        BOOST_CHECK_EQUAL(substring_points_1_6_d.size(), 0);
        BOOST_CHECK_EQUAL(substring_points_1_6_d.has_length(), false);

        auto substring_points_1_7_d = map_matching_2::geometry::substring(line_1, 7.0L, 9.0L);
        BOOST_CHECK_EQUAL(substring_points_1_7_d.size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_1_7_d.length(), 2.0, 1e-6);

        auto line_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 2, 0, 2, 1, 1, 1, 1, 0, 3, 0})};
        BOOST_CHECK_EQUAL(line_2.size(), 6);
        BOOST_CHECK_CLOSE_FRACTION(line_2.length(), 7.0, 1e-6);

        auto substring_points_2_1 = map_matching_2::geometry::substring_points(
                line_2, point_type{-1, -1}, point_type{3, 1});
        BOOST_CHECK_EQUAL(substring_points_2_1.size(), 6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_1.length(), 7.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_1.azimuth(), 90.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_1.directions(), 360.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_1.absolute_directions(), 360, 1e-6);

        auto substring_points_2_2 = map_matching_2::geometry::substring_points(
                line_2, point_type{2, -1}, point_type{1, -1});
        BOOST_CHECK_EQUAL(substring_points_2_2.size(), 4);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_2.length(), 3.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_2.azimuth(), -90.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_2.directions(), 180.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_2.absolute_directions(), 180, 1e-6);

        auto substring_points_2_3 = map_matching_2::geometry::substring_points(
                line_2, point_type{1, -1}, point_type{3, 4});
        BOOST_CHECK_EQUAL(substring_points_2_3.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_3.length(), 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_3.azimuth(), 45.0, 1e-6);

        auto substring_points_2_4 = map_matching_2::geometry::substring_points(
                line_2, point_type{4, 5}, point_type{1, -3});
        BOOST_CHECK_EQUAL(substring_points_2_4.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_4.length(), 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_4.azimuth(), -135.0, 1e-6);

        auto substring_points_2_5 = map_matching_2::geometry::substring_points(
                line_2, point_type{3, 4}, point_type{0, -3});
        BOOST_CHECK_EQUAL(substring_points_2_5.size(), 0);
        BOOST_CHECK_EQUAL(substring_points_2_5.has_length(), false);

        auto substring_points_2_6 = map_matching_2::geometry::substring_points(
                line_2, point_type{3, 4}, point_type{3, 5});
        BOOST_CHECK_EQUAL(substring_points_2_6.size(), 0);
        BOOST_CHECK_EQUAL(substring_points_2_6.has_length(), false);

        auto substring_points_2_7 = map_matching_2::geometry::substring_points(
                line_2, point_type{1.9, 0.0}, point_type{1.1, 0.0});
        BOOST_CHECK_EQUAL(substring_points_2_7.size(), 6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_7.length(), 3.2, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_7.azimuth(), -90.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_7.directions(), 360.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(substring_points_2_7.absolute_directions(), 360, 1e-6);
    }

BOOST_AUTO_TEST_SUITE_END()
