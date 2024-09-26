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

#include <boost/test/unit_test.hpp>

#include "types/geometry/models.hpp"
#include "types/geometry/rich_type/rich_line.hpp"
#include "types/geometry/rich_type/eager_rich_line.hpp"
#include "types/geometry/rich_type/lazy_rich_line.hpp"
#include "types/geometry/network/route/route.hpp"
#include "types/geometry/network/route/eager_route.hpp"
#include "types/geometry/network/route/lazy_route.hpp"

#include "helper/geometry_helper.hpp"

using point_type = map_matching_2::geometry::point_type<map_matching_2::geometry::cs_cartesian>;
using line_type = map_matching_2::geometry::linestring_type<point_type>;

BOOST_AUTO_TEST_SUITE(route_tests)

    BOOST_AUTO_TEST_CASE(rich_line_merge_test) {
        auto test_routes = []<typename Route>() {
            using route_type = Route;

            auto test_rich_lines = []<typename RichLine>() {
                using rich_line_type = RichLine;

                rich_line_type line_empty{};
                rich_line_type line_1{create_line<point_type, line_type>({0, 0, 1, 0})};
                rich_line_type line_2{create_line<point_type, line_type>({1, 0, 2, 0})};
                rich_line_type line_3{create_line<point_type, line_type>({0, 0, 1, 0, 1, 1, 2, 1, 2, 2})};
                rich_line_type line_4{create_line<point_type, line_type>({2, 2, 3, 2, 3, 1, 4, 1, 4, 0})};
                rich_line_type line_5{create_line<point_type, line_type>({4, 0, 5, 0})};

                BOOST_CHECK_EQUAL(line_empty.has_length(), false);
                BOOST_CHECK_EQUAL(line_empty.has_azimuth(), false);
                BOOST_CHECK_EQUAL(line_empty.has_directions(), false);

                BOOST_CHECK_EQUAL(line_1.has_length(), true);
                BOOST_CHECK_EQUAL(line_1.has_azimuth(), true);
                BOOST_CHECK_EQUAL(line_1.has_directions(), false);
                BOOST_CHECK_CLOSE_FRACTION(line_1.length(), 1.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(line_1.azimuth(), 90.0, 1e-6);

                BOOST_CHECK_EQUAL(line_3.has_length(), true);
                BOOST_CHECK_EQUAL(line_3.has_azimuth(), true);
                BOOST_CHECK_EQUAL(line_3.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(line_3.length(), 4.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(line_3.azimuth(), 45.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(line_3.directions(), 90.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(line_3.absolute_directions(), 270.0, 1e-6);

                rich_line_type line_merge_1_1{line_empty};
                rich_line_type line_merge_1_2{line_1};
                auto line_merge_result_1 = line_merge_1_1.template merge<rich_line_type>({line_merge_1_2});
                BOOST_CHECK_EQUAL(line_merge_result_1.has_length(), true);
                BOOST_CHECK_EQUAL(line_merge_result_1.has_azimuth(), true);
                BOOST_CHECK_EQUAL(line_merge_result_1.has_directions(), false);
                BOOST_CHECK_CLOSE_FRACTION(line_merge_result_1.length(), 1.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(line_merge_result_1.azimuth(), 90.0, 1e-6);

                rich_line_type line_merge_2_1{line_1};
                rich_line_type line_merge_2_2{line_2};
                auto line_merge_result_2 = line_merge_2_1.template merge<rich_line_type>({line_merge_2_2});
                BOOST_CHECK_EQUAL(line_merge_result_2.has_length(), true);
                BOOST_CHECK_EQUAL(line_merge_result_2.has_azimuth(), true);
                BOOST_CHECK_EQUAL(line_merge_result_2.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(line_merge_result_2.length(), 2.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(line_merge_result_2.azimuth(), 90.0, 1e-6);
                BOOST_CHECK_LE(line_merge_result_2.directions(), 1e-6);
                BOOST_CHECK_LE(line_merge_result_2.absolute_directions(), 1e-6);

                rich_line_type line_merge_3_1{line_3};
                rich_line_type line_merge_3_2{line_empty};
                rich_line_type line_merge_3_3{line_4};
                rich_line_type line_merge_3_4{line_5};
                auto line_merge_result_3 = line_merge_3_1.template merge<rich_line_type>(
                        {line_merge_3_2, line_merge_3_3, line_merge_3_4});
                BOOST_CHECK_EQUAL(line_merge_result_3.has_length(), true);
                BOOST_CHECK_EQUAL(line_merge_result_3.has_azimuth(), true);
                BOOST_CHECK_EQUAL(line_merge_result_3.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(line_merge_result_3.length(), 9.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(line_merge_result_3.azimuth(), 90.0, 1e-6);
                BOOST_CHECK_LE(line_merge_result_3.directions(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(line_merge_result_3.absolute_directions(), 8 * 90.0, 1e-6);

                rich_line_type line_merge_4_1{line_1};
                rich_line_type line_merge_4_2{line_empty};
                rich_line_type line_merge_4_3{line_3};
                BOOST_CHECK_THROW(
                        line_merge_4_1.template merge<rich_line_type>({line_merge_4_2, line_merge_4_3}),
                        map_matching_2::geometry::rich_line_merge_exception);
            };

            test_rich_lines.template operator()<map_matching_2::geometry::rich_line_type<point_type>>();
            test_rich_lines.template operator()<map_matching_2::geometry::eager_rich_line_type<point_type>>();
            test_rich_lines.template operator()<map_matching_2::geometry::lazy_rich_line_type<point_type>>();
        };

        test_routes.template operator()<map_matching_2::geometry::network::route_type<
            map_matching_2::geometry::cs_cartesian>>();
        test_routes.template operator()<map_matching_2::geometry::network::eager_route_type<
            map_matching_2::geometry::cs_cartesian>>();
        test_routes.template operator()<map_matching_2::geometry::network::lazy_route_type<
            map_matching_2::geometry::cs_cartesian>>();
    }

    BOOST_AUTO_TEST_CASE(route_combination_test) {
        auto test_routes = []<typename Route>() {
            using route_type = Route;

            auto test_rich_lines = []<typename RichLine>() {
                using rich_line_type = RichLine;

                const auto route_1 = route_type{rich_line_type{create_line<point_type, line_type>({0, 0, 1, 0})}};
                const auto route_2 = route_type{rich_line_type{create_line<point_type, line_type>({1, 0, 1, 1})}};
                const auto route_3 = route_type{rich_line_type{create_line<point_type, line_type>({1, 0, 2, 0})}};
                const auto route_4 = route_type{rich_line_type{create_line<point_type, line_type>({1, 0, 1, -1})}};
                const auto route_5 = route_type{rich_line_type{create_line<point_type, line_type>({1, 0, 0, 0})}};
                const auto route_6 = route_type{rich_line_type{create_line<point_type, line_type>({0, 1, 1, 1})}};

                const auto route_empty = route_type{false};
                const auto route_invalid = route_type{true};

                auto route_1_1 = route_type{route_1};
                auto route_1_2 = route_type{route_2};
                auto result_1 = route_type::template merge<rich_line_type>({route_1_1, route_1_2});
                BOOST_CHECK_EQUAL(result_1.has_length(), true);
                BOOST_CHECK_EQUAL(result_1.has_azimuth(), true);
                BOOST_CHECK_EQUAL(result_1.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(result_1.length(), 2.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_1.azimuth(), 45.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_1.directions(), 90.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_1.absolute_directions(), 90.0, 1e-6);

                auto route_2_1 = route_type{route_1};
                auto route_2_2 = route_type{route_3};
                auto result_2 = route_type::template merge<rich_line_type>({route_2_1, route_2_2});
                BOOST_CHECK_EQUAL(result_2.has_length(), true);
                BOOST_CHECK_EQUAL(result_2.has_azimuth(), true);
                BOOST_CHECK_EQUAL(result_2.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(result_2.length(), 2.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_2.azimuth(), 90.0, 1e-6);
                BOOST_CHECK_LE(std::fabs(result_2.directions()), 1e-6);
                BOOST_CHECK_LE(result_2.absolute_directions(), 1e-6);

                auto route_3_1 = route_type{route_1};
                auto route_3_2 = route_type{route_4};
                auto result_3 = route_type::template merge<rich_line_type>({route_3_1, route_3_2});
                BOOST_CHECK_EQUAL(result_3.has_length(), true);
                BOOST_CHECK_EQUAL(result_3.has_azimuth(), true);
                BOOST_CHECK_EQUAL(result_3.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(result_3.length(), 2.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_3.azimuth(), 135.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_3.directions(), -90.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_3.absolute_directions(), 90.0, 1e-6);

                auto route_4_1 = route_type{route_1};
                auto route_4_2 = route_type{route_5};
                auto result_4 = route_type::template merge<rich_line_type>({route_4_1, route_4_2});
                BOOST_CHECK_EQUAL(result_4.has_length(), true);
                BOOST_CHECK_EQUAL(result_4.has_azimuth(), false);
                BOOST_CHECK_EQUAL(result_4.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(result_4.length(), 2.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(std::fabs(result_4.directions()), 180.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_4.absolute_directions(), 180.0, 1e-6);

                auto route_5_1 = route_type{route_1};
                auto route_5_2 = route_type{route_empty};
                auto result_5 = route_type::template merge<rich_line_type>({route_5_1, route_5_2});
                BOOST_CHECK_EQUAL(result_5.has_length(), true);
                BOOST_CHECK_EQUAL(result_5.has_azimuth(), true);
                BOOST_CHECK_EQUAL(result_5.has_directions(), false);
                BOOST_CHECK_CLOSE_FRACTION(result_5.length(), route_1.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_5.azimuth(), route_1.azimuth(), 1e-6);

                auto route_6_1 = route_type{route_empty};
                auto route_6_2 = route_type{route_2};
                auto result_6 = route_type::template merge<rich_line_type>({route_6_1, route_6_2});
                BOOST_CHECK_EQUAL(result_6.has_length(), true);
                BOOST_CHECK_EQUAL(result_6.has_azimuth(), true);
                BOOST_CHECK_EQUAL(result_6.has_directions(), false);
                BOOST_CHECK_CLOSE_FRACTION(result_6.length(), route_2.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_6.azimuth(), route_2.azimuth(), 1e-6);

                auto route_7_1 = route_type{route_3};
                auto route_7_2 = route_type{route_invalid};
                auto result_7 = route_type::template merge<rich_line_type>({route_7_1, route_7_2});
                BOOST_CHECK_EQUAL(result_7.has_length(), true);
                BOOST_CHECK_EQUAL(result_7.has_azimuth(), true);
                BOOST_CHECK_EQUAL(result_7.has_directions(), false);
                BOOST_CHECK_CLOSE_FRACTION(result_7.length(), route_3.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_7.azimuth(), route_3.azimuth(), 1e-6);

                auto route_8_1 = route_type{route_invalid};
                auto route_8_2 = route_type{route_4};
                auto result_8 = route_type::template merge<rich_line_type>({route_8_1, route_8_2});
                BOOST_CHECK_EQUAL(result_8.has_length(), true);
                BOOST_CHECK_EQUAL(result_8.has_azimuth(), true);
                BOOST_CHECK_EQUAL(result_8.has_directions(), false);
                BOOST_CHECK_CLOSE_FRACTION(result_8.length(), route_4.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(result_8.azimuth(), route_4.azimuth(), 1e-6);

                auto route_9_1 = route_type{route_1};
                auto route_9_2 = route_type{route_6};
                auto result_9 = route_type::template merge<rich_line_type>({route_9_1, route_9_2});
                BOOST_CHECK_EQUAL(result_9.is_connected(), false);

                auto route_10_1 = route_type{route_invalid};
                auto route_10_2 = route_type{route_invalid};
                auto result_10 = route_type::template merge<rich_line_type>({route_10_1, route_10_2});
                BOOST_CHECK_EQUAL(result_10.is_invalid(), true);

                auto route_11_1 = route_type{route_empty};
                auto route_11_2 = route_type{route_empty};
                auto result_11 = route_type::template merge<rich_line_type>({route_11_1, route_11_2});
                BOOST_CHECK_EQUAL(result_11.is_invalid(), false);
                BOOST_CHECK_EQUAL(result_11.has_length(), false);
                BOOST_CHECK_EQUAL(result_11.has_azimuth(), false);
                BOOST_CHECK_EQUAL(result_11.has_directions(), false);
            };

            test_rich_lines.template operator()<map_matching_2::geometry::rich_line_type<point_type>>();
            test_rich_lines.template operator()<map_matching_2::geometry::eager_rich_line_type<point_type>>();
            test_rich_lines.template operator()<map_matching_2::geometry::lazy_rich_line_type<point_type>>();
        };

        test_routes.template operator()<map_matching_2::geometry::network::route_type<
            map_matching_2::geometry::cs_cartesian>>();
        test_routes.template operator()<map_matching_2::geometry::network::eager_route_type<
            map_matching_2::geometry::cs_cartesian>>();
        test_routes.template operator()<map_matching_2::geometry::network::lazy_route_type<
            map_matching_2::geometry::cs_cartesian>>();
    }

    BOOST_AUTO_TEST_CASE(route_return_test) {
        auto test_routes = []<typename Route>() {
            using route_type = Route;

            auto test_rich_lines = []<typename RichLine>() {
                using rich_line_type = RichLine;

                using rich_line_variant_type = typename route_type::rich_line_variant_type;
                using return_line_parts_type = typename map_matching_2::geometry::network::route_return_line_parts;

                const auto route_1 = route_type{
                        rich_line_type{
                                create_line<point_type, line_type>(
                                        {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0})
                        }
                };
                const auto route_2 = route_type{
                        rich_line_type{
                                create_line<point_type, line_type>({3, 0, 2, 0, 1, 0, 0, 0})
                        }
                };
                const auto route_3 = route_type{rich_line_type{create_line<point_type, line_type>({9, 0, 8, 0, 7, 0})}};

                auto search_1 = route_1.find_return_line(route_2);
                auto expectation_1 = return_line_parts_type{};
                BOOST_CHECK_EQUAL(search_1, expectation_1);

                auto search_2 = route_2.find_return_line(route_1);
                auto expectation_2 = return_line_parts_type{0, 0, 0, 3, 0, 0, 0, 3};
                BOOST_CHECK_EQUAL(search_2, expectation_2);

                auto search_3 = route_1.find_return_line(route_3);
                auto expectation_3 = return_line_parts_type{0, 7, 0, 9, 0, 0, 0, 2};
                BOOST_CHECK_EQUAL(search_3, expectation_3);

                auto search_4 = route_3.find_return_line(route_1);
                auto expectation_4 = return_line_parts_type{};
                BOOST_CHECK_EQUAL(search_4, expectation_4);

                auto return_1 = route_2.template extract_return_line<rich_line_type>(route_1);
                BOOST_CHECK_EQUAL(return_1.has_length(), true);
                BOOST_CHECK_EQUAL(return_1.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_1.length(), route_2.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(return_1.azimuth(), route_2.azimuth(), 1e-6);

                auto return_2 = route_2.template extract_return_line<rich_line_type>(route_1, false);
                BOOST_CHECK_EQUAL(return_2.has_length(), true);
                BOOST_CHECK_EQUAL(return_2.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_2.length(), route_2.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(return_2.azimuth(), route_1.azimuth(), 1e-6);

                auto return_3 = route_1.template extract_return_line<rich_line_type>(route_3);
                BOOST_CHECK_EQUAL(return_3.has_length(), true);
                BOOST_CHECK_EQUAL(return_3.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_3.length(), route_3.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(return_3.azimuth(), route_1.azimuth(), 1e-6);

                auto return_4 = route_1.template extract_return_line<rich_line_type>(route_3, false);
                BOOST_CHECK_EQUAL(return_4.has_length(), true);
                BOOST_CHECK_EQUAL(return_4.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_4.length(), route_3.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(return_4.azimuth(), route_3.azimuth(), 1e-6);

                auto trim_1 = route_2.template trim_merge<rich_line_type>(route_1);
                BOOST_CHECK_EQUAL(trim_1.has_length(), true);
                BOOST_CHECK_EQUAL(trim_1.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_1.length(), route_1.length() - route_2.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_1.azimuth(), route_1.azimuth(), 1e-6);

                auto trim_2 = route_1.template trim_merge<rich_line_type>(route_3);
                BOOST_CHECK_EQUAL(trim_2.has_length(), true);
                BOOST_CHECK_EQUAL(trim_2.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_2.length(), route_1.length() - route_3.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_2.azimuth(), route_1.azimuth(), 1e-6);

                auto trim_3 = route_2.template trim_merge<rich_line_type>(route_3);
                BOOST_CHECK_EQUAL(trim_3.is_invalid(), true);

                const auto route_4 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({0, 0, 1, 0})},
                                rich_line_type{create_line<point_type, line_type>({1, 0, 2, 0})},
                                rich_line_type{create_line<point_type, line_type>({2, 0, 3, 0})},
                                rich_line_type{create_line<point_type, line_type>({3, 0, 4, 0})},
                                rich_line_type{create_line<point_type, line_type>({4, 0, 5, 0})},
                                rich_line_type{create_line<point_type, line_type>({5, 0, 6, 0})},
                                rich_line_type{create_line<point_type, line_type>({6, 0, 7, 0})},
                                rich_line_type{create_line<point_type, line_type>({7, 0, 8, 0})},
                                rich_line_type{create_line<point_type, line_type>({8, 0, 9, 0})}
                        }
                };
                const auto route_5 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({4, 0, 3, 0})},
                                rich_line_type{create_line<point_type, line_type>({3, 0, 2, 0})},
                                rich_line_type{create_line<point_type, line_type>({2, 0, 1, 0})},
                                rich_line_type{create_line<point_type, line_type>({1, 0, 0, 0})}
                        }
                };
                const auto route_6 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({9, 0, 8, 0})},
                                rich_line_type{create_line<point_type, line_type>({8, 0, 7, 0})},
                                rich_line_type{create_line<point_type, line_type>({7, 0, 6, 0})}
                        }
                };

                auto search_5 = route_4.find_return_line(route_5);
                auto expectation_5 = return_line_parts_type{};
                BOOST_CHECK_EQUAL(search_5, expectation_5);

                auto search_6 = route_5.find_return_line(route_4);
                auto expectation_6 = return_line_parts_type{0, 0, 3, 1, 0, 0, 3, 1};
                BOOST_CHECK_EQUAL(search_6, expectation_6);

                auto search_7 = route_4.find_return_line(route_6);
                auto expectation_7 = return_line_parts_type{5, 1, 8, 1, 0, 0, 2, 1};
                BOOST_CHECK_EQUAL(search_7, expectation_7);

                auto search_8 = route_6.find_return_line(route_4);
                auto expectation_8 = return_line_parts_type{};
                BOOST_CHECK_EQUAL(search_8, expectation_8);

                auto return_5 = route_5.template extract_return_line<rich_line_type>(route_4);
                BOOST_CHECK_EQUAL(return_5.has_length(), true);
                BOOST_CHECK_EQUAL(return_5.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_5.length(), route_5.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(return_5.azimuth(), route_5.azimuth(), 1e-6);

                auto return_6 = route_5.template extract_return_line<rich_line_type>(route_4, false);
                BOOST_CHECK_EQUAL(return_6.has_length(), true);
                BOOST_CHECK_EQUAL(return_6.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_6.length(), route_5.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(return_6.azimuth(), route_4.azimuth(), 1e-6);

                auto return_7 = route_4.template extract_return_line<rich_line_type>(route_6);
                BOOST_CHECK_EQUAL(return_7.has_length(), true);
                BOOST_CHECK_EQUAL(return_7.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_7.length(), route_6.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(return_7.azimuth(), route_4.azimuth(), 1e-6);

                auto return_8 = route_4.template extract_return_line<rich_line_type>(route_6, false);
                BOOST_CHECK_EQUAL(return_8.has_length(), true);
                BOOST_CHECK_EQUAL(return_8.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_8.length(), route_6.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(return_8.azimuth(), route_6.azimuth(), 1e-6);

                auto trim_4 = route_5.template trim_merge<rich_line_type>(route_4);
                BOOST_CHECK_EQUAL(trim_4.has_length(), true);
                BOOST_CHECK_EQUAL(trim_4.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_4.length(), route_4.length() - route_5.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_4.azimuth(), route_4.azimuth(), 1e-6);

                auto trim_5 = route_4.template trim_merge<rich_line_type>(route_6);
                BOOST_CHECK_EQUAL(trim_5.has_length(), true);
                BOOST_CHECK_EQUAL(trim_5.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_5.length(), route_4.length() - route_6.length(), 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_5.azimuth(), route_4.azimuth(), 1e-6);

                auto trim_6 = route_5.template trim_merge<rich_line_type>(route_6);
                BOOST_CHECK_EQUAL(trim_6.is_invalid(), true);

                const auto route_7 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({0, 0, 2, 0})},
                                rich_line_type{create_line<point_type, line_type>({2, 0, 4, 0, 6, 0})},
                                rich_line_type{create_line<point_type, line_type>({6, 0, 9, 0})}
                        }
                };
                const auto route_8 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({4, 3, 4, 1, 4, 0})},
                                rich_line_type{create_line<point_type, line_type>({4, 0, 2, 0, 0, 0})}
                        }
                };

                auto trim_7 = route_8.template trim_merge<rich_line_type>(route_7);
                BOOST_CHECK_EQUAL(trim_7.has_length(), true);
                BOOST_CHECK_EQUAL(trim_7.has_azimuth(), true);
                BOOST_CHECK_EQUAL(trim_7.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_7.length(), 8.0, 1e-6);
                BOOST_CHECK(trim_7.azimuth() > 90 and trim_7.azimuth() < 180);
                BOOST_CHECK_CLOSE_FRACTION(trim_7.directions(), 90.0, 1e-6);

                const auto route_9 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({5, 3, 5, 1, 5, 0})},
                                rich_line_type{create_line<point_type, line_type>({5, 0, 4, 0})},
                                rich_line_type{create_line<point_type, line_type>({4, 0, 2, 0, 0, 0})}
                        }
                };

                auto trim_8 = route_9.template trim_merge<rich_line_type>(route_7);
                BOOST_CHECK_EQUAL(trim_8.has_length(), true);
                BOOST_CHECK_EQUAL(trim_8.has_azimuth(), true);
                BOOST_CHECK_EQUAL(trim_8.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_8.length(), 7.0, 1e-6);
                BOOST_CHECK(trim_8.azimuth() > 90 and trim_8.azimuth() < 180);
                BOOST_CHECK_CLOSE_FRACTION(trim_8.directions(), 90.0, 1e-6);

                const auto route_10 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({5, 4, 5, 3})},
                                rich_line_type{create_line<point_type, line_type>({5, 3, 5, 1, 5, 0, 4, 0})},
                                rich_line_type{create_line<point_type, line_type>({4, 0, 2, 0, 0, 0})}
                        }
                };

                auto trim_9 = route_10.template trim_merge<rich_line_type>(route_7);
                BOOST_CHECK_EQUAL(trim_9.has_length(), true);
                BOOST_CHECK_EQUAL(trim_9.has_azimuth(), true);
                BOOST_CHECK_EQUAL(trim_9.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_9.length(), 8.0, 1e-6);
                BOOST_CHECK(trim_9.azimuth() > 90 and trim_9.azimuth() < 180);
                BOOST_CHECK_CLOSE_FRACTION(trim_9.directions(), 90.0, 1e-6);

                auto return_9 = route_10.template extract_return_line<rich_line_type>(route_7);
                BOOST_CHECK_EQUAL(return_9.has_length(), true);
                BOOST_CHECK_EQUAL(return_9.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(return_9.length(), 4.0, 1e-6);

                const auto route_11 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({4, 4, 4, 3})},
                                rich_line_type{create_line<point_type, line_type>({4, 3, 4, 1})},
                                rich_line_type{create_line<point_type, line_type>({4, 0, 2, 0, 0, 0})}
                        }
                };

                auto trim_10 = route_11.template trim_merge<rich_line_type>(route_7);
                BOOST_CHECK_EQUAL(trim_10.is_connected(), false);
                BOOST_CHECK_EQUAL(trim_10.has_length(), true);
                BOOST_CHECK_EQUAL(trim_10.has_azimuth(), true);
                BOOST_CHECK_EQUAL(trim_10.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_10.length(), 8.0, 1e-6);
                BOOST_CHECK(trim_10.azimuth() > 90 and trim_10.azimuth() < 180);
                BOOST_CHECK_LE(trim_10.directions(), 1e-6);

                const auto route_12 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({5, 0, 4, 0, 2, 0, 0, 0})}
                        }
                };

                auto trim_11 = route_12.template trim_merge<rich_line_type>(route_7);
                BOOST_CHECK_EQUAL(trim_11.has_length(), true);
                BOOST_CHECK_EQUAL(trim_11.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_11.length(), 4.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_11.azimuth(), route_7.azimuth(), 1e-6);

                auto trim_12 = route_5.template trim_merge<rich_line_type>(route_4);
                trim_12 = trim_12.template trim_merge<rich_line_type>(route_6);
                BOOST_CHECK_EQUAL(trim_12.has_length(), true);
                BOOST_CHECK_EQUAL(trim_12.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_12.length(), 2.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_12.azimuth(), route_4.azimuth(), 1e-6);

                const auto route_13 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({0, 3, 0, 1, 0, 0})}
                        }
                };

                auto trim_13 = route_13.template trim_merge<rich_line_type>(route_7);
                BOOST_CHECK_EQUAL(trim_13.has_length(), true);
                BOOST_CHECK_EQUAL(trim_13.has_azimuth(), true);
                BOOST_CHECK_EQUAL(trim_13.has_directions(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_13.length(), 12.0, 1e-6);
                BOOST_CHECK(trim_13.azimuth() > 90 and trim_13.azimuth() < 180);
                BOOST_CHECK_CLOSE_FRACTION(trim_13.directions(), 90.0, 1e-6);

                auto return_10 = route_13.template extract_return_line<rich_line_type>(route_7);
                BOOST_CHECK_EQUAL(return_10.has_length(), false);

                const auto route_14 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({1, 2, 2, 2})},
                                rich_line_type{create_line<point_type, line_type>({2, 2, 3, 2})}
                        }
                };

                const auto route_15 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({2, 2, 1, 2})}
                        }
                };

                const auto route_16 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({3, 2, 2, 2})}
                        }
                };

                auto trim_14 = route_15.template trim_merge<rich_line_type>(route_14);
                BOOST_CHECK_EQUAL(trim_14.has_length(), true);
                BOOST_CHECK_EQUAL(trim_14.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_14.length(), 1.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_14.azimuth(), route_14.azimuth(), 1e-6);

                auto trim_15 = route_14.template trim_merge<rich_line_type>(route_16);
                BOOST_CHECK_EQUAL(trim_15.has_length(), true);
                BOOST_CHECK_EQUAL(trim_15.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_15.length(), 1.0, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_15.azimuth(), route_14.azimuth(), 1e-6);

                auto trim_16 = trim_14.template trim_merge<rich_line_type>(route_16);
                BOOST_CHECK_EQUAL(trim_16.is_invalid(), false);
                BOOST_CHECK_EQUAL(trim_16.has_length(), false);
                BOOST_CHECK_EQUAL(trim_16.has_azimuth(), false);

                const auto route_17 = route_type{
                        std::list<rich_line_variant_type>{
                                rich_line_type{create_line<point_type, line_type>({2, 2, 2.2, 2})}
                        }
                };

                auto trim_17 = route_16.template trim_merge<rich_line_type>(route_17);
                BOOST_CHECK_EQUAL(trim_17.has_length(), true);
                BOOST_CHECK_EQUAL(trim_17.has_azimuth(), true);
                BOOST_CHECK_CLOSE_FRACTION(trim_17.length(), 0.8, 1e-6);
                BOOST_CHECK_CLOSE_FRACTION(trim_17.azimuth(), route_16.azimuth(), 1e-6);
            };

            test_rich_lines.template operator()<map_matching_2::geometry::rich_line_type<point_type>>();
            test_rich_lines.template operator()<map_matching_2::geometry::eager_rich_line_type<point_type>>();
            test_rich_lines.template operator()<map_matching_2::geometry::lazy_rich_line_type<point_type>>();
        };

        test_routes.template operator()<map_matching_2::geometry::network::route_type<
            map_matching_2::geometry::cs_cartesian>>();
        test_routes.template operator()<map_matching_2::geometry::network::eager_route_type<
            map_matching_2::geometry::cs_cartesian>>();
        test_routes.template operator()<map_matching_2::geometry::network::lazy_route_type<
            map_matching_2::geometry::cs_cartesian>>();
    }

BOOST_AUTO_TEST_SUITE_END()
