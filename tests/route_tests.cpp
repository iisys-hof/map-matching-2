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

#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include <geometry/rich_line.hpp>

#include "helper/matcher_fixture.hpp"

BOOST_AUTO_TEST_SUITE(route_tests)

    BOOST_FIXTURE_TEST_CASE(rich_line_merge_test, matcher_fixture<matcher_metric>) {
        using line_type = typename matcher_metric::matcher_static::line_type;
        using rich_line_type = typename matcher_metric::matcher_static::rich_line_type;

        network_fixture<network_metric> network_fixture;

        network_metric network;
        auto network_matcher = prepare_network(network);
        auto matcher = create_matcher(network_matcher);

        rich_line_type line_empty{};
        rich_line_type line_1{network_fixture.create_line({0, 0, 1, 0})};
        rich_line_type line_2{network_fixture.create_line({1, 0, 2, 0})};
        rich_line_type line_3{network_fixture.create_line({0, 0, 1, 0, 1, 1, 2, 1, 2, 2})};
        rich_line_type line_4{network_fixture.create_line({2, 2, 3, 2, 3, 1, 4, 1, 4, 0})};
        rich_line_type line_5{network_fixture.create_line({4, 0, 5, 0})};

        BOOST_CHECK_EQUAL(line_empty.has_length, false);
        BOOST_CHECK_EQUAL(line_empty.has_azimuth, false);
        BOOST_CHECK_EQUAL(line_empty.has_directions, false);

        BOOST_CHECK_EQUAL(line_1.has_length, true);
        BOOST_CHECK_EQUAL(line_1.has_azimuth, true);
        BOOST_CHECK_EQUAL(line_1.has_directions, false);
        BOOST_CHECK_CLOSE_FRACTION(line_1.length, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line_1.azimuth, 90.0, 1e-6);

        BOOST_CHECK_EQUAL(line_3.has_length, true);
        BOOST_CHECK_EQUAL(line_3.has_azimuth, true);
        BOOST_CHECK_EQUAL(line_3.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(line_3.length, 4.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line_3.azimuth, 45.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line_3.directions, 90.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line_3.absolute_directions, 270.0, 1e-6);

        rich_line_type line_merge_1_1{line_empty};
        rich_line_type line_merge_1_2{line_1};
        auto line_merge_result_1 = rich_line_type::merge({line_merge_1_1, line_merge_1_2});
        BOOST_CHECK_EQUAL(line_merge_result_1.has_length, true);
        BOOST_CHECK_EQUAL(line_merge_result_1.has_azimuth, true);
        BOOST_CHECK_EQUAL(line_merge_result_1.has_directions, false);
        BOOST_CHECK_CLOSE_FRACTION(line_merge_result_1.length, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line_merge_result_1.azimuth, 90.0, 1e-6);

        rich_line_type line_merge_2_1{line_1};
        rich_line_type line_merge_2_2{line_2};
        auto line_merge_result_2 = rich_line_type::merge({line_merge_2_1, line_merge_2_2});
        BOOST_CHECK_EQUAL(line_merge_result_2.has_length, true);
        BOOST_CHECK_EQUAL(line_merge_result_2.has_azimuth, true);
        BOOST_CHECK_EQUAL(line_merge_result_2.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(line_merge_result_2.length, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line_merge_result_2.azimuth, 90.0, 1e-6);
        BOOST_CHECK_LE(line_merge_result_2.directions, 1e-6);
        BOOST_CHECK_LE(line_merge_result_2.absolute_directions, 1e-6);

        rich_line_type line_merge_3_1{line_3};
        rich_line_type line_merge_3_2{line_empty};
        rich_line_type line_merge_3_3{line_4};
        rich_line_type line_merge_3_4{line_5};
        auto line_merge_result_3 = rich_line_type::merge(
                {line_merge_3_1, line_merge_3_2, line_merge_3_3, line_merge_3_4});
        BOOST_CHECK_EQUAL(line_merge_result_3.has_length, true);
        BOOST_CHECK_EQUAL(line_merge_result_3.has_azimuth, true);
        BOOST_CHECK_EQUAL(line_merge_result_3.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(line_merge_result_3.length, 9.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line_merge_result_3.azimuth, 90.0, 1e-6);
        BOOST_CHECK_LE(line_merge_result_3.directions, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line_merge_result_3.absolute_directions, 8 * 90.0, 1e-6);

        rich_line_type line_merge_4_1{line_1};
        rich_line_type line_merge_4_2{line_empty};
        rich_line_type line_merge_4_3{line_3};
        BOOST_CHECK_THROW(rich_line_type::merge({line_merge_4_1, line_merge_4_2, line_merge_4_3}),
                          map_matching_2::geometry::rich_line_merge_exception);
    }

    BOOST_FIXTURE_TEST_CASE(route_combination_test, matcher_fixture<matcher_metric>) {
        using line_type = typename matcher_metric::matcher_static::line_type;
        using route_type = typename matcher_metric::matcher_static::route_type;

        network_fixture<network_metric> network_fixture;

        network_metric network;
        auto network_matcher = prepare_network(network);
        auto matcher = create_matcher(network_matcher);

        const auto route_1 = route_type{network_fixture.create_line({0, 0, 1, 0})};
        const auto route_2 = route_type{network_fixture.create_line({1, 0, 1, 1})};
        const auto route_3 = route_type{network_fixture.create_line({1, 0, 2, 0})};
        const auto route_4 = route_type{network_fixture.create_line({1, 0, 1, -1})};
        const auto route_5 = route_type{network_fixture.create_line({1, 0, 0, 0})};
        const auto route_6 = route_type{network_fixture.create_line({0, 1, 1, 1})};

        const auto route_empty = route_type{false};
        const auto route_invalid = route_type{true};

        auto route_1_1 = route_type{route_1};
        auto route_1_2 = route_type{route_2};
        auto result_1 = route_type::merge({route_1_1, route_1_2});
        BOOST_CHECK_EQUAL(result_1.has_length, true);
        BOOST_CHECK_EQUAL(result_1.has_azimuth, true);
        BOOST_CHECK_EQUAL(result_1.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(result_1.length, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_1.azimuth, 45.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_1.directions, 90.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_1.absolute_directions, 90.0, 1e-6);

        auto route_2_1 = route_type{route_1};
        auto route_2_2 = route_type{route_3};
        auto result_2 = route_type::merge({route_2_1, route_2_2});
        BOOST_CHECK_EQUAL(result_2.has_length, true);
        BOOST_CHECK_EQUAL(result_2.has_azimuth, true);
        BOOST_CHECK_EQUAL(result_2.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(result_2.length, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_2.azimuth, 90.0, 1e-6);
        BOOST_CHECK_LE(std::fabs(result_2.directions), 1e-6);
        BOOST_CHECK_LE(result_2.absolute_directions, 1e-6);

        auto route_3_1 = route_type{route_1};
        auto route_3_2 = route_type{route_4};
        auto result_3 = route_type::merge({route_3_1, route_3_2});
        BOOST_CHECK_EQUAL(result_3.has_length, true);
        BOOST_CHECK_EQUAL(result_3.has_azimuth, true);
        BOOST_CHECK_EQUAL(result_3.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(result_3.length, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_3.azimuth, 135.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_3.directions, -90.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_3.absolute_directions, 90.0, 1e-6);

        auto route_4_1 = route_type{route_1};
        auto route_4_2 = route_type{route_5};
        auto result_4 = route_type::merge({route_4_1, route_4_2});
        BOOST_CHECK_EQUAL(result_4.has_length, true);
        BOOST_CHECK_EQUAL(result_4.has_azimuth, false);
        BOOST_CHECK_EQUAL(result_4.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(result_4.length, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(std::fabs(result_4.directions), 180.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_4.absolute_directions, 180.0, 1e-6);

        auto route_5_1 = route_type{route_1};
        auto route_5_2 = route_type{route_empty};
        auto result_5 = route_type::merge({route_5_1, route_5_2});
        BOOST_CHECK_EQUAL(result_5.has_length, true);
        BOOST_CHECK_EQUAL(result_5.has_azimuth, true);
        BOOST_CHECK_EQUAL(result_5.has_directions, false);
        BOOST_CHECK_CLOSE_FRACTION(result_5.length, route_1.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_5.azimuth, route_1.azimuth, 1e-6);

        auto route_6_1 = route_type{route_empty};
        auto route_6_2 = route_type{route_2};
        auto result_6 = route_type::merge({route_6_1, route_6_2});
        BOOST_CHECK_EQUAL(result_6.has_length, true);
        BOOST_CHECK_EQUAL(result_6.has_azimuth, true);
        BOOST_CHECK_EQUAL(result_6.has_directions, false);
        BOOST_CHECK_CLOSE_FRACTION(result_6.length, route_2.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_6.azimuth, route_2.azimuth, 1e-6);

        auto route_7_1 = route_type{route_3};
        auto route_7_2 = route_type{route_invalid};
        auto result_7 = route_type::merge({route_7_1, route_7_2});
        BOOST_CHECK_EQUAL(result_7.has_length, true);
        BOOST_CHECK_EQUAL(result_7.has_azimuth, true);
        BOOST_CHECK_EQUAL(result_7.has_directions, false);
        BOOST_CHECK_CLOSE_FRACTION(result_7.length, route_3.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_7.azimuth, route_3.azimuth, 1e-6);

        auto route_8_1 = route_type{route_invalid};
        auto route_8_2 = route_type{route_4};
        auto result_8 = route_type::merge({route_8_1, route_8_2});
        BOOST_CHECK_EQUAL(result_8.has_length, true);
        BOOST_CHECK_EQUAL(result_8.has_azimuth, true);
        BOOST_CHECK_EQUAL(result_8.has_directions, false);
        BOOST_CHECK_CLOSE_FRACTION(result_8.length, route_4.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_8.azimuth, route_4.azimuth, 1e-6);

        auto route_9_1 = route_type{route_1};
        auto route_9_2 = route_type{route_6};
        auto result_9 = route_type::merge({route_9_1, route_9_2});
        BOOST_CHECK_EQUAL(result_9.is_connected, false);

        auto route_10_1 = route_type{route_invalid};
        auto route_10_2 = route_type{route_invalid};
        auto result_10 = route_type::merge({route_10_1, route_10_2});
        BOOST_CHECK_EQUAL(result_10.is_invalid, true);

        auto route_11_1 = route_type{route_empty};
        auto route_11_2 = route_type{route_empty};
        auto result_11 = route_type::merge({route_11_1, route_11_2});
        BOOST_CHECK_EQUAL(result_11.is_invalid, false);
        BOOST_CHECK_EQUAL(result_11.has_length, false);
        BOOST_CHECK_EQUAL(result_11.has_azimuth, false);
        BOOST_CHECK_EQUAL(result_11.has_directions, false);
    }

BOOST_AUTO_TEST_SUITE_END()
