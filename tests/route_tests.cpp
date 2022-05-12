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

    BOOST_FIXTURE_TEST_CASE(route_return_test, matcher_fixture<matcher_metric>) {
        using line_type = typename matcher_metric::matcher_static::line_type;
        using rich_line_type = typename matcher_metric::matcher_static::rich_line_type;
        using route_type = typename matcher_metric::matcher_static::route_type;
        using search_type = std::array<std::int64_t, 8>;

        network_fixture<network_metric> network_fixture;

        const auto route_1 = route_type{
                network_fixture.create_line({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0})};
        const auto route_2 = route_type{network_fixture.create_line({3, 0, 2, 0, 1, 0, 0, 0})};
        const auto route_3 = route_type{network_fixture.create_line({9, 0, 8, 0, 7, 0})};

        auto search_1 = route_1.find_return_line(route_2);
        auto expectation_1 = search_type{-1, -1, -1, -1, -1, -1, -1, -1};
        BOOST_CHECK_EQUAL_COLLECTIONS(search_1.cbegin(), search_1.cend(), expectation_1.cbegin(), expectation_1.cend());

        auto search_2 = route_2.find_return_line(route_1);
        auto expectation_2 = search_type{0, 0, 0, 3, 0, 0, 0, 3};
        BOOST_CHECK_EQUAL_COLLECTIONS(search_2.cbegin(), search_2.cend(), expectation_2.cbegin(), expectation_2.cend());

        auto search_3 = route_1.find_return_line(route_3);
        auto expectation_3 = search_type{0, 7, 0, 9, 0, 0, 0, 2};
        BOOST_CHECK_EQUAL_COLLECTIONS(search_3.cbegin(), search_3.cend(), expectation_3.cbegin(), expectation_3.cend());

        auto search_4 = route_3.find_return_line(route_1);
        auto expectation_4 = search_type{-1, -1, -1, -1, -1, -1, -1, -1};
        BOOST_CHECK_EQUAL_COLLECTIONS(search_4.cbegin(), search_4.cend(), expectation_4.cbegin(), expectation_4.cend());

        auto return_1 = route_2.extract_return_line(route_1);
        BOOST_CHECK_EQUAL(return_1.has_length, true);
        BOOST_CHECK_EQUAL(return_1.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_1.length, route_2.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(return_1.azimuth, route_2.azimuth, 1e-6);

        auto return_2 = route_2.extract_return_line(route_1, false);
        BOOST_CHECK_EQUAL(return_2.has_length, true);
        BOOST_CHECK_EQUAL(return_2.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_2.length, route_2.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(return_2.azimuth, route_1.azimuth, 1e-6);

        auto return_3 = route_1.extract_return_line(route_3);
        BOOST_CHECK_EQUAL(return_3.has_length, true);
        BOOST_CHECK_EQUAL(return_3.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_3.length, route_3.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(return_3.azimuth, route_1.azimuth, 1e-6);

        auto return_4 = route_1.extract_return_line(route_3, false);
        BOOST_CHECK_EQUAL(return_4.has_length, true);
        BOOST_CHECK_EQUAL(return_4.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_4.length, route_3.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(return_4.azimuth, route_3.azimuth, 1e-6);

        auto trim_1 = route_2.trim_merge(route_1);
        BOOST_CHECK_EQUAL(trim_1.has_length, true);
        BOOST_CHECK_EQUAL(trim_1.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_1.length, route_1.length - route_2.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_1.azimuth, route_1.azimuth, 1e-6);

        auto trim_2 = route_1.trim_merge(route_3);
        BOOST_CHECK_EQUAL(trim_2.has_length, true);
        BOOST_CHECK_EQUAL(trim_2.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_2.length, route_1.length - route_3.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_2.azimuth, route_1.azimuth, 1e-6);

        auto trim_3 = route_2.trim_merge(route_3);
        BOOST_CHECK_EQUAL(trim_3.is_invalid, true);

        const auto route_4 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({0, 0, 1, 0}),
                network_fixture.create_line({1, 0, 2, 0}),
                network_fixture.create_line({2, 0, 3, 0}),
                network_fixture.create_line({3, 0, 4, 0}),
                network_fixture.create_line({4, 0, 5, 0}),
                network_fixture.create_line({5, 0, 6, 0}),
                network_fixture.create_line({6, 0, 7, 0}),
                network_fixture.create_line({7, 0, 8, 0}),
                network_fixture.create_line({8, 0, 9, 0})}};
        const auto route_5 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({4, 0, 3, 0}),
                network_fixture.create_line({3, 0, 2, 0}),
                network_fixture.create_line({2, 0, 1, 0}),
                network_fixture.create_line({1, 0, 0, 0})}};
        const auto route_6 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({9, 0, 8, 0}),
                network_fixture.create_line({8, 0, 7, 0}),
                network_fixture.create_line({7, 0, 6, 0})}};

        auto search_5 = route_4.find_return_line(route_5);
        auto expectation_5 = search_type{-1, -1, -1, -1, -1, -1, -1, -1};
        BOOST_CHECK_EQUAL_COLLECTIONS(search_5.cbegin(), search_5.cend(), expectation_5.cbegin(), expectation_5.cend());

        auto search_6 = route_5.find_return_line(route_4);
        auto expectation_6 = search_type{0, 0, 3, 1, 0, 0, 3, 1};
        BOOST_CHECK_EQUAL_COLLECTIONS(search_6.cbegin(), search_6.cend(), expectation_6.cbegin(), expectation_6.cend());

        auto search_7 = route_4.find_return_line(route_6);
        auto expectation_7 = search_type{5, 1, 8, 1, 0, 0, 2, 1};
        BOOST_CHECK_EQUAL_COLLECTIONS(search_7.cbegin(), search_7.cend(), expectation_7.cbegin(), expectation_7.cend());

        auto search_8 = route_6.find_return_line(route_4);
        auto expectation_8 = search_type{-1, -1, -1, -1, -1, -1, -1, -1};
        BOOST_CHECK_EQUAL_COLLECTIONS(search_8.cbegin(), search_8.cend(), expectation_8.cbegin(), expectation_8.cend());

        auto return_5 = route_5.extract_return_line(route_4);
        BOOST_CHECK_EQUAL(return_5.has_length, true);
        BOOST_CHECK_EQUAL(return_5.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_5.length, route_5.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(return_5.azimuth, route_5.azimuth, 1e-6);

        auto return_6 = route_5.extract_return_line(route_4, false);
        BOOST_CHECK_EQUAL(return_6.has_length, true);
        BOOST_CHECK_EQUAL(return_6.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_6.length, route_5.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(return_6.azimuth, route_4.azimuth, 1e-6);

        auto return_7 = route_4.extract_return_line(route_6);
        BOOST_CHECK_EQUAL(return_7.has_length, true);
        BOOST_CHECK_EQUAL(return_7.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_7.length, route_6.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(return_7.azimuth, route_4.azimuth, 1e-6);

        auto return_8 = route_4.extract_return_line(route_6, false);
        BOOST_CHECK_EQUAL(return_8.has_length, true);
        BOOST_CHECK_EQUAL(return_8.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_8.length, route_6.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(return_8.azimuth, route_6.azimuth, 1e-6);

        auto trim_4 = route_5.trim_merge(route_4);
        BOOST_CHECK_EQUAL(trim_4.has_length, true);
        BOOST_CHECK_EQUAL(trim_4.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_4.length, route_4.length - route_5.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_4.azimuth, route_4.azimuth, 1e-6);

        auto trim_5 = route_4.trim_merge(route_6);
        BOOST_CHECK_EQUAL(trim_5.has_length, true);
        BOOST_CHECK_EQUAL(trim_5.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_5.length, route_4.length - route_6.length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_5.azimuth, route_4.azimuth, 1e-6);

        auto trim_6 = route_5.trim_merge(route_6);
        BOOST_CHECK_EQUAL(trim_6.is_invalid, true);

        const auto route_7 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({0, 0, 2, 0}),
                network_fixture.create_line({2, 0, 4, 0, 6, 0}),
                network_fixture.create_line({6, 0, 9, 0})}};
        const auto route_8 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({4, 3, 4, 1, 4, 0}),
                network_fixture.create_line({4, 0, 2, 0, 0, 0})}};

        auto trim_7 = route_8.trim_merge(route_7);
        BOOST_CHECK_EQUAL(trim_7.has_length, true);
        BOOST_CHECK_EQUAL(trim_7.has_azimuth, true);
        BOOST_CHECK_EQUAL(trim_7.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_7.length, 8.0, 1e-6);
        BOOST_CHECK(trim_7.azimuth > 90 and trim_7.azimuth < 180);
        BOOST_CHECK_CLOSE_FRACTION(trim_7.directions, 90.0, 1e-6);

        const auto route_9 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({5, 3, 5, 1, 5, 0}),
                network_fixture.create_line({5, 0, 4, 0}),
                network_fixture.create_line({4, 0, 2, 0, 0, 0})}};

        auto trim_8 = route_9.trim_merge(route_7);
        BOOST_CHECK_EQUAL(trim_8.has_length, true);
        BOOST_CHECK_EQUAL(trim_8.has_azimuth, true);
        BOOST_CHECK_EQUAL(trim_8.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_8.length, 7.0, 1e-6);
        BOOST_CHECK(trim_8.azimuth > 90 and trim_8.azimuth < 180);
        BOOST_CHECK_CLOSE_FRACTION(trim_8.directions, 90.0, 1e-6);

        const auto route_10 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({5, 4, 5, 3}),
                network_fixture.create_line({5, 3, 5, 1, 5, 0, 4, 0}),
                network_fixture.create_line({4, 0, 2, 0, 0, 0})}};

        auto trim_9 = route_10.trim_merge(route_7);
        BOOST_CHECK_EQUAL(trim_9.has_length, true);
        BOOST_CHECK_EQUAL(trim_9.has_azimuth, true);
        BOOST_CHECK_EQUAL(trim_9.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_9.length, 8.0, 1e-6);
        BOOST_CHECK(trim_9.azimuth > 90 and trim_9.azimuth < 180);
        BOOST_CHECK_CLOSE_FRACTION(trim_9.directions, 90.0, 1e-6);

        auto return_9 = route_10.extract_return_line(route_7);
        BOOST_CHECK_EQUAL(return_9.has_length, true);
        BOOST_CHECK_EQUAL(return_9.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(return_9.length, 4.0, 1e-6);

        const auto route_11 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({4, 4, 4, 3}),
                network_fixture.create_line({4, 3, 4, 1}),
                network_fixture.create_line({4, 0, 2, 0, 0, 0})}};

        auto trim_10 = route_11.trim_merge(route_7);
        BOOST_CHECK_EQUAL(trim_10.is_connected, false);
        BOOST_CHECK_EQUAL(trim_10.has_length, true);
        BOOST_CHECK_EQUAL(trim_10.has_azimuth, true);
        BOOST_CHECK_EQUAL(trim_10.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_10.length, 8.0, 1e-6);
        BOOST_CHECK(trim_10.azimuth > 90 and trim_10.azimuth < 180);
        BOOST_CHECK_LE(trim_10.directions, 1e-6);

        const auto route_12 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({5, 0, 4, 0, 2, 0, 0, 0})}};

        auto trim_11 = route_12.trim_merge(route_7);
        BOOST_CHECK_EQUAL(trim_11.has_length, true);
        BOOST_CHECK_EQUAL(trim_11.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_11.length, 4.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_11.azimuth, route_7.azimuth, 1e-6);

        auto trim_12 = route_5.trim_merge(route_4);
        trim_12 = trim_12.trim_merge(route_6);
        BOOST_CHECK_EQUAL(trim_12.has_length, true);
        BOOST_CHECK_EQUAL(trim_12.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_12.length, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_12.azimuth, route_4.azimuth, 1e-6);

        const auto route_13 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({0, 3, 0, 1, 0, 0})}};

        auto trim_13 = route_13.trim_merge(route_7);
        BOOST_CHECK_EQUAL(trim_13.has_length, true);
        BOOST_CHECK_EQUAL(trim_13.has_azimuth, true);
        BOOST_CHECK_EQUAL(trim_13.has_directions, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_13.length, 12.0, 1e-6);
        BOOST_CHECK(trim_13.azimuth > 90 and trim_13.azimuth < 180);
        BOOST_CHECK_CLOSE_FRACTION(trim_13.directions, 90.0, 1e-6);

        auto return_10 = route_13.extract_return_line(route_7);
        BOOST_CHECK_EQUAL(return_10.has_length, false);

        const auto route_14 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({1, 2, 2, 2}),
                network_fixture.create_line({2, 2, 3, 2})}};

        const auto route_15 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({2, 2, 1, 2})}};

        const auto route_16 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({3, 2, 2, 2})}};

        auto trim_14 = route_15.trim_merge(route_14);
        BOOST_CHECK_EQUAL(trim_14.has_length, true);
        BOOST_CHECK_EQUAL(trim_14.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_14.length, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_14.azimuth, route_14.azimuth, 1e-6);

        auto trim_15 = route_14.trim_merge(route_16);
        BOOST_CHECK_EQUAL(trim_15.has_length, true);
        BOOST_CHECK_EQUAL(trim_15.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_15.length, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_15.azimuth, route_14.azimuth, 1e-6);

        auto trim_16 = trim_14.trim_merge(route_16);
        BOOST_CHECK_EQUAL(trim_16.is_invalid, false);
        BOOST_CHECK_EQUAL(trim_16.has_length, false);
        BOOST_CHECK_EQUAL(trim_16.has_azimuth, false);

        const auto route_17 = route_type{std::deque<rich_line_type>{
                network_fixture.create_line({2, 2, 2.2, 2})}};

        auto trim_17 = route_16.trim_merge(route_17);
        BOOST_CHECK_EQUAL(trim_17.has_length, true);
        BOOST_CHECK_EQUAL(trim_17.has_azimuth, true);
        BOOST_CHECK_CLOSE_FRACTION(trim_17.length, 0.8, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(trim_17.azimuth, route_16.azimuth, 1e-6);
    }

BOOST_AUTO_TEST_SUITE_END()
