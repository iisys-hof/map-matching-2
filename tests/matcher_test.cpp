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

#include <geometry/compare.hpp>

#include <environment/environments/single.hpp>
#include <environment/environments/hmm.hpp>
#include <learning/algorithms/value_iteration.hpp>
#include <learning/algorithms/viterbi.hpp>

#include "helper/matcher_fixture.hpp"

BOOST_AUTO_TEST_SUITE(matcher_tests)

    BOOST_FIXTURE_TEST_CASE(candidate_search_grid_test, matcher_fixture<matcher_metric>) {
        network_fixture<network_metric> network_fixture;
        auto network = network_fixture.generate_grid_network();

        auto network_matcher = prepare_network(network);
        auto matcher = create_matcher(network_matcher);

        auto track = create_track("one", {{0, {2.2, 2.2}},
                                          {1, {4.5, 2.5}},
                                          {2, {4.9, 5.8}}});

        std::cout << track.str() << std::endl;

        auto candidates_1_1 = matcher.candidate_search_buffer(track, 32, 0.75, 10.0, 2.0, false, false);

        BOOST_CHECK_EQUAL(candidates_1_1.size(), 3);
        BOOST_CHECK_EQUAL(candidates_1_1.at(0).measurement->timestamp, track.measurements.at(0).timestamp);
        BOOST_CHECK_EQUAL(candidates_1_1.at(0).nodes.size(), 5);
        BOOST_CHECK_EQUAL(candidates_1_1.at(0).edges.size(), 8);
        BOOST_CHECK_EQUAL(candidates_1_1.at(1).measurement->timestamp, track.measurements.at(1).timestamp);
        BOOST_CHECK_EQUAL(candidates_1_1.at(1).nodes.size(), 12);
        BOOST_CHECK_EQUAL(candidates_1_1.at(1).edges.size(), 24);
        BOOST_CHECK_EQUAL(candidates_1_1.at(2).measurement->timestamp, track.measurements.at(2).timestamp);
        BOOST_CHECK_EQUAL(candidates_1_1.at(2).nodes.size(), 5);
        BOOST_CHECK_EQUAL(candidates_1_1.at(2).edges.size(), 8);

        auto candidates_1_2 = matcher.candidate_search_buffer(track, 32, 0.75, 10.0, 2.0, false, true);

        BOOST_CHECK_EQUAL(candidates_1_2.size(), 3);
        BOOST_CHECK_EQUAL(candidates_1_2.at(0).measurement->timestamp, track.measurements.at(0).timestamp);
        BOOST_CHECK_EQUAL(candidates_1_2.at(0).nodes.size(), 16);
        BOOST_CHECK_EQUAL(candidates_1_2.at(0).edges.size(), 32);
        BOOST_CHECK_EQUAL(candidates_1_2.at(1).measurement->timestamp, track.measurements.at(1).timestamp);
        BOOST_CHECK_EQUAL(candidates_1_2.at(1).nodes.size(), 21);
        BOOST_CHECK_EQUAL(candidates_1_2.at(1).edges.size(), 40);
        BOOST_CHECK_EQUAL(candidates_1_2.at(2).measurement->timestamp, track.measurements.at(2).timestamp);
        BOOST_CHECK_EQUAL(candidates_1_2.at(2).nodes.size(), 17);
        BOOST_CHECK_EQUAL(candidates_1_2.at(2).edges.size(), 32);

        auto route_1_1 = matcher.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}),
                2, get_candidate_position(network_matcher, candidates_1_1, 2, {55, 56}));
        BOOST_CHECK_EQUAL(route_1_1.get_line().size(), 8);
        BOOST_CHECK_CLOSE_FRACTION(route_1_1.length, 6.6, 1e-3);
        BOOST_CHECK_CLOSE_FRACTION(route_1_1.directions, 90.0, 1e-2);

        auto route_1_2 = matcher.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 12}),
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {32, 22}));
        BOOST_CHECK_EQUAL(route_1_2.get_line().size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(route_1_2.length, 3.8, 1e-3);

        auto route_1_3 = matcher.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}),
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {12, 22}));
        BOOST_CHECK_EQUAL(route_1_3.get_line().size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(route_1_3.length, 3.8, 1e-3);

        auto route_1_4 = matcher.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}),
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}));
        BOOST_CHECK_EQUAL(route_1_4.get_line().size(), 0);
        BOOST_CHECK(not route_1_4.has_length);

        auto candidates_2 = matcher.candidate_search_buffer(track, 32, 3.5, 10.0, 2.0, false, false);

        auto route_2_1 = matcher.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {22, 32}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {22, 32}));
        BOOST_CHECK_EQUAL(route_2_1.get_line().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_2_1.length, 0.8, 1e-3);
        BOOST_CHECK(not route_2_1.has_directions);

        auto route_2_2 = matcher.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {22, 32}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {32, 42}));
        BOOST_CHECK_EQUAL(route_2_2.get_line().size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(route_2_2.length, 1.8, 1e-3);
        BOOST_CHECK_LE(std::fabs(route_2_2.directions), 1e-2);

        auto route_2_3 = matcher.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {22, 23}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {22, 23}));
        BOOST_CHECK_EQUAL(route_2_3.get_line().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_2_3.length, 0.3, 1e-3);

        auto route_2_4 = matcher.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {23, 22}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {23, 22}));
        BOOST_CHECK_EQUAL(route_2_4.get_line().size(), 4);
        BOOST_CHECK_CLOSE_FRACTION(route_2_4.length, 1.7, 1e-3);

        auto route_2_5 = matcher.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {22, 23}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {23, 22}));
        BOOST_CHECK_EQUAL(route_2_5.get_line().size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(route_2_5.length, 1.3, 1e-3);

        auto route_2_6 = matcher.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {23, 22}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {22, 23}));
        BOOST_CHECK_EQUAL(route_2_6.get_line().size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(route_2_6.length, 0.7, 1e-3);

        auto track_2 = create_track("two", {{0, {2.2, 2.2}},
                                            {1, {4.2, 2.1}},
                                            {2, {2.8, 1.9}},
                                            {4, {5.3, 1.8}}});

        std::cout << track_2.str() << std::endl;

        auto candidates_3 = matcher.candidate_search_buffer(track_2, 32, 0.75);

        auto route_3_1 = matcher.candidate_route(
                candidates_3,
                1, get_candidate_position(network_matcher, candidates_3, 1, {32, 42}),
                2, get_candidate_position(network_matcher, candidates_3, 2, {32, 42}));
        BOOST_CHECK_EQUAL(route_3_1.get_line().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_3_1.length, 1.0, 1e-3);
    }

    BOOST_FIXTURE_TEST_CASE(candidate_search_test, matcher_fixture<matcher_metric>) {
        network_fixture<network_metric> network_fixture;

        network_metric network;
        network_fixture.add_vertex(network, 1, {0, 0});
        network_fixture.add_vertex(network, 2, {10, 0});
        network_fixture.add_edge(network, 1, {1, 2}, {1, 0, 1, 1, 4, 1, 4, -1, 8, -1, 8, 0});

        auto network_matcher = prepare_network(network);
        auto matcher = create_matcher(network_matcher);

        network.save("candidate_search_test_nodes.csv",
                     "candidate_search_test_edges.csv");

        auto track = create_track("two", {{0, {0.1, 0.1}},
                                          {1, {4.2, -0.2}},
                                          {2, {9.8, 0.3}}});

        std::cout << track.str() << std::endl;

        auto candidates = matcher.candidate_search_buffer(track, 32, 2.0);

        BOOST_CHECK_EQUAL(candidates.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(0).edges.at(0).distance, 0.1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<0>(candidates.at(0).edges.at(0).projection_point), 0.1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<1>(candidates.at(0).edges.at(0).projection_point), 0.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<0>(candidates.at(0).edges.at(0).from.line.at(0)), 0.0, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<1>(candidates.at(0).edges.at(0).from.line.at(0)), 0.0, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<0>(candidates.at(0).edges.at(0).from.line.at(1)), 0.1, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<1>(candidates.at(0).edges.at(0).from.line.at(1)), 0.0, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(0).edges.at(0).from.length, 0.1, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(0).edges.at(0).to.length, 13.9, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(1).edges.at(0).from.length, 6.2, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(1).edges.at(0).to.length, 7.8, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(2).edges.at(0).from.length, 13.8, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(2).edges.at(0).to.length, 0.2, 1e-4);
        BOOST_CHECK_EQUAL(candidates.at(0).edges.at(0).from.line.size(), 2);
        BOOST_CHECK_EQUAL(candidates.at(0).edges.at(0).to.line.size(), 8);
        BOOST_CHECK_EQUAL(candidates.at(1).edges.at(0).from.line.size(), 5);
        BOOST_CHECK_EQUAL(candidates.at(1).edges.at(0).to.line.size(), 5);
        BOOST_CHECK_EQUAL(candidates.at(2).edges.at(0).from.line.size(), 8);
        BOOST_CHECK_EQUAL(candidates.at(2).edges.at(0).to.line.size(), 2);

        auto route_1 = matcher.candidate_route(candidates, 0, 0, 1, 0);
        BOOST_CHECK_EQUAL(route_1.get_line().size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(route_1.length, 6.1, 1e-4);

        auto route_2 = matcher.candidate_route(candidates, 1, 0, 2, 0);
        BOOST_CHECK_EQUAL(route_2.get_line().size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(route_2.length, 7.6, 1e-4);
    }

    BOOST_FIXTURE_TEST_CASE(matching_value_iteration_test, matcher_fixture<matcher_metric>) {
        network_fixture<network_metric> network_fixture;
        auto network = network_fixture.generate_grid_network();

        auto network_matcher = prepare_network(network);
        auto matcher = create_matcher(network_matcher);

        map_matching_2::learning::settings learn_settings;
        map_matching_2::matching::settings match_settings;
        match_settings.buffer_radius = 1.5;
        match_settings.simplify = false;
        match_settings.median_merge = false;

        using environment = map_matching_2::environment::single<typename matcher_metric::matcher_static>;
        using learn_function = map_matching_2::learning::value_iteration<environment>;

        auto track = create_track("one", {{0, {2.3, 2.2}},
                                          {1, {5.1, 2.3}},
                                          {2, {4.9, 5.8}}});

        auto candidates = matcher.candidate_search_buffer(track, match_settings.buffer_points,
                                                          match_settings.buffer_radius);
        matcher.save_candidates("matching_value_iteration_test_candidates.csv", candidates);

        typename matcher_metric::matcher_static::line_type ground_truth_line;
        boost::geometry::read_wkt("LINESTRING(2.3 2,3 2,4 2,5 2,5 3,5 4,5 5,5 5.8)", ground_truth_line);
        const typename matcher_metric::matcher_static::route_type ground_truth{ground_truth_line};

        matcher.template match<learn_function>(
                track, learn_settings, match_settings,
                [&](const auto &environment, const auto &learner, const auto &policy,
                    const auto &track, const auto &prepared, const auto &route, const auto duration) {
                    std::cout << "Track: " << track.str() << std::endl;
                    std::cout << "Prepared: " << prepared.str() << std::endl;
                    std::cout << "Route: " << route.str() << std::endl;
                    std::cout << "Ground truth: " << ground_truth.str() << std::endl;

                    map_matching_2::geometry::line_comparator<typename network_metric::route_type::rich_line_type> line_comparator;
                    const auto comparison = line_comparator.compare(
                            route.get_rich_line(), ground_truth.get_rich_line());

                    BOOST_CHECK_EQUAL(comparison.error_fraction, 0.0L);
                    BOOST_CHECK_EQUAL(comparison.error_added, 0.0L);
                    BOOST_CHECK_EQUAL(comparison.error_missed, 0.0L);
                });
    }

    BOOST_FIXTURE_TEST_CASE(matching_viterbi_hmm_test, matcher_fixture<matcher_metric>) {
        network_fixture<network_metric> network_fixture;
        auto network = network_fixture.generate_grid_network();

        auto network_matcher = prepare_network(network);
        auto matcher = create_matcher(network_matcher);

        map_matching_2::learning::settings learn_settings;
        map_matching_2::matching::settings match_settings;
        match_settings.buffer_radius = 1.5;
        match_settings.simplify = false;
        match_settings.median_merge = false;

        using environment = map_matching_2::environment::hmm<typename matcher_metric::matcher_static>;
        using learn_function = map_matching_2::learning::viterbi<environment>;

        auto track = create_track("one", {{0, {2.3, 2.2}},
                                          {1, {5.1, 2.3}},
                                          {2, {4.9, 5.8}}});

        auto candidates = matcher.candidate_search_buffer(track, match_settings.buffer_points,
                                                          match_settings.buffer_radius);
        matcher.save_candidates("matching_viterbi_hmm_test_candidates.csv", candidates);

        typename matcher_metric::matcher_static::line_type ground_truth_line;
        boost::geometry::read_wkt("LINESTRING(2.3 2,3 2,4 2,5 2,5 3,5 4,5 5,5 5.8)", ground_truth_line);
        const typename matcher_metric::matcher_static::route_type ground_truth{ground_truth_line};

        matcher.template match<learn_function>(
                track, learn_settings, match_settings,
                [&](const auto &environment, const auto &learner, const auto &policy,
                    const auto &track, const auto &prepared, const auto &route, const auto duration) {
                    std::cout << "Track: " << track.str() << std::endl;
                    std::cout << "Prepared: " << prepared.str() << std::endl;
                    std::cout << "Route: " << route.str() << std::endl;
                    std::cout << "Ground truth: " << ground_truth.str() << std::endl;

                    map_matching_2::geometry::line_comparator<typename network_metric::route_type::rich_line_type> line_comparator;
                    const auto comparison = line_comparator.compare(
                            route.get_rich_line(), ground_truth.get_rich_line());

                    BOOST_CHECK_EQUAL(comparison.error_fraction, 0.0L);
                    BOOST_CHECK_EQUAL(comparison.error_added, 0.0L);
                    BOOST_CHECK_EQUAL(comparison.error_missed, 0.0L);
                });
    }

BOOST_AUTO_TEST_SUITE_END()
