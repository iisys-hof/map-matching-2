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

#include "types/matching/matcher/matcher_forwarder.hpp"
#include "types/io/track/csv_track_importer.hpp"

#include "compare/comparator/comparator_forwarder_matches.hpp"
#include "compare/comparator/comparator_forwarder_compares.hpp"

#include "geometry/algorithm/compare.hpp"

#include "matching/traits/network.hpp"

#include "util/time_helper.hpp"

#include "helper/matcher_fixture.hpp"

BOOST_AUTO_TEST_SUITE(matcher_tests)

    BOOST_FIXTURE_TEST_CASE(candidate_search_test, matcher_fixture<matcher_metric>) {
        network_fixture<network_import_metric> network_fixture;

        network_import_metric network;
        network_fixture.add_vertex(network, 1, {0, 0});
        network_fixture.add_vertex(network, 2, {10, 0});
        network_fixture.add_vertex(network, 3, {1, 0});
        network_fixture.add_vertex(network, 4, {1, 1});
        network_fixture.add_vertex(network, 5, {4, 1});
        network_fixture.add_vertex(network, 6, {4, -1});
        network_fixture.add_vertex(network, 7, {8, -1});
        network_fixture.add_vertex(network, 8, {8, 0});
        network_fixture.add_edge(network, 1, {1, 3});
        network_fixture.add_edge(network, 2, {3, 4});
        network_fixture.add_edge(network, 3, {4, 5});
        network_fixture.add_edge(network, 4, {5, 6});
        network_fixture.add_edge(network, 5, {6, 7});
        network_fixture.add_edge(network, 6, {7, 8});
        network_fixture.add_edge(network, 7, {8, 2});

        network_metric network_matcher = prepare_network(network);

        network_matcher.save_csv("candidate_search_test_nodes.csv",
                "candidate_search_test_edges.csv");

        auto track = create_track("two", {
                {0.1, 0.1, 0},
                {4.2, -0.2, 1},
                {9.8, 0.3, 2}
        });

        std::cout << track.str() << std::endl;

        map_matching_2::util::time_helper _time_helper;
        map_matching_2::matching::algorithms _algorithms{network_matcher, _time_helper};

        map_matching_2::matching::settings match_settings{};
        match_settings.candidate_search = map_matching_2::matching::settings::CANDIDATE_SEARCH::CIRCLE;
        match_settings.radius = 2.0;

        auto candidates = _algorithms.searcher.candidate_search(track, match_settings);

        BOOST_CHECK_EQUAL(candidates.size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(0).edges.at(0).distance, 0.1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<0>(candidates.at(0).edges.at(0).projection_point), 0.1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<1>(candidates.at(0).edges.at(0).projection_point), 0.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<0>(candidates.at(0).edges.at(0).from->at(0)), 0.0, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<1>(candidates.at(0).edges.at(0).from->at(0)), 0.0, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<0>(candidates.at(0).edges.at(0).from->at(1)), 0.1, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(boost::geometry::get<1>(candidates.at(0).edges.at(0).from->at(1)), 0.0, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(0).edges.at(0).from->length(), 0.1, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(0).edges.at(0).to->length(), 13.9, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(1).edges.at(0).from->length(), 6.2, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(1).edges.at(0).to->length(), 7.8, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(2).edges.at(0).from->length(), 13.8, 1e-4);
        BOOST_CHECK_CLOSE_FRACTION(candidates.at(2).edges.at(0).to->length(), 0.2, 1e-4);
        BOOST_CHECK_EQUAL(candidates.at(0).edges.at(0).from->size(), 2);
        BOOST_CHECK_EQUAL(candidates.at(0).edges.at(0).to->size(), 8);
        BOOST_CHECK_EQUAL(candidates.at(1).edges.at(0).from->size(), 5);
        BOOST_CHECK_EQUAL(candidates.at(1).edges.at(0).to->size(), 5);
        BOOST_CHECK_EQUAL(candidates.at(2).edges.at(0).from->size(), 8);
        BOOST_CHECK_EQUAL(candidates.at(2).edges.at(0).to->size(), 2);

        using rich_line_type = decltype(_algorithms.router)::lazy_rich_line_type;

        auto route_1 = _algorithms.router.candidate_route(candidates, 0, 0, 1, 0);
        BOOST_CHECK_EQUAL(route_1.get_rich_line<rich_line_type>().size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(route_1.length(), 6.1, 1e-4);

        auto route_2 = _algorithms.router.candidate_route(candidates, 1, 0, 2, 0);
        BOOST_CHECK_EQUAL(route_2.get_rich_line<rich_line_type>().size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(route_2.length(), 7.6, 1e-4);
    }

    BOOST_FIXTURE_TEST_CASE(candidate_search_grid_test, matcher_fixture<matcher_metric>) {
        network_fixture<network_import_metric> network_fixture;
        network_import_metric network = network_fixture.generate_grid_network();

        network_metric network_matcher = prepare_network(network);

        network_matcher.save_csv("candidate_search_grid_test_nodes.csv",
                "candidate_search_grid_test_edges.csv");

        auto track = create_track("one", {
                {2.2, 2.2, 0},
                {4.5, 2.5, 1},
                {4.9, 5.8, 2}
        });

        std::cout << track.str() << std::endl;

        map_matching_2::util::time_helper _time_helper;
        map_matching_2::matching::algorithms _algorithms{network_matcher, _time_helper};

        map_matching_2::matching::settings match_settings{};
        match_settings.candidate_search = map_matching_2::matching::settings::CANDIDATE_SEARCH::CIRCLE;
        match_settings.radius = 0.75;
        match_settings.radius_upper_limit = 10.0;
        match_settings.radius_lower_limit = 2.0;
        match_settings.adaptive_radius = false;
        match_settings.candidate_adoption_siblings = false;
        match_settings.candidate_adoption_nearby = false;

        auto candidates_1_1 = _algorithms.searcher.candidate_search(track, match_settings);

        BOOST_CHECK_EQUAL(candidates_1_1.size(), 3);
        BOOST_CHECK_EQUAL(candidates_1_1.at(0).nodes.size(), 5);
        BOOST_CHECK_EQUAL(candidates_1_1.at(0).edges.size(), 8);
        BOOST_CHECK_EQUAL(candidates_1_1.at(1).nodes.size(), 12);
        BOOST_CHECK_EQUAL(candidates_1_1.at(1).edges.size(), 24);
        BOOST_CHECK_EQUAL(candidates_1_1.at(2).nodes.size(), 5);
        BOOST_CHECK_EQUAL(candidates_1_1.at(2).edges.size(), 8);

        match_settings.candidate_adoption_siblings = true;
        auto candidates_1_2 = _algorithms.searcher.candidate_search(track, match_settings);

        BOOST_CHECK_EQUAL(candidates_1_2.size(), 3);
        BOOST_CHECK_EQUAL(candidates_1_2.at(0).nodes.size(), 16);
        BOOST_CHECK_EQUAL(candidates_1_2.at(0).edges.size(), 32);
        BOOST_CHECK_EQUAL(candidates_1_2.at(1).nodes.size(), 21);
        BOOST_CHECK_EQUAL(candidates_1_2.at(1).edges.size(), 40);
        BOOST_CHECK_EQUAL(candidates_1_2.at(2).nodes.size(), 17);
        BOOST_CHECK_EQUAL(candidates_1_2.at(2).edges.size(), 32);

        using rich_line_type = decltype(_algorithms.router)::lazy_rich_line_type;

        auto route_1_1 = _algorithms.router.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}),
                2, get_candidate_position(network_matcher, candidates_1_1, 2, {55, 56}));
        BOOST_CHECK_EQUAL(route_1_1.get_rich_line<rich_line_type>().size(), 8);
        BOOST_CHECK_CLOSE_FRACTION(route_1_1.length(), 6.6, 1e-3);
        BOOST_CHECK_CLOSE_FRACTION(route_1_1.directions(), 90.0, 1e-2);

        auto route_1_2_1 = _algorithms.router.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 12}),
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {32, 22}),
                true);
        BOOST_CHECK_EQUAL(route_1_2_1.get_rich_line<rich_line_type>().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_1_2_1.length(), 0.2, 1e-3);

        auto route_1_2_2 = _algorithms.router.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 12}),
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {32, 22}));
        BOOST_CHECK_EQUAL(route_1_2_2.get_rich_line<rich_line_type>().size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(route_1_2_2.length(), 3.8, 1e-3);

        auto route_1_3_1 = _algorithms.router.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}),
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {12, 22}),
                true);
        BOOST_CHECK_EQUAL(route_1_3_1.get_rich_line<rich_line_type>().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_1_3_1.length(), 0.2, 1e-3);

        auto route_1_3_2 = _algorithms.router.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}),
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {12, 22}));
        BOOST_CHECK_EQUAL(route_1_3_2.get_rich_line<rich_line_type>().size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(route_1_3_2.length(), 3.8, 1e-3);

        auto route_1_4 = _algorithms.router.candidate_route(
                candidates_1_1,
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}),
                0, get_candidate_position(network_matcher, candidates_1_1, 0, {22, 32}));
        BOOST_CHECK_EQUAL(route_1_4.get_rich_line<rich_line_type>().size(), 0);
        BOOST_CHECK(not route_1_4.has_length());

        match_settings.radius = 3.5;
        match_settings.candidate_adoption_siblings = false;
        auto candidates_2 = _algorithms.searcher.candidate_search(track, match_settings);

        auto route_2_1 = _algorithms.router.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {22, 32}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {22, 32}));
        BOOST_CHECK_EQUAL(route_2_1.get_rich_line<rich_line_type>().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_2_1.length(), 0.8, 1e-3);
        BOOST_CHECK(not route_2_1.has_directions());

        auto route_2_2 = _algorithms.router.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {22, 32}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {32, 42}));
        BOOST_CHECK_EQUAL(route_2_2.get_rich_line<rich_line_type>().size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(route_2_2.length(), 1.8, 1e-3);
        BOOST_CHECK_LE(std::fabs(route_2_2.directions()), 1e-2);

        auto route_2_3 = _algorithms.router.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {22, 23}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {22, 23}));
        BOOST_CHECK_EQUAL(route_2_3.get_rich_line<rich_line_type>().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_2_3.length(), 0.3, 1e-3);

        auto route_2_4_1 = _algorithms.router.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {23, 22}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {23, 22}),
                true);
        BOOST_CHECK_EQUAL(route_2_4_1.get_rich_line<rich_line_type>().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_2_4_1.length(), 0.3, 1e-3);

        auto route_2_4_2 = _algorithms.router.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {23, 22}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {23, 22}));
        BOOST_CHECK_EQUAL(route_2_4_2.get_rich_line<rich_line_type>().size(), 4);
        BOOST_CHECK_CLOSE_FRACTION(route_2_4_2.length(), 1.7, 1e-3);

        auto route_2_5 = _algorithms.router.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {22, 23}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {23, 22}));
        BOOST_CHECK_EQUAL(route_2_5.get_rich_line<rich_line_type>().size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(route_2_5.length(), 1.3, 1e-3);

        auto route_2_6 = _algorithms.router.candidate_route(
                candidates_2,
                0, get_candidate_position(network_matcher, candidates_2, 0, {23, 22}),
                1, get_candidate_position(network_matcher, candidates_2, 1, {22, 23}));
        BOOST_CHECK_EQUAL(route_2_6.get_rich_line<rich_line_type>().size(), 3);
        BOOST_CHECK_CLOSE_FRACTION(route_2_6.length(), 0.7, 1e-3);

        auto track_2 = create_track("two", {
                {2.2, 2.2, 0},
                {4.2, 2.1, 1},
                {2.8, 1.9, 2},
                {5.3, 1.8, 3}
        });

        std::cout << track_2.str() << std::endl;

        match_settings.radius = 0.75;
        match_settings.radius_upper_limit = 10.000;
        match_settings.radius_lower_limit = 200.0;
        match_settings.adaptive_radius = true;
        match_settings.candidate_adoption_siblings = true;
        auto candidates_3 = _algorithms.searcher.candidate_search(track_2, match_settings);

        auto route_3_1 = _algorithms.router.candidate_route(
                candidates_3,
                1, get_candidate_position(network_matcher, candidates_3, 1, {32, 42}),
                2, get_candidate_position(network_matcher, candidates_3, 2, {32, 42}));
        BOOST_CHECK_EQUAL(route_3_1.get_rich_line<rich_line_type>().size(), 2);
        BOOST_CHECK_CLOSE_FRACTION(route_3_1.length(), 1.0, 1e-3);
    }

    BOOST_FIXTURE_TEST_CASE(matching_test, matcher_fixture<matcher_metric>) {
        auto matching_func = [&](const map_matching_2::matching::settings::MODEL model,
                const std::string &file_prefix) {
            using time_point_type = typename map_matching_2::matching::network_traits<typename
                matcher_metric::network_type>::time_point_type;
            using multi_track = typename map_matching_2::geometry::track::multi_track_type<time_point_type>;
            using line_type = typename multi_track::line_type;

            const std::string matches_filename{file_prefix + "_test_matches.csv"};
            const std::string candidates_filename{file_prefix + "_test_candidates.csv"};
            const std::string compare_filename{file_prefix + "_test_comparison.csv"};

            // match
            {
                network_fixture<network_import_metric> network_fixture;
                network_import_metric network = network_fixture.generate_grid_network();

                network_metric network_matcher = prepare_network(network);
                matcher_metric matcher{
                        network_matcher, map_matching_2::matching::matcher_output_settings{
                                matches_filename, map_matching_2::matching::DEFAULT_COLUMNS,
                                candidates_filename, map_matching_2::matching::DEFAULT_CANDIDATE_COLUMNS,
                                false, true
                        },
                        1
                };
                matcher.start();

                map_matching_2::matching::settings match_settings;
                match_settings.export_timestamps = true;
                match_settings.radius = 1.5;
                match_settings.simplify_track = false;
                match_settings.median_merge = false;
                match_settings.export_candidates = file_prefix + "_test_candidates.csv";
                match_settings.model = model;

                map_matching_2::matching::matcher_forwarder matcher_forwarder{matcher, match_settings};

                auto track = create_track("one", {
                        {2.3, 2.2, 0},
                        {5.1, 2.3, 1},
                        {4.9, 5.8, 2}
                });

                matcher_forwarder.pass(track);

                matcher.stop();
                matcher.join();
            }

            // compare
            {
                line_type ground_truth_line;
                boost::geometry::read_wkt("LINESTRING(2.3 2,3 2,4 2,5 2,5 3,5 4,5 5,5 5.8)", ground_truth_line);
                multi_track ground_truth_track{"one", ground_truth_line};

                map_matching_2::compare::comparator comparator{
                        map_matching_2::compare::comparator_output_settings{
                                compare_filename, map_matching_2::compare::DEFAULT_COLUMNS, false, true
                        },
                        1
                };
                comparator.start();

                map_matching_2::compare::settings compare_settings{};

                map_matching_2::compare::comparator_forwarder_data comparator_data{comparator, compare_settings};

                map_matching_2::compare::comparator_forwarder_matches comparator_matches_forwarder{comparator_data};
                map_matching_2::compare::comparator_forwarder_compares comparator_compares_forwarder{comparator_data};

                map_matching_2::io::csv_settings csv_settings{};
                csv_settings.wkt = true;
                csv_settings.field_geometry = "match";

                map_matching_2::geometry::srs_transform srs_transform{
                        {3857, map_matching_2::geometry::SRS::CS_CARTESIAN},
                        3857, map_matching_2::geometry::SRS::CS_CARTESIAN
                };
                auto reprojector = map_matching_2::geometry::create_point_reprojector(srs_transform);

                map_matching_2::io::track::csv_track_importer matches_importer{
                        std::vector{matches_filename}, csv_settings,
                        comparator_matches_forwarder, srs_transform, reprojector
                };
                matches_importer.read();

                comparator_compares_forwarder.pass(ground_truth_track);

                comparator.stop();
                comparator.join();
            }

            // check
            csv::CSVFormat format;
            format.delimiter(',');
            csv::CSVReader csv{compare_filename, format};

            auto columns = csv.get_col_names();

            for (const auto &row : csv) {
                double correct_fraction = std::stod(row["correct_fraction"].get<std::string>());
                double error_fraction = std::stod(row["error_fraction"].get<std::string>());
                double error_added = std::stod(row["error_added"].get<std::string>());
                double error_missed = std::stod(row["error_missed"].get<std::string>());

                BOOST_CHECK_EQUAL(correct_fraction, 1.0);
                BOOST_CHECK_EQUAL(error_fraction, 0.0);
                BOOST_CHECK_EQUAL(error_added, 0.0);
                BOOST_CHECK_EQUAL(error_missed, 0.0);
            }
        };

        matching_func(map_matching_2::matching::settings::MODEL::VALUE_ITERATION, "value_iteration");
        matching_func(map_matching_2::matching::settings::MODEL::POLICY_ITERATION, "policy_iteration");
        matching_func(map_matching_2::matching::settings::MODEL::VITERBI, "viterbi");
    }

BOOST_AUTO_TEST_SUITE_END()
