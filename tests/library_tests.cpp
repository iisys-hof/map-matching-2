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

#include <boost/geometry.hpp>

BOOST_AUTO_TEST_SUITE(library_tests)

    template<typename CT>
    void closest_path_tester(const std::string point_wkt, const std::string segment_wkt, const double error) {
        using point_type = boost::geometry::model::point<CT, 2, boost::geometry::cs::geographic<boost::geometry::degree>>;
        using segment_type = boost::geometry::model::segment<point_type>;

        boost::geometry::strategy::closest_points::geographic_cross_track<> closest_point_strategy;
        boost::geometry::strategy::distance::geographic_cross_track<> cross_track_strategy;
        boost::geometry::strategy::distance::geographic<> distance_strategy;

        point_type point;
        segment_type segment;

        boost::geometry::read_wkt(point_wkt, point);
        boost::geometry::read_wkt(segment_wkt, segment);

        const auto distance = boost::geometry::distance(point, segment, cross_track_strategy);

        segment_type projection;
        boost::geometry::closest_points(point, segment, projection, closest_point_strategy);
        const auto length = boost::geometry::length(projection, distance_strategy);

        std::cout << std::setprecision(15)
                  << "Point: " << boost::geometry::wkt(point) << "\n"
                  << "Segment: " << boost::geometry::wkt(segment) << "\n"
                  << "Distance: " << distance << "\n"
                  << "Projection: " << boost::geometry::wkt(projection) << "\n"
                  << "Length: " << length << std::endl;

        BOOST_CHECK_CLOSE_FRACTION(distance, length, error);
    }

    BOOST_AUTO_TEST_CASE(closest_path_test_1)
    {
        auto point_wkt = "POINT(11.845747600604916272 50.303247769986953131)";
        auto segment_wkt = "SEGMENT(11.8449176 50.3030635,11.8458063 50.3032608)";

        closest_path_tester<double>(point_wkt, segment_wkt, 1e-20);
        closest_path_tester<long double>(point_wkt, segment_wkt, 1e-20);
    }

    BOOST_AUTO_TEST_CASE(closest_path_test_2)
    {
        auto point_wkt = "POINT(11.921418190002441406 50.316425323486328125)";
        auto segment_wkt = "SEGMENT(11.9214920 50.3161678,11.9212341 50.3161381)";

        closest_path_tester<double>(point_wkt, segment_wkt, 1e-20);
        closest_path_tester<long double>(point_wkt, segment_wkt, 1e-8);
    }

    BOOST_AUTO_TEST_CASE(closest_path_test_3)
    {
        auto point_wkt = "POINT(11.904624124918561169 50.317349861000025692)";
        auto segment_wkt = "SEGMENT(11.9046808 50.3173523,11.9045925 50.3173485)";

        closest_path_tester<double>(point_wkt, segment_wkt, 1e-20);
        closest_path_tester<long double>(point_wkt, segment_wkt, 1e-20);
    }

    BOOST_AUTO_TEST_CASE(closest_path_test_4)
    {
        auto point_wkt = "POINT(11.907328887553041017 50.311933736642272308)";
        auto segment_wkt = "SEGMENT(11.9072659 50.3119291,11.9074099 50.3119397)";

        closest_path_tester<double>(point_wkt, segment_wkt, 1e-20);
        closest_path_tester<long double>(point_wkt, segment_wkt, 1e-20);
    }

    BOOST_AUTO_TEST_CASE(closest_path_test_5)
    {
        auto point_wkt = "POINT(11.894846916198730469 50.316379547119140625)";
        auto segment_wkt = "SEGMENT(11.8958402 50.3155918,11.8953426 50.3155504)";

        closest_path_tester<double>(point_wkt, segment_wkt, 1e-6);
        closest_path_tester<long double>(point_wkt, segment_wkt, 1e-20);
    }

    BOOST_AUTO_TEST_CASE(closest_path_test_6)
    {
        auto point_wkt = "POINT(11.914519782008024862 50.319138765234583843)";
        auto segment_wkt = "SEGMENT(11.9145157 50.3191017,11.9145257 50.3191925)";

        closest_path_tester<double>(point_wkt, segment_wkt, 1e-6);
        closest_path_tester<long double>(point_wkt, segment_wkt, 1e-20);
    }

BOOST_AUTO_TEST_SUITE_END()