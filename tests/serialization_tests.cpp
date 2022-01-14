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

#include <filesystem>
#include <fstream>
#include <utility>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#define POINTS 100 // crash
// #define POINTS 200 // works

BOOST_AUTO_TEST_SUITE(serialization_tests)

    BOOST_AUTO_TEST_CASE(rtree_points_serialization)
    {
        using point_type = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
        using rtree_points_type = boost::geometry::index::rtree<
                std::pair<point_type, std::size_t>, boost::geometry::index::rstar<16>>;

        std::vector<std::pair<point_type, std::size_t>> points;

        std::size_t index = 0;
        for (std::size_t x = 0; x < POINTS; ++x) {
            for (std::size_t y = 0; y < POINTS; ++y) {
                point_type point{(double) x, (double) y};

                points.emplace_back(std::pair{point, index});
            }
        }

        // build rtree
        rtree_points_type points_index{points};

        // write
        std::filesystem::path file{"rtree_points.txt"};

        std::ofstream file_out;
        file_out.open(file, std::ios_base::trunc);

        if (file_out.is_open() && file_out.good()) {
            boost::archive::text_oarchive out{file_out};
            out << points_index;
        }

        file_out.close();

        // read
        rtree_points_type points_index_read;

        std::ifstream file_in;
        file_in.open(file);

        if (file_in.is_open() && file_in.good()) {
            boost::archive::text_iarchive in{file_in};
            in >> points_index_read;
        }

        file_in.close();

        // remove obsolete test file
        std::filesystem::remove(file);

        BOOST_CHECK_EQUAL(points_index.size(), points_index_read.size());
    }

    BOOST_AUTO_TEST_CASE(rtree_segments_serialization)
    {
        using point_type = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
        using segment_type = boost::geometry::model::segment<point_type>;
        using rtree_segments_type = boost::geometry::index::rtree<
                std::pair<segment_type, std::size_t>, boost::geometry::index::rstar<16>>;

        std::vector<std::pair<segment_type, std::size_t>> segments;

        std::size_t index = 0;
        for (std::size_t x = 0; x < POINTS; ++x) {
            for (std::size_t y = 0; y < POINTS; ++y) {
                point_type a{(double) x, (double) y};
                point_type b{(double) x + 1, (double) y + 2};
                segment_type segment{a, b};

                segments.emplace_back(std::pair{segment, index});
            }
        }

        // build rtree
        rtree_segments_type segments_index{segments};

        // write
        std::filesystem::path file{"rtree_segments.txt"};

        std::ofstream file_out;
        file_out.open(file, std::ios_base::trunc);

        if (file_out.is_open() && file_out.good()) {
            boost::archive::text_oarchive out{file_out};
            out << segments_index;
        }

        file_out.close();

        // read
        rtree_segments_type segments_index_read;

        std::ifstream file_in;
        file_in.open(file);

        if (file_in.is_open() && file_in.good()) {
            boost::archive::text_iarchive in{file_in};
            in >> segments_index_read;
        }

        file_in.close();

        // remove obsolete test file
        std::filesystem::remove(file);

        BOOST_CHECK_EQUAL(segments_index.size(), segments_index_read.size());
    }

BOOST_AUTO_TEST_SUITE_END()

namespace boost::serialization {

    template<typename Archive, typename Point>
    void serialize(Archive &ar, boost::geometry::model::segment<Point> &segment, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP((static_cast<std::pair<Point, Point> &>(segment)));
    }

}
