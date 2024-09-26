// Copyright (C) 2023-2024 Adrian Wöltche
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

#include <random>

#include <boost/test/unit_test.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "types/io/memory_mapped/types.hpp"
#include "io/memory_mapped/utils.hpp"

const std::size_t area_size = 1000;
const std::size_t vertices_size = 1000;

const char points_index_file[] = "points_index.bin";
std::size_t points_index_size = 100 * 1024;
const char points_index_name[] = "points_rtree";

using point_type = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;

using mmap_types = map_matching_2::io::memory_mapped::file_types;
using managed_mapped_file = map_matching_2::io::memory_mapped::managed_mapped_file;
using segment_manager_type = mmap_types::mmap_segment_manager_type;

using points_index_value_type = std::pair<point_type, std::size_t>;
using points_index_params_type = boost::geometry::index::rstar<16>;
using points_index_indexable_type = boost::geometry::index::indexable<points_index_value_type>;
using points_index_equal_to_type = boost::geometry::index::equal_to<points_index_value_type>;
using points_index_allocator_type = mmap_types::allocator_type<points_index_value_type>;

using points_index_type = boost::geometry::index::rtree<
    points_index_value_type, points_index_params_type, points_index_indexable_type,
    points_index_equal_to_type, points_index_allocator_type>;

std::random_device random_device;
std::default_random_engine random_engine{random_device()};
std::uniform_int_distribution<std::size_t> random_index{0, vertices_size - 1};
std::uniform_real_distribution<double> random_double{-((double) area_size), (double) area_size};

BOOST_AUTO_TEST_SUITE(memory_mapped_index_tests)

    BOOST_AUTO_TEST_CASE(memory_mapped_index_rtree_test) {
        std::vector<points_index_value_type> points;
        points.reserve(vertices_size);
        for (std::size_t i = 0; i < points.capacity(); ++i) {
            points.emplace_back(
                    points_index_value_type{point_type{random_double(random_engine), random_double(random_engine)}, i});
        }

        map_matching_2::io::memory_mapped::mmap_start_remover mmap_points_index_remover{points_index_file};

        {
            auto *points_index_mmap_file = new managed_mapped_file{
                    boost::interprocess::open_or_create, points_index_file, points_index_size
            };
            auto *points_index_allocator = new points_index_allocator_type{
                    points_index_mmap_file->get_segment_manager()
            };

            BOOST_TEST_MESSAGE("points index file size: " << points_index_mmap_file->get_size());
            BOOST_TEST_MESSAGE("points index file free size: " << points_index_mmap_file->get_free_memory());

            auto *points_index = points_index_mmap_file->construct<points_index_type>(points_index_name)(
                    points.begin(), points.end(), *points_index_allocator, std::allocator<points_index_value_type>{});

            BOOST_TEST_MESSAGE("points index file size: " << points_index_mmap_file->get_size());
            BOOST_TEST_MESSAGE("points index file free size: " << points_index_mmap_file->get_free_memory());
            BOOST_TEST_MESSAGE("points index size: " << points_index->size());
            BOOST_CHECK_EQUAL(vertices_size, points_index->size());

            delete points_index_allocator;
            delete points_index_mmap_file;
        }

        managed_mapped_file::shrink_to_fit(points_index_file);

        {
            auto *points_index_mmap_file = new managed_mapped_file{
                    boost::interprocess::open_only, points_index_file
            };
            auto *points_index = points_index_mmap_file->find<points_index_type>(points_index_name).first;

            BOOST_TEST_MESSAGE("points index file size: " << points_index_mmap_file->get_size());
            BOOST_TEST_MESSAGE("points index file free size: " << points_index_mmap_file->get_free_memory());
            BOOST_TEST_MESSAGE("points index size: " << points_index->size());
            BOOST_CHECK_EQUAL(vertices_size, points_index->size());

            delete points_index_mmap_file;
        }

    }

BOOST_AUTO_TEST_SUITE_END()
