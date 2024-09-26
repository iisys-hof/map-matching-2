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

#include <boost/test/unit_test.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/cached_adaptive_pool.hpp>
#include <boost/interprocess/containers/vector.hpp>

BOOST_AUTO_TEST_SUITE(memory_mapped_wrapper_tests)

    BOOST_AUTO_TEST_CASE(memory_mapped_wrapper_test) {

        struct shm_remove {
            shm_remove() { boost::interprocess::shared_memory_object::remove("memory"); }

            ~shm_remove() { boost::interprocess::shared_memory_object::remove("memory"); }
        } remover;

        using point_type = boost::geometry::model::point<double, 2, boost::geometry::cs::geographic<
            boost::geometry::degree>>;

        using segment_manager_type = boost::interprocess::managed_shared_memory::segment_manager;
        using point_allocator_type = boost::interprocess::cached_adaptive_pool<point_type, segment_manager_type>;
        using points_type = boost::interprocess::vector<point_type, point_allocator_type>;

        struct wrapper {
            points_type data;
        };

        {
            auto *memory = new boost::interprocess::managed_shared_memory{
                    boost::interprocess::open_or_create, "memory", 1024
            };
            auto *allocator = new point_allocator_type{memory->get_segment_manager()};
            auto *wrap = memory->find_or_construct<wrapper>("identifier")(wrapper{points_type{*allocator}});

            auto &data = wrap->data;
            data.emplace_back(point_type{1, 2});

            BOOST_CHECK_EQUAL(boost::geometry::get<0>(data.at(0)), 1);
            BOOST_CHECK_EQUAL(boost::geometry::get<1>(data.at(0)), 2);

            delete allocator;
            delete memory;
        }

        {
            auto *memory = new boost::interprocess::managed_shared_memory{
                    boost::interprocess::open_only, "memory"
            };
            auto *allocator = new point_allocator_type{memory->get_segment_manager()};
            auto *wrap = memory->find<wrapper>("identifier").first;

            auto &data = wrap->data;

            BOOST_CHECK_EQUAL(boost::geometry::get<0>(data.at(0)), 1);
            BOOST_CHECK_EQUAL(boost::geometry::get<1>(data.at(0)), 2);

            delete allocator;
            delete memory;
        }
    }

BOOST_AUTO_TEST_SUITE_END()
