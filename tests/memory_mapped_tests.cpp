// Copyright (C) 2022 Adrian Wöltche
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

#include <boost/interprocess/managed_mapped_file.hpp>
#include <boost/interprocess/allocators/private_node_allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>

BOOST_AUTO_TEST_SUITE(memory_mapped_tests)

    BOOST_AUTO_TEST_CASE(memory_mapped_file_test) {
        std::string filename{"mmap_file_1.bin"};

        struct mmap_remover {
            std::string filename;

            mmap_remover(std::string filename) : filename{
                    std::move(filename)} { boost::interprocess::file_mapping::remove(this->filename.c_str()); }

            ~mmap_remover() { boost::interprocess::file_mapping::remove(this->filename.c_str()); }
        } remover{filename};

        using point_type = std::pair<double, double>;

        {
            boost::interprocess::managed_mapped_file file{boost::interprocess::open_or_create, filename.c_str(), 1024};

            point_type *a = file.construct<point_type>("a")(1, 2);
            point_type *b = file.construct<point_type>("b")(3, 4);
            point_type *c = file.construct<point_type>("c")(5, 6);

            BOOST_CHECK_EQUAL(a->first, 1);
            BOOST_CHECK_EQUAL(a->second, 2);
            BOOST_CHECK_EQUAL(b->first, 3);
            BOOST_CHECK_EQUAL(b->second, 4);
            BOOST_CHECK_EQUAL(c->first, 5);
            BOOST_CHECK_EQUAL(c->second, 6);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);
        }

        {
            boost::interprocess::managed_mapped_file file{boost::interprocess::open_only, filename.c_str()};

            point_type *a = file.find<point_type>("a").first;
            point_type *b = file.find<point_type>("b").first;
            point_type *c = file.find<point_type>("c").first;

            BOOST_CHECK_EQUAL(a->first, 1);
            BOOST_CHECK_EQUAL(a->second, 2);
            BOOST_CHECK_EQUAL(b->first, 3);
            BOOST_CHECK_EQUAL(b->second, 4);
            BOOST_CHECK_EQUAL(c->first, 5);
            BOOST_CHECK_EQUAL(c->second, 6);

            file.destroy<point_type>("b");

            BOOST_CHECK_EQUAL(a->first, 1);
            BOOST_CHECK_EQUAL(a->second, 2);
            BOOST_CHECK_EQUAL(file.find<point_type>("b").first, nullptr);
            BOOST_CHECK_EQUAL(c->first, 5);
            BOOST_CHECK_EQUAL(c->second, 6);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);
        }
    }

    BOOST_AUTO_TEST_CASE(memory_mapped_file_grow_test) {
        std::string filename{"mmap_file_2.bin"};

        struct mmap_remover {
            std::string filename;

            mmap_remover(std::string filename) : filename{
                    std::move(filename)} { boost::interprocess::file_mapping::remove(this->filename.c_str()); }

            ~mmap_remover() { boost::interprocess::file_mapping::remove(this->filename.c_str()); }
        } remover{filename};

        using point_type = std::pair<double, double>;

        {
            boost::interprocess::managed_mapped_file file{boost::interprocess::open_or_create, filename.c_str(), 1024};

            for (std::size_t i = 0; i <= 5; ++i) {
                file.construct<point_type>(std::to_string(i).c_str())(i, i * i);
            }

            point_type *p = file.find<point_type>("5").first;
            BOOST_CHECK_EQUAL(p->first, 5);
            BOOST_CHECK_EQUAL(p->second, 5 * 5);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);
        }

        {
            boost::interprocess::managed_mapped_file::grow(filename.c_str(), 1024);

            boost::interprocess::managed_mapped_file file{boost::interprocess::open_only, filename.c_str()};

            for (std::size_t i = 6; i <= 10; ++i) {
                file.construct<point_type>(std::to_string(i).c_str())(i, i * i);
            }

            point_type *p = file.find<point_type>("5").first;
            BOOST_CHECK_EQUAL(p->first, 5);
            BOOST_CHECK_EQUAL(p->second, 5 * 5);
            p = file.find<point_type>("10").first;
            BOOST_CHECK_EQUAL(p->first, 10);
            BOOST_CHECK_EQUAL(p->second, 10 * 10);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);
        }

        {
            boost::interprocess::managed_mapped_file::shrink_to_fit(filename.c_str());

            boost::interprocess::managed_mapped_file file{boost::interprocess::open_only, filename.c_str()};

            point_type *p = file.find<point_type>("10").first;
            BOOST_CHECK_EQUAL(p->first, 10);
            BOOST_CHECK_EQUAL(p->second, 10 * 10);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);
        }
    }

    BOOST_AUTO_TEST_CASE(memory_mapped_file_allocation_grow_test) {
        std::string filename{"mmap_file_3.bin"};

        struct mmap_remover {
            std::string filename;

            mmap_remover(std::string filename) : filename{
                    std::move(filename)} { boost::interprocess::file_mapping::remove(this->filename.c_str()); }

            ~mmap_remover() { boost::interprocess::file_mapping::remove(this->filename.c_str()); }
        } remover{filename};

        using point_type = std::pair<double, double>;

        {
            boost::interprocess::managed_mapped_file file{boost::interprocess::open_or_create, filename.c_str(),
                                                          100 * 1024};

            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);

            boost::interprocess::managed_mapped_file::size_type min_size = 100;
            boost::interprocess::managed_mapped_file::size_type received_size = 1000;

            point_type *hint = nullptr;
            point_type *data = file.allocation_command(
                    boost::interprocess::allocate_new, min_size, received_size, hint);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);

            for (std::size_t i = 0; i < received_size; ++i) {
                data[i] = point_type{i, i * i};
            }

            BOOST_CHECK_EQUAL(data[received_size - 1].first, received_size - 1);
            BOOST_CHECK_EQUAL(data[received_size - 1].second, (received_size - 1) * (received_size - 1));

            min_size = received_size;
            boost::interprocess::managed_mapped_file::size_type expanded_size = 5 * received_size;
            point_type *next = file.allocation_command(
                    boost::interprocess::expand_fwd, min_size, received_size, data);

            BOOST_CHECK_EQUAL(next, data);
            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);

            for (std::size_t i = received_size; i < expanded_size; ++i) {
                data[i] = point_type{i, i * i};
            }

            BOOST_CHECK_EQUAL(data[expanded_size - 1].first, expanded_size - 1);
            BOOST_CHECK_EQUAL(data[expanded_size - 1].second, (expanded_size - 1) * (expanded_size - 1));
            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);
        }
    }

    BOOST_AUTO_TEST_CASE(memory_mapped_file_vector_test) {
        std::string filename{"mmap_file_4.bin"};

        struct mmap_remover {
            std::string filename;

            mmap_remover(std::string filename) : filename{
                    std::move(filename)} { boost::interprocess::file_mapping::remove(this->filename.c_str()); }

            ~mmap_remover() { boost::interprocess::file_mapping::remove(this->filename.c_str()); }
        } remover{filename};

        using point_type = std::pair<double, double>;

        using allocator_type = boost::interprocess::private_node_allocator<point_type,
                boost::interprocess::managed_mapped_file::segment_manager>;

        using vector_type = boost::interprocess::vector<point_type, allocator_type>;

        {
            boost::interprocess::managed_mapped_file file{boost::interprocess::open_or_create, filename.c_str(),
                                                          100 * 1024};

            allocator_type allocator{file.get_segment_manager()};

            vector_type *vector = file.construct<vector_type>("points")(allocator);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);

            for (std::size_t i = 0; i < 1000; ++i) {
                vector->emplace_back(point_type{i, i * i});
            }

            BOOST_CHECK_EQUAL(vector->size(), 1000);
            BOOST_CHECK_EQUAL(vector->back().first, 999);
            BOOST_CHECK_EQUAL(vector->back().second, 999 * 999);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);
        }

        {
            boost::interprocess::managed_mapped_file file{boost::interprocess::open_only, filename.c_str()};

            allocator_type allocator{file.get_segment_manager()};

            vector_type *vector = file.find<vector_type>("points").first;

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);

            BOOST_CHECK_EQUAL(vector->size(), 1000);
            BOOST_CHECK_EQUAL(vector->back().first, 999);
            BOOST_CHECK_EQUAL(vector->back().second, 999 * 999);

            for (std::size_t i = 999; i >= 500; --i) {
                vector->pop_back();
            }

            vector->shrink_to_fit();

            BOOST_CHECK_EQUAL(vector->size(), 500);
            BOOST_CHECK_EQUAL(vector->back().first, 499);
            BOOST_CHECK_EQUAL(vector->back().second, 499 * 499);

            BOOST_TEST_MESSAGE("file size: " << file.get_size());
            BOOST_TEST_MESSAGE("file free size: " << file.get_free_memory());
            BOOST_CHECK_GE(file.get_size(), 0);
            BOOST_CHECK_GE(file.get_free_memory(), 0);
        }
    }

BOOST_AUTO_TEST_SUITE_END()
