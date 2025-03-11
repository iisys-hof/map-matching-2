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

#ifndef MAP_MATCHING_2_IO_HELPER_INDEX_HELPER_HPP
#define MAP_MATCHING_2_IO_HELPER_INDEX_HELPER_HPP

#include <boost/serialization/serialization.hpp>

#include "util/concepts.hpp"

#include "io/memory_mapped/concepts.hpp"
#include "io/memory_mapped/utils.hpp"

#include "index_build_helper.hpp"

namespace map_matching_2::io::helper {

    template<typename IndexHelper>
    concept is_memory_index_helper = io::memory_mapped::is_memory_types<typename IndexHelper::index_storage_types>;

    template<typename IndexHelper>
    concept is_mmap_index_helper = io::memory_mapped::is_mmap_types<typename IndexHelper::index_storage_types>;

    template<typename IndexStorage, typename IndexBuildHelper>
    class index_helper {

    public:
        using index_storage_type = IndexStorage;
        using index_build_helper_type = IndexBuildHelper;
        using index_storage_types = typename index_storage_type::types;

        using point_type = typename index_storage_type::point_type;
        using allocator_type = typename index_storage_type::allocator_type;

        using rtree_rstar_parameters = typename index_storage_type::rtree_rstar_parameters;

        using rtree_points_value_type = typename index_storage_type::rtree_points_value_type;
        using rtree_points_indexable_type = typename index_storage_type::rtree_points_indexable_type;
        using rtree_points_equal_to_type = typename index_storage_type::rtree_points_equal_to_type;
        using rtree_points_allocator_type = typename index_storage_type::rtree_points_allocator_type;
        using rtree_points_type = typename index_storage_type::rtree_points_type;

        using rtree_segments_value_type = typename index_storage_type::rtree_segments_value_type;
        using rtree_segments_indexable_type = typename index_storage_type::rtree_segments_indexable_type;
        using rtree_segments_equal_to_type = typename index_storage_type::rtree_segments_equal_to_type;
        using rtree_segments_allocator_type = typename index_storage_type::rtree_segments_allocator_type;
        using rtree_segments_type = typename index_storage_type::rtree_segments_type;

        constexpr index_helper(index_storage_type index_storage = {}, index_build_helper_type index_build_helper = {})
            : _index_storage{std::move(index_storage)}, _index_build_helper{std::move(index_build_helper)} {}

        constexpr index_helper(const index_helper &other) = delete;

        constexpr index_helper(index_helper &&other) noexcept
            : _index_storage{std::move(other._index_storage)},
            _index_build_helper{std::move(other._index_build_helper)} {}

        constexpr index_helper &operator=(const index_helper &other) = delete;

        constexpr index_helper &operator=(index_helper &&other) noexcept {
            if (this != &other) {
                _index_storage = std::move(other._index_storage);
                _index_build_helper = std::move(other._index_build_helper);
            }
            return *this;
        }

        constexpr ~index_helper() = default;

        [[nodiscard]] constexpr const index_storage_type &index_storage() const {
            return _index_storage;
        }

        [[nodiscard]] constexpr index_storage_type &index_storage() {
            return _index_storage;
        }

        [[nodiscard]] constexpr const index_build_helper_type &index_build_helper() const {
            return _index_build_helper;
        }

        [[nodiscard]] constexpr index_build_helper_type &index_build_helper() {
            return _index_build_helper;
        }

        [[nodiscard]] constexpr const rtree_points_type &points_index() const {
            return index_storage().points_index();
        }

        [[nodiscard]] constexpr rtree_points_type &points_index() {
            return index_storage().points_index();
        }

        [[nodiscard]] constexpr const rtree_segments_type &segments_index() const {
            return index_storage().segments_index();
        }

        [[nodiscard]] constexpr rtree_segments_type &segments_index() {
            return index_storage().segments_index();
        }

        [[nodiscard]] constexpr const allocator_type &allocator() const {
            return index_storage().allocator();
        }

        [[nodiscard]] constexpr allocator_type &allocator() {
            return index_storage().allocator();
        }

        void build_indices() requires (not is_dummy_index_build_helper<index_build_helper_type>) {
            if constexpr (is_mmap_index_helper<index_helper>) {
                // grow index storage with the help of the index build storage size
                const auto grow_size = 2 * index_build_helper().size();
                if constexpr (is_mmap_index_build_helper<index_build_helper_type>) {
                    index_storage().grow(grow_size);
                } else {
                    index_storage().grow(2 * grow_size);
                }

                return io::memory_mapped::retry_alloc([&]() {
                            index_storage().build_points_index(index_build_helper().points());
                            index_storage().build_segments_index(index_build_helper().segments());
                        }, [&](auto &ex) {
                            drop_handle_bad_alloc(ex);
                        }, _critical, index_storage().max_retries());
            } else if constexpr (is_memory_index_helper<index_helper>) {
                index_storage().build_points_index(index_build_helper().points());
                index_storage().build_segments_index(index_build_helper().segments());
            } else {
                static_assert(util::dependent_false_v<index_helper>, "index_helper type unknown");
            }
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) requires
            (is_memory_index_helper<index_helper>) {
            ar & _index_storage;
        }

    private:
        index_storage_type _index_storage;
        index_build_helper_type _index_build_helper;
        bool _critical{false};

        void drop_handle_bad_alloc(auto &ex) {
            if constexpr (io::memory_mapped::has_grow<index_storage_type>) {
                index_storage().drop_grow();
                std::cout << "INFO: graph_storage.drop_grow() called." << std::endl;
            } else {
                std::rethrow_exception(std::current_exception());
            }
        }

    };

}

#endif //MAP_MATCHING_2_IO_HELPER_INDEX_HELPER_HPP
