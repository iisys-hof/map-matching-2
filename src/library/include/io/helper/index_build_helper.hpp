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

#ifndef MAP_MATCHING_2_IO_HELPER_INDEX_BUILD_HELPER_HPP
#define MAP_MATCHING_2_IO_HELPER_INDEX_BUILD_HELPER_HPP

#include "util/concepts.hpp"

#include "io/memory_mapped/concepts.hpp"
#include "io/memory_mapped/utils.hpp"

namespace map_matching_2::io::helper {

    struct index_build_helper_dummy {};

    template<typename IndexBuildHelper>
    concept is_dummy_index_build_helper = std::same_as<IndexBuildHelper, index_build_helper_dummy>;

    template<typename IndexBuildHelper>
    concept is_memory_index_build_helper = io::memory_mapped::is_memory_types<typename
        IndexBuildHelper::index_build_storage_types>;

    template<typename IndexBuildHelper>
    concept is_mmap_index_build_helper = io::memory_mapped::is_mmap_types<typename
        IndexBuildHelper::index_build_storage_types>;

    template<typename IndexBuildStorage>
    class index_build_helper {

    public:
        using index_build_storage_type = IndexBuildStorage;
        using index_build_storage_types = typename index_build_storage_type::types;

        using point_type = typename index_build_storage_type::point_type;
        using allocator_type = typename index_build_storage_type::allocator_type;

        using points_value_type = typename index_build_storage_type::points_value_type;

        using segment_type = typename index_build_storage_type::segment_type;
        using segments_value_type = typename index_build_storage_type::segments_value_type;

        using points_container_type = typename index_build_storage_type::points_container_type;
        using segments_container_type = typename index_build_storage_type::segments_container_type;

        constexpr index_build_helper(index_build_storage_type index_build_storage = {})
            : _index_build_storage{std::move(index_build_storage)} {}

        constexpr index_build_helper(const index_build_helper &other) = delete;

        constexpr index_build_helper(index_build_helper &&other) noexcept
            : _index_build_storage(std::move(other._index_build_storage)) {}

        constexpr index_build_helper &operator=(const index_build_helper &other) = delete;

        constexpr index_build_helper &operator=(index_build_helper &&other) noexcept {
            if (this != &other) {
                _index_build_storage = std::move(other._index_build_storage);
            }
            return *this;
        }

        constexpr ~index_build_helper() = default;

        [[nodiscard]] constexpr const index_build_storage_type &index_build_storage() const {
            return _index_build_storage;
        }

        [[nodiscard]] constexpr index_build_storage_type &index_build_storage() {
            return _index_build_storage;
        }

        [[nodiscard]] constexpr const points_container_type &points() const {
            return index_build_storage().points();
        }

        [[nodiscard]] constexpr points_container_type &points() {
            return index_build_storage().points();
        }

        [[nodiscard]] constexpr const segments_container_type &segments() const {
            return index_build_storage().segments();
        }

        [[nodiscard]] constexpr segments_container_type &segments() {
            return index_build_storage().segments();
        }

        [[nodiscard]] constexpr const allocator_type &allocator() const {
            return index_build_storage().allocator();
        }

        [[nodiscard]] constexpr allocator_type &allocator() {
            return index_build_storage().allocator();
        }

        void reserve_points(std::size_t size) {
            if constexpr (is_mmap_index_build_helper<index_build_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            points().reserve(size);
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        }, _critical, index_build_storage().max_retries());
            } else if constexpr (is_memory_index_build_helper<index_build_helper>) {
                points().reserve(size);
            } else {
                static_assert(util::dependent_false_v<index_build_helper>, "index_build_helper type unknown");
            }
        }

        void reserve_segments(std::size_t size) {
            if constexpr (is_mmap_index_build_helper<index_build_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            segments().reserve(size);
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        }, _critical, index_build_storage().max_retries());
            } else if constexpr (is_memory_index_build_helper<index_build_helper>) {
                segments().reserve(size);
            } else {
                static_assert(util::dependent_false_v<index_build_helper>, "index_build_helper type unknown");
            }
        }

        void add_point(points_value_type point) {
            if constexpr (is_mmap_index_build_helper<index_build_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            points().emplace_back(std::move(point));
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        }, _critical);
            } else if constexpr (is_memory_index_build_helper<index_build_helper>) {
                points().emplace_back(std::move(point));
            } else {
                static_assert(util::dependent_false_v<index_build_helper>, "index_build_helper type unknown");
            }
        }

        void add_segment(segments_value_type segment) {
            if constexpr (is_mmap_index_build_helper<index_build_helper>) {
                return io::memory_mapped::retry_alloc([&]() {
                            segments().emplace_back(std::move(segment));
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        }, _critical);
            } else if constexpr (is_memory_index_build_helper<index_build_helper>) {
                segments().emplace_back(std::move(segment));
            } else {
                static_assert(util::dependent_false_v<index_build_helper>, "index_build_helper type unknown");
            }
        }

        [[nodiscard]] std::size_t size() const {
            return _index_build_storage.size();
        }

    private:
        index_build_storage_type _index_build_storage;
        bool _critical{false};

        void handle_bad_alloc(auto &ex) {
            if constexpr (io::memory_mapped::has_grow<index_build_storage_type>) {
                std::size_t previous_points_size = points().size();
                std::size_t previous_segments_size = segments().size();
                index_build_storage().grow();
                if (points().size() != previous_points_size or
                    segments().size() != previous_segments_size) {
                    std::rethrow_exception(std::current_exception());
                }
            } else {
                std::rethrow_exception(std::current_exception());
            }
        }

    };

}

#endif //MAP_MATCHING_2_IO_HELPER_INDEX_BUILD_HELPER_HPP
