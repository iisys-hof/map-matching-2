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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_INDEX_BUILD_STORAGE_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_INDEX_BUILD_STORAGE_HPP

#include <memory>
#include <memory_resource>

#include "geometry/types.hpp"
#include "io/allocator/concepts.hpp"

#include "base_storage.hpp"

namespace map_matching_2::io::memory_mapped::storage {

    template<typename Point, typename Types>
    class index_build_storage;

    template<typename Point, typename Types> requires
        (is_memory_types<Types>)
    class index_build_storage<Point, Types> {

    public:
        using point_type = Point;
        using types = Types;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = typename types::template fast_counting_allocator_type<void>;

        using points_value_type = std::pair<point_type, std::size_t>;

        using segment_type = typename geometry::models<point_type>::segment_type;
        using segments_value_type = std::pair<segment_type, std::pair<std::size_t, std::size_t>>;

        using points_container_type = typename types::template vector_type<points_value_type,
            typename types::template allocator_traits_type<allocator_type>::template rebind_alloc<points_value_type>>;
        using segments_container_type = typename types::template vector_type<segments_value_type,
            typename types::template allocator_traits_type<allocator_type>::template rebind_alloc<segments_value_type>>;

        index_build_storage() {
            _prepare_allocator();
            _create();
        }

        constexpr index_build_storage(const index_build_storage &other) = delete;

        constexpr index_build_storage(index_build_storage &&other) noexcept
            : _upstream{std::move(other._upstream)}, _memory{std::move(other._memory)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _points{std::move(other._points)}, _segments{std::move(other._segments)} {
            other._points = nullptr;
            other._segments = nullptr;
        }

        constexpr index_build_storage &operator=(const index_build_storage &other) = delete;

        constexpr index_build_storage &operator=(index_build_storage &&other) noexcept {
            if (this != &other) {
                _upstream = std::move(other._upstream);
                _memory = std::move(other._memory);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _points = std::move(other._points);
                _segments = std::move(other._segments);
                other._points = nullptr;
                other._segments = nullptr;
            }
            return *this;
        }

        constexpr ~index_build_storage() = default;

        [[nodiscard]] constexpr const allocation_counter_type &allocation_counter() const {
            return *_allocation_counter;
        }

        [[nodiscard]] constexpr allocation_counter_type &allocation_counter() {
            return *_allocation_counter;
        }

        [[nodiscard]] constexpr const allocator_type &allocator() const {
            return *_allocator;
        }

        [[nodiscard]] constexpr allocator_type &allocator() {
            return *_allocator;
        }

        [[nodiscard]] constexpr const points_container_type &points() const {
            return *_points;
        }

        [[nodiscard]] constexpr points_container_type &points() {
            return *_points;
        }

        [[nodiscard]] constexpr const segments_container_type &segments() const {
            return *_segments;
        }

        [[nodiscard]] constexpr segments_container_type &segments() {
            return *_segments;
        }

        [[nodiscard]] std::size_t size() const {
            return _allocation_counter->bytes();
        }

    private:
        std::unique_ptr<std::pmr::monotonic_buffer_resource> _upstream{};
        std::unique_ptr<std::pmr::unsynchronized_pool_resource> _memory{};
        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        std::unique_ptr<points_container_type> _points{};
        std::unique_ptr<segments_container_type> _segments{};

        void _prepare_allocator() {
            if constexpr (io::allocator::is_counting_polymorphic_allocator<allocator_type>) {
                if (not _upstream) {
                    _upstream = std::make_unique<std::pmr::monotonic_buffer_resource>(1024);
                }
                if (not _memory) {
                    _memory = std::make_unique<std::pmr::unsynchronized_pool_resource>(_upstream.get());
                }
                if (not _allocation_counter) {
                    _allocation_counter = std::make_unique<allocation_counter_type>();
                }
                if (not _allocator) {
                    _allocator = std::make_unique<allocator_type>(_memory.get(), *_allocation_counter);
                }
            } else if constexpr (io::allocator::is_counting_allocator<allocator_type>) {
                if (not _allocation_counter) {
                    _allocation_counter = std::make_unique<allocation_counter_type>();
                }
                if (not _allocator) {
                    _allocator = std::make_unique<allocator_type>(*_allocation_counter);
                }
            } else {
                if (not _allocator) {
                    _allocator = std::make_unique<allocator_type>();
                }
            }
        }

        void _create() {
            _points = std::make_unique<points_container_type>(*_allocator);
            _segments = std::make_unique<segments_container_type>(*_allocator);
        }

    };

    template<typename Point, typename Types> requires
        (is_mmap_types<Types>)
    class index_build_storage<Point, Types> : public mmap_base<Types> {

    public:
        using point_type = Point;
        using types = Types;

        using base_type = mmap_base<types>;
        using mmap_container_type = typename base_type::mmap_container_type;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = typename types::template fast_counting_allocator_type<void>;

        using points_value_type = std::pair<point_type, std::size_t>;

        using segment_type = typename geometry::models<point_type>::segment_type;
        using segments_value_type = std::pair<segment_type, std::pair<std::size_t, std::size_t>>;

        using points_container_type = typename types::template vector_type<points_value_type,
            typename types::template allocator_traits_type<allocator_type>::template rebind_alloc<points_value_type>>;
        using segments_container_type = typename types::template vector_type<segments_value_type,
            typename types::template allocator_traits_type<allocator_type>::template rebind_alloc<segments_value_type>>;

        struct points_container_wrapper {
            points_container_type points;
        };

        struct segments_container_wrapper {
            segments_container_type segments;
        };

        explicit index_build_storage(
                const std::string &directory = (std::filesystem::temp_directory_path() / "map_matching_2").string(),
                const std::string &index_build_file = "index_build.bin",
                const open_or_create_tag open_or_create = OPEN_READ_ONLY,
                const std::size_t index_size = 1024 * 1024,
                const destroy_on_close_tag destroy_on_close = DO_NOT_DESTROY_ON_CLOSE,
                const shrink_on_close_tag shrink_on_close = DO_NOT_SHRINK_ON_CLOSE)
            : base_type{directory}, _container{
                    open_or_create == DESTROY_AND_CREATE
                    ? this->destroy_and_create(index_build_file, index_size)
                    : this->open_read_only(index_build_file)
            }, _open_or_create{open_or_create}, _destroy_on_close{destroy_on_close}, _shrink_on_close{shrink_on_close} {
            _prepare_allocator();
            if (_open_or_create == DESTROY_AND_CREATE) {
                _create();
            } else {
                _open();
            }
        }

        constexpr index_build_storage(const index_build_storage &other) = delete;

        constexpr index_build_storage(index_build_storage &&other) noexcept
            : base_type{std::move(other)}, _container{std::move(other._container)},
            _open_or_create{std::move(other._open_or_create)},
            _destroy_on_close{std::move(other._destroy_on_close)},
            _shrink_on_close{std::move(other._shrink_on_close)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _points{std::move(other._points)}, _segments{std::move(other._segments)} {
            other._points = nullptr;
            other._segments = nullptr;
        }

        constexpr index_build_storage &operator=(const index_build_storage &other) = delete;

        constexpr index_build_storage &operator=(index_build_storage &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::move(other));
                _container = std::move(other._container);
                _open_or_create = std::move(other._open_or_create);
                _destroy_on_close = std::move(other._destroy_on_close);
                _shrink_on_close = std::move(other._shrink_on_close);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _points = std::move(other._points);
                _segments = std::move(other._segments);
                other._points = nullptr;
                other._segments = nullptr;
            }
            return *this;
        }

        ~index_build_storage() {
            if (_shrink_on_close == SHRINK_ON_CLOSE and _destroy_on_close == DO_NOT_DESTROY_ON_CLOSE) {
                shrink_to_fit(false, true);
            } else if (_destroy_on_close == DESTROY_ON_CLOSE) {
                _container.destroy();
            } else {
                _container.close();
            }
        }

        [[nodiscard]] std::size_t max_retries() const {
            return _container.max_retries();
        }

        void grow() {
            grow(2 * _container.size());
        }

        void grow(const std::size_t size) {
            _allocator.reset();

            _container.grow(size);

            _prepare_allocator();
            _open();

            if (not _points) {
                throw std::runtime_error("could not find points_container.");
            }
            if (not _segments) {
                throw std::runtime_error("could not find segments_container.");
            }
        }

        void drop_grow() {
            drop_grow(2 * _container.size());
        }

        void drop_grow(const std::size_t size) {
            _allocator.reset();
            _allocation_counter.reset();

            _container.drop_grow(size);

            _prepare_allocator();
            _create();

            if (not _points) {
                throw std::runtime_error("could not find points_container.");
            }
            if (not _segments) {
                throw std::runtime_error("could not find segments_container.");
            }
        }

        void shrink_to_fit(bool reopen = false, bool flush = false) {
            _container.shrink_to_fit(reopen, flush);
        }

        [[nodiscard]] constexpr const allocation_counter_type &allocation_counter() const {
            return *_allocation_counter;
        }

        [[nodiscard]] constexpr allocation_counter_type &allocation_counter() {
            return *_allocation_counter;
        }

        [[nodiscard]] constexpr const allocator_type &allocator() const {
            return *_allocator;
        }

        [[nodiscard]] constexpr allocator_type &allocator() {
            return *_allocator;
        }

        [[nodiscard]] constexpr const points_container_type &points() const {
            return *_points;
        }

        [[nodiscard]] constexpr points_container_type &points() {
            return *_points;
        }

        [[nodiscard]] constexpr const segments_container_type &segments() const {
            return *_segments;
        }

        [[nodiscard]] constexpr segments_container_type &segments() {
            return *_segments;
        }

        [[nodiscard]] std::size_t size() const {
            return _container.size();
        }

    private:
        mmap_container_type _container;
        open_or_create_tag _open_or_create{OPEN_READ_ONLY};
        destroy_on_close_tag _destroy_on_close{DO_NOT_DESTROY_ON_CLOSE};
        shrink_on_close_tag _shrink_on_close{DO_NOT_SHRINK_ON_CLOSE};

        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        points_container_type *_points{nullptr};
        segments_container_type *_segments{nullptr};

        void _prepare_allocator() {
            if constexpr (io::allocator::is_counting_interprocess_allocator<allocator_type>) {
                if (not _allocation_counter) {
                    _allocation_counter = std::make_unique<allocation_counter_type>();
                }
                if (not _allocator) {
                    _allocator = std::make_unique<allocator_type>(_container.storage().get_segment_manager(),
                            *_allocation_counter);
                }
            } else {
                if (not _allocator) {
                    _allocator = std::make_unique<allocator_type>(_container.storage().get_segment_manager());
                }
            }
        }

        void _create() {
            _points = &_container.storage().template construct<points_container_wrapper>
                    (boost::interprocess::unique_instance)
                    (points_container_wrapper{points_container_type{*_allocator}})->points;
            _segments = &_container.storage().template construct<segments_container_wrapper>
                    (boost::interprocess::unique_instance)
                    (segments_container_wrapper{segments_container_type{*_allocator}})->segments;
        }

        void _open() {
            _points = &_container.storage().template find<points_container_wrapper>
                    (boost::interprocess::unique_instance).first->points;
            _segments = &_container.storage().template find<segments_container_wrapper>
                    (boost::interprocess::unique_instance).first->segments;
        }

    };

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_INDEX_BUILD_STORAGE_HPP
