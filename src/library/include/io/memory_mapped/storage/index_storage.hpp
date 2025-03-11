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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_INDEX_STORAGE_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_INDEX_STORAGE_HPP

#include "types/geometry/index/network.hpp"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/unique_ptr.hpp>

#include "geometry/traits.hpp"
#include "io/allocator/concepts.hpp"

#include "base_storage.hpp"

namespace map_matching_2::io::memory_mapped::storage {

    template<typename Point, typename Types>
    class index_storage;

    template<typename Point, typename Types> requires
        (is_memory_types<Types>)
    class index_storage<Point, Types> {

    public:
        using point_type = Point;
        using types = Types;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = typename types::template allocator_type<void>;

        using segment_type = typename geometry::models<point_type>::segment_type;

        using rtree_points_type = geometry::rtree_points_type<point_type, types>;
        using rtree_rstar_parameters = typename rtree_points_type::parameters_type;
        using rtree_points_value_type = typename rtree_points_type::value_type;
        using rtree_points_indexable_type = typename rtree_points_type::indexable_getter;
        using rtree_points_equal_to_type = typename rtree_points_type::value_equal;
        using rtree_points_allocator_type = typename rtree_points_type::allocator_type;

        using rtree_segments_type = geometry::rtree_segments_type<segment_type, types>;
        using rtree_segments_value_type = typename rtree_segments_type::value_type;
        using rtree_segments_indexable_type = typename rtree_segments_type::indexable_getter;
        using rtree_segments_equal_to_type = typename rtree_segments_type::value_equal;
        using rtree_segments_allocator_type = typename rtree_segments_type::allocator_type;

        index_storage() {
            _prepare_allocator();
        }

        constexpr index_storage(const index_storage &other) = delete;

        constexpr index_storage(index_storage &&other) noexcept
            : _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _points_index{std::move(other._points_index)}, _segments_index{std::move(other._segments_index)} {
            other._points_index = nullptr;
            other._segments_index = nullptr;
        }

        constexpr index_storage &operator=(const index_storage &other) = delete;

        constexpr index_storage &operator=(index_storage &&other) noexcept {
            if (this != &other) {
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _points_index = std::move(other._points_index);
                _segments_index = std::move(other._segments_index);
                other._points_index = nullptr;
                other._segments_index = nullptr;
            }
            return *this;
        }

        constexpr ~index_storage() = default;

        template<typename Range>
        void build_points_index(const Range &points) {
            _points_index = std::make_unique<rtree_points_type>(points, *_allocator, *_allocator);
        }

        template<typename Range>
        void build_segments_index(const Range &segments) {
            _segments_index = std::make_unique<rtree_segments_type>(segments, *_allocator, *_allocator);
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

        [[nodiscard]] constexpr const rtree_points_type &points_index() const {
            return *_points_index;
        }

        [[nodiscard]] constexpr rtree_points_type &points_index() {
            return *_points_index;
        }

        [[nodiscard]] constexpr const rtree_segments_type &segments_index() const {
            return *_segments_index;
        }

        [[nodiscard]] constexpr rtree_segments_type &segments_index() {
            return *_segments_index;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _points_index;
            ar & _segments_index;
        }

    private:
        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        std::unique_ptr<rtree_points_type> _points_index{};
        std::unique_ptr<rtree_segments_type> _segments_index{};

        void _prepare_allocator() {
            if constexpr (io::allocator::is_counting_allocator<allocator_type>) {
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

    };

    template<typename Point, typename Types> requires
        (is_mmap_types<Types>)
    class index_storage<Point, Types> : public mmap_base<Types> {

    public:
        using point_type = Point;
        using types = Types;

        using base_type = mmap_base<types>;
        using mmap_container_type = typename base_type::mmap_container_type;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = typename types::template allocator_type<void>;

        using segment_type = typename geometry::models<point_type>::segment_type;

        using rtree_points_type = geometry::rtree_points_type<point_type, types>;
        using rtree_rstar_parameters = typename rtree_points_type::parameters_type;
        using rtree_points_value_type = typename rtree_points_type::value_type;
        using rtree_points_indexable_type = typename rtree_points_type::indexable_getter;
        using rtree_points_equal_to_type = typename rtree_points_type::value_equal;
        using rtree_points_allocator_type = typename rtree_points_type::allocator_type;

        using rtree_segments_type = geometry::rtree_segments_type<segment_type, types>;
        using rtree_segments_value_type = typename rtree_segments_type::value_type;
        using rtree_segments_indexable_type = typename rtree_segments_type::indexable_getter;
        using rtree_segments_equal_to_type = typename rtree_segments_type::value_equal;
        using rtree_segments_allocator_type = typename rtree_segments_type::allocator_type;

        struct points_index_wrapper {
            rtree_points_type points_index;
        };

        struct segments_index_wrapper {
            rtree_segments_type segments_index;
        };

        explicit index_storage(
                const std::string &directory = (std::filesystem::temp_directory_path() / "map_matching_2").string(),
                const std::string &index_file = "index.bin",
                const open_or_create_tag open_or_create = OPEN_READ_ONLY,
                const std::size_t index_size = 1024 * 1024,
                const destroy_on_close_tag destroy_on_close = DO_NOT_DESTROY_ON_CLOSE,
                const shrink_on_close_tag shrink_on_close = DO_NOT_SHRINK_ON_CLOSE)
            : base_type{directory}, _container{
                    open_or_create == DESTROY_AND_CREATE
                    ? this->destroy_and_create(index_file, index_size)
                    : this->open_read_only(index_file)
            }, _open_or_create{open_or_create}, _destroy_on_close{destroy_on_close}, _shrink_on_close{shrink_on_close} {
            _prepare_allocator();
            if (_open_or_create == DESTROY_AND_CREATE) {
                _create();
            } else {
                _open();
            }
        }

        constexpr index_storage(const index_storage &other) = delete;

        constexpr index_storage(index_storage &&other) noexcept
            : base_type{std::move(other)}, _container{std::move(other._container)},
            _open_or_create{std::move(other._open_or_create)},
            _destroy_on_close{std::move(other._destroy_on_close)},
            _shrink_on_close{std::move(other._shrink_on_close)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _points_index{std::move(other._points_index)},
            _segments_index{std::move(other._segments_index)} {
            other._points_index = nullptr;
            other._segments_index = nullptr;
        }

        constexpr index_storage &operator=(const index_storage &other) = delete;

        constexpr index_storage &operator=(index_storage &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::move(other));
                _container = std::move(other._container);
                _open_or_create = std::move(other._open_or_create);
                _destroy_on_close = std::move(other._destroy_on_close);
                _shrink_on_close = std::move(other._shrink_on_close);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _points_index = std::move(other._points_index);
                _segments_index = std::move(other._segments_index);
                other._points_index = nullptr;
                other._segments_index = nullptr;
            }
            return *this;
        }

        ~index_storage() {
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
            bool has_points_index{false}, has_segments_index{false};
            if (_points_index) {
                has_points_index = true;
            }
            if (_segments_index) {
                has_segments_index = true;
            }

            _allocator.reset();

            _container.grow(size);

            _open();
            _prepare_allocator();

            if (has_points_index and not _points_index) {
                throw std::runtime_error("could not find points_index.");
            }
            if (has_segments_index and not _segments_index) {
                throw std::runtime_error("could not find segments_index.");
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
        }

        void shrink_to_fit(bool reopen = false, bool flush = false) {
            _container.shrink_to_fit(reopen, flush);
        }

        template<typename Range>
        void build_points_index(const Range &points) {
            _points_index = &_container.storage().template construct<points_index_wrapper>
                    (boost::interprocess::unique_instance)
                    (points_index_wrapper{rtree_points_type{points, *_allocator, *_allocator}})->points_index;
        }

        template<typename Range>
        void build_segments_index(const Range &segments) {
            _segments_index = &_container.storage().template construct<segments_index_wrapper>
                    (boost::interprocess::unique_instance)
                    (segments_index_wrapper{rtree_segments_type{segments, *_allocator, *_allocator}})->segments_index;
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

        [[nodiscard]] constexpr const rtree_points_type &points_index() const {
            return *_points_index;
        }

        [[nodiscard]] constexpr rtree_points_type &points_index() {
            return *_points_index;
        }

        [[nodiscard]] constexpr const rtree_segments_type &segments_index() const {
            return *_segments_index;
        }

        [[nodiscard]] constexpr rtree_segments_type &segments_index() {
            return *_segments_index;
        }

    private:
        mmap_container_type _container;
        open_or_create_tag _open_or_create{OPEN_READ_ONLY};
        destroy_on_close_tag _destroy_on_close{DO_NOT_DESTROY_ON_CLOSE};
        shrink_on_close_tag _shrink_on_close{DO_NOT_SHRINK_ON_CLOSE};

        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        rtree_points_type *_points_index{nullptr};
        rtree_segments_type *_segments_index{nullptr};

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

        void _create() {}

        void _open() {
            _points_index = &_container.storage().template find<points_index_wrapper>
                    (boost::interprocess::unique_instance).first->points_index;
            _segments_index = &_container.storage().template find<segments_index_wrapper>
                    (boost::interprocess::unique_instance).first->segments_index;
        }

    };

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_INDEX_STORAGE_HPP
