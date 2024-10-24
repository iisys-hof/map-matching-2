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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_OSM_HANDLER_STORAGE_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_OSM_HANDLER_STORAGE_HPP

#include <memory>
#include <memory_resource>

#include <boost/serialization/serialization.hpp>

#include "geometry/network/traits/node.hpp"
#include "io/allocator/concepts.hpp"

#include "base_storage.hpp"

namespace map_matching_2::io::memory_mapped::storage {

    template<typename Vertex>
    struct osm_vertex {
        using vertex_type = Vertex;
        using vertex_size_type = std::size_t;

        constexpr explicit osm_vertex(vertex_type vertex)
            : vertex{std::move(vertex)} {}

        template<typename... Args>
        constexpr explicit osm_vertex(Args &&... args)
            : vertex{std::forward<Args>(args)...} {}

        constexpr osm_vertex(const osm_vertex &other) = delete;

        constexpr osm_vertex(osm_vertex &&other) noexcept = default;

        constexpr osm_vertex &operator=(const osm_vertex &other) = delete;

        constexpr osm_vertex &operator=(osm_vertex &&other) noexcept = default;

        constexpr ~osm_vertex() = default;

        vertex_type vertex;
        vertex_size_type vertex_index{};
        bool added{false};
    };

    template<typename Vertex, typename Types>
    class osm_handler_storage;

    template<typename Vertex, typename Types> requires
        (is_memory_types<Types>)
    class osm_handler_storage<Vertex, Types> {

    public:
        using types = Types;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = typename types::template fast_counting_allocator_type<void>;

        using vertex_type = Vertex;

        using osm_vertex_type = osm_vertex<vertex_type>;
        using vertex_container_type = typename types::template vector_type<osm_vertex_type,
            typename types::template allocator_traits_type<allocator_type>::template rebind_alloc<osm_vertex_type>>;

        osm_handler_storage() {
            _prepare_allocator();
            _create();
        }

        constexpr osm_handler_storage(const osm_handler_storage &other) = delete;

        constexpr osm_handler_storage(osm_handler_storage &&other) noexcept
            : _upstream{std::move(other._upstream)}, _memory{std::move(other._memory)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _vertices{std::move(other._vertices)} {
            other._vertices = nullptr;
        }

        constexpr osm_handler_storage &operator=(const osm_handler_storage &other) = delete;

        constexpr osm_handler_storage &operator=(osm_handler_storage &&other) noexcept {
            if (this != &other) {
                _upstream = std::move(other._upstream);
                _memory = std::move(other._memory);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _vertices = std::move(other._vertices);
                other._vertices = nullptr;
            }
            return *this;
        }

        constexpr ~osm_handler_storage() = default;

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

        [[nodiscard]] constexpr const vertex_container_type &vertices() const {
            return *_vertices;
        }

        [[nodiscard]] constexpr vertex_container_type &vertices() {
            return *_vertices;
        }

    private:
        std::unique_ptr<std::pmr::monotonic_buffer_resource> _upstream{};
        std::unique_ptr<std::pmr::unsynchronized_pool_resource> _memory{};
        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        std::unique_ptr<vertex_container_type> _vertices{};

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
            _vertices = std::make_unique<vertex_container_type>(*_allocator);
        }

    };

    template<typename Vertex, typename Types> requires
        (is_mmap_types<Types>)
    class osm_handler_storage<Vertex, Types> : public mmap_base<Types> {

    public:
        using types = Types;
        using base_type = mmap_base<types>;
        using mmap_container_type = typename base_type::mmap_container_type;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = typename types::template fast_counting_allocator_type<void>;

        using vertex_type = Vertex;

        using osm_vertex_type = osm_vertex<vertex_type>;
        using vertex_container_type = typename types::template vector_type<osm_vertex_type,
            typename types::template allocator_traits_type<allocator_type>::template rebind_alloc<osm_vertex_type>>;

        struct osm_vertex_wrapper {
            vertex_container_type vertices;
        };

        explicit osm_handler_storage(
                const std::string &directory = (std::filesystem::temp_directory_path() / "map_matching_2").string(),
                const std::string &osm_file = "osm_vertices.bin",
                const open_or_create_tag open_or_create = OPEN_READ_ONLY,
                const std::size_t osm_size = 1024 * 1024,
                const destroy_on_close_tag destroy_on_close = DO_NOT_DESTROY_ON_CLOSE,
                const shrink_on_close_tag shrink_on_close = DO_NOT_SHRINK_ON_CLOSE)
            : base_type{directory}, _container{
                    open_or_create == DESTROY_AND_CREATE
                    ? this->destroy_and_create(osm_file, osm_size)
                    : this->open_read_only(osm_file)
            }, _open_or_create{open_or_create}, _destroy_on_close{destroy_on_close}, _shrink_on_close{shrink_on_close} {
            _prepare_allocator();
            if (_open_or_create == DESTROY_AND_CREATE) {
                _create();
            } else {
                _open();
            }
        }

        constexpr osm_handler_storage(const osm_handler_storage &other) = delete;

        constexpr osm_handler_storage(osm_handler_storage &&other) noexcept
            : base_type{std::move(other)}, _container{std::move(other._container)},
            _open_or_create{std::move(other._open_or_create)},
            _destroy_on_close{std::move(other._destroy_on_close)},
            _shrink_on_close{std::move(other._shrink_on_close)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _vertices{std::move(other._vertices)} {
            other._vertices = nullptr;
        }

        constexpr osm_handler_storage &operator=(const osm_handler_storage &other) = delete;

        constexpr osm_handler_storage &operator=(osm_handler_storage &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::move(other));
                _container = std::move(other._container);
                _open_or_create = std::move(other._open_or_create);
                _destroy_on_close = std::move(other._destroy_on_close);
                _shrink_on_close = std::move(other._shrink_on_close);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _vertices = std::move(other._vertices);
                other._vertices = nullptr;
            }
            return *this;
        }

        ~osm_handler_storage() {
            if (_shrink_on_close == SHRINK_ON_CLOSE and _destroy_on_close == DO_NOT_DESTROY_ON_CLOSE) {
                shrink_to_fit(false);
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

            if (not _vertices) {
                throw std::runtime_error("could not find osm_vertices.");
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

            if (not _vertices) {
                throw std::runtime_error("could not find osm_vertices.");
            }
        }

        void shrink_to_fit(bool reopen = true) {
            _container.shrink_to_fit(reopen);
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

        [[nodiscard]] constexpr const vertex_container_type &vertices() const {
            return *_vertices;
        }

        [[nodiscard]] constexpr vertex_container_type &vertices() {
            return *_vertices;
        }

    private:
        mmap_container_type _container;
        open_or_create_tag _open_or_create{OPEN_READ_ONLY};
        destroy_on_close_tag _destroy_on_close{DO_NOT_DESTROY_ON_CLOSE};
        shrink_on_close_tag _shrink_on_close{DO_NOT_SHRINK_ON_CLOSE};

        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        vertex_container_type *_vertices{nullptr};

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
            _vertices = &_container.storage().template construct<osm_vertex_wrapper>
                    (boost::interprocess::unique_instance)
                    (osm_vertex_wrapper{vertex_container_type{*_allocator}})->vertices;
        }

        void _open() {
            _vertices = &_container.storage().template find<osm_vertex_wrapper>
                    (boost::interprocess::unique_instance).first->vertices;
        }

    };

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_OSM_HANDLER_STORAGE_HPP
