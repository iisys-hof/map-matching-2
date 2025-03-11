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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_GRAPH_STORAGE_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_GRAPH_STORAGE_HPP

#include <memory>
#include <memory_resource>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/unique_ptr.hpp>

#include "graph/traits/graph.hpp"
#include "io/allocator/concepts.hpp"
#include "util/compiler.hpp"

#include "base_storage.hpp"

namespace map_matching_2::io::memory_mapped::storage {

    template<typename Graph, typename Types>
    class graph_storage;

    template<typename Graph, typename Types> requires
        (is_memory_types<Types>)
    class graph_storage<Graph, Types> {

    public:
        using graph_type = Graph;
        using types = Types;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = typename graph::graph_traits<graph_type>::allocator_type;

        template<typename Type>
        using allocator_traits_type = typename graph::graph_traits<graph_type>::template allocator_traits_type<Type>;

        using vertex_descriptor = typename graph::graph_traits<graph_type>::vertex_descriptor;
        using vertex_index_map_type = typename types::template vector_type<vertex_descriptor,
            typename allocator_traits_type<allocator_type>::template rebind_alloc<vertex_descriptor>>;

        graph_storage() {
            _prepare_allocator();
            _create();
        }

        constexpr graph_storage(const graph_storage &other) = delete;

        constexpr graph_storage(graph_storage &&other) noexcept
            : _upstream{std::move(other._upstream)}, _memory{std::move(other._memory)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _graph{std::move(other._graph)}, _vertex_index_map{std::move(other._vertex_index_map)} {
            other._graph = nullptr;
            other._vertex_index_map = nullptr;
        }

        constexpr graph_storage &operator=(const graph_storage &other) = delete;

        constexpr graph_storage &operator=(graph_storage &&other) noexcept {
            if (this != &other) {
                _upstream = std::move(other._upstream);
                _memory = std::move(other._memory);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _graph = std::move(other._graph);
                _vertex_index_map = std::move(other._vertex_index_map);
                other._graph = nullptr;
                other._vertex_index_map = nullptr;
            }
            return *this;
        }

        constexpr ~graph_storage() = default;

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

        [[nodiscard]] constexpr const graph_type &graph() const {
            return *_graph;
        }

        [[nodiscard]] constexpr graph_type &graph() {
            return *_graph;
        }

        [[nodiscard]] constexpr const vertex_index_map_type &vertex_index_map() const {
            return *_vertex_index_map;
        }

        [[nodiscard]] constexpr vertex_index_map_type &vertex_index_map() {
            return *_vertex_index_map;
        }

        [[nodiscard]] std::size_t size() const {
            return _allocation_counter->bytes();
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _graph;
        }

    private:
        std::unique_ptr<std::pmr::monotonic_buffer_resource> _upstream{};
        std::unique_ptr<std::pmr::unsynchronized_pool_resource> _memory{};
        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        std::unique_ptr<graph_type> _graph{};
        std::unique_ptr<vertex_index_map_type> _vertex_index_map{};

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
            _graph = std::make_unique<graph_type>(*_allocator);
            _vertex_index_map = std::make_unique<vertex_index_map_type>(*_allocator);
        }

    };

    template<typename Graph, typename Types> requires
        (is_mmap_types<Types>)
    class graph_storage<Graph, Types> : public mmap_base<Types> {

    public:
        using graph_type = Graph;
        using types = Types;

        using base_type = mmap_base<types>;
        using mmap_container_type = typename base_type::mmap_container_type;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = typename graph::graph_traits<graph_type>::allocator_type;

        template<typename Type>
        using allocator_traits_type = typename graph::graph_traits<graph_type>::template allocator_traits_type<Type>;

        using vertex_descriptor = typename graph::graph_traits<graph_type>::vertex_descriptor;
        using vertex_index_map_type = typename types::template vector_type<vertex_descriptor,
            typename allocator_traits_type<allocator_type>::template rebind_alloc<vertex_descriptor>>;

        struct graph_wrapper {
            graph_type graph;
        };

        struct vertex_index_map_wrapper {
            vertex_index_map_type vertex_index_map;
        };

        explicit graph_storage(
                const std::string &directory = (std::filesystem::temp_directory_path() / "map_matching_2").string(),
                const std::string &graph_file = "graph.bin",
                const open_or_create_tag open_or_create = OPEN_READ_ONLY,
                const std::size_t graph_size = 1024 * 1024,
                const destroy_on_close_tag destroy_on_close = DO_NOT_DESTROY_ON_CLOSE,
                const shrink_on_close_tag shrink_on_close = DO_NOT_SHRINK_ON_CLOSE)
            : base_type{directory}, _container{
                    open_or_create == DESTROY_AND_CREATE
                    ? this->destroy_and_create(graph_file, graph_size)
                    : this->open_read_only(graph_file)
            }, _open_or_create{open_or_create}, _destroy_on_close{destroy_on_close}, _shrink_on_close{shrink_on_close} {
            _prepare_allocator();
            if (_open_or_create == DESTROY_AND_CREATE) {
                _create();
            } else {
                _open();
            }
        }

        constexpr graph_storage(const graph_storage &other) = delete;

        constexpr graph_storage(graph_storage &&other) noexcept
            : base_type{std::move(other)},
            _container{std::move(other._container)},
            _open_or_create{std::move(other._open_or_create)},
            _destroy_on_close{std::move(other._destroy_on_close)},
            _shrink_on_close{std::move(other._shrink_on_close)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _graph{std::move(other._graph)},
            _vertex_index_map{std::move(other._vertex_index_map)} {
            other._graph = nullptr;
            other._vertex_index_map = nullptr;
        }

        constexpr graph_storage &operator=(const graph_storage &other) = delete;

        constexpr graph_storage &operator=(graph_storage &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::move(other));
                _container = std::move(other._container);
                _open_or_create = std::move(other._open_or_create);
                _destroy_on_close = std::move(other._destroy_on_close);
                _shrink_on_close = std::move(other._shrink_on_close);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _graph = std::move(other._graph);
                _vertex_index_map = std::move(other._vertex_index_map);
                other._graph = nullptr;
                other._vertex_index_map = nullptr;
            }
            return *this;
        }

        ~graph_storage() {
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

            if (not _graph) {
                throw std::runtime_error("could not find graph.");
            }
            if (not _vertex_index_map) {
                throw std::runtime_error("could not find vertex_index_map.");
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

            if (not _graph) {
                throw std::runtime_error("could not find graph.");
            }
            if (not _vertex_index_map) {
                throw std::runtime_error("could not find vertex_index_map.");
            }
        }

        void shrink_to_fit(bool reopen = false) {
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

        [[nodiscard]] constexpr const graph_type &graph() const {
            return *_graph;
        }

        [[nodiscard]] constexpr graph_type &graph() {
            return *_graph;
        }

        [[nodiscard]] constexpr const vertex_index_map_type &vertex_index_map() const {
            return *_vertex_index_map;
        }

        [[nodiscard]] constexpr vertex_index_map_type &vertex_index_map() {
            return *_vertex_index_map;
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

        graph_type *_graph{nullptr};
        vertex_index_map_type *_vertex_index_map{nullptr};

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
            _graph = &_container.storage().template construct<graph_wrapper>
                    (boost::interprocess::unique_instance)
                    (graph_wrapper{graph_type{*_allocator}})->graph;
            _vertex_index_map = &_container.storage().template construct<vertex_index_map_wrapper>
                    (boost::interprocess::unique_instance)
                    (vertex_index_map_wrapper{vertex_index_map_type(*_allocator)})->vertex_index_map;
        }

        void _open() {
            _graph = &_container.storage().template find<graph_wrapper>
                    (boost::interprocess::unique_instance).first->graph;
            _vertex_index_map = &_container.storage().template find<vertex_index_map_wrapper>
                    (boost::interprocess::unique_instance).first->vertex_index_map;
        }

    };

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_GRAPH_STORAGE_HPP
