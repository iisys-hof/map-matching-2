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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_TAG_STORAGE_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_TAG_STORAGE_HPP

#include <format>
#include <memory>
#include <memory_resource>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/string.hpp>

#include <boost/unordered/unordered_flat_map.hpp>

#include <boost/interprocess/containers/string.hpp>

#include "io/allocator/concepts.hpp"

#include "base_storage.hpp"

namespace map_matching_2::io::memory_mapped::storage {

    template<typename StringA, typename StringB>
    struct tag_comparator;

    template<typename StringA, typename StringB> requires
        (std::same_as<StringA, StringB>)
    struct tag_comparator<StringA, StringB> {

        using is_transparent = void;

        using string_type_a = StringA;
        using string_type_b = StringB;

        constexpr bool operator()(const string_type_a &left, const string_type_b &right) const {
            return left == right;
        }

    };

    template<typename StringA, typename StringB> requires
        (not std::same_as<StringA, StringB>)
    struct tag_comparator<StringA, StringB> {

        using is_transparent = void;

        using string_type_a = StringA;
        using string_type_b = StringB;

        constexpr bool operator()(const string_type_a &left, const string_type_a &right) const {
            return left == right;
        }

        constexpr bool operator()(const string_type_b &left, const string_type_b &right) const {
            return left == right;
        }

        constexpr bool operator()(const string_type_a &left, const string_type_b &right) const {
            return left == right.c_str();
        }

        constexpr bool operator()(const string_type_b &left, const string_type_a &right) const {
            return left == right.c_str();
        }

    };

    template<typename StringA, typename StringB>
    struct tag_hash;

    template<typename StringA, typename StringB> requires
        (std::same_as<StringA, StringB>)
    struct tag_hash<StringA, StringB> {
        using is_transparent = void;

        using string_type_a = StringA;
        using string_type_b = StringB;

        constexpr std::size_t operator()(const string_type_a &str) const {
            return boost::hash_value(str);
        }
    };

    template<typename StringA, typename StringB> requires
        (not std::same_as<StringA, StringB>)
    struct tag_hash<StringA, StringB> {
        using is_transparent = void;

        using string_type_a = StringA;
        using string_type_b = StringB;

        constexpr std::size_t operator()(const string_type_a &str) const {
            return boost::hash_value(str);
        }

        constexpr std::size_t operator()(const string_type_b &str) const {
            return boost::hash_value(str);
        }
    };

    template<typename Types, typename Allocator>
    class tag_storage;

    template<typename Types, typename Allocator> requires
        (is_memory_types<Types>)
    class tag_storage<Types, Allocator> {

    public:
        using types = Types;

        using allocation_counter_type = typename types::allocation_counter_type;

        using allocator_type = Allocator;

        using string_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<char>;
        using string_type = std::basic_string<char, std::char_traits<char>, string_allocator_type>;

        using pair_type = std::pair<std::uint64_t, std::uint64_t>;

        using key_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<std::pair<const std::uint64_t, string_type>>;
        using value_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<std::pair<const string_type, std::uint64_t>>;
        using tags_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<std::pair<const std::uint64_t, pair_type>>;
        using ids_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<std::pair<const pair_type, std::uint64_t>>;

        using key_map_type = boost::unordered_flat_map<std::uint64_t, string_type,
            boost::hash<std::uint64_t>, std::equal_to<>, key_allocator_type>;
        using value_map_type = boost::unordered_flat_map<string_type, std::uint64_t,
            tag_hash<std::string, string_type>, tag_comparator<std::string, string_type>, value_allocator_type>;
        using tags_map_type = boost::unordered_flat_map<std::uint64_t, pair_type,
            boost::hash<std::uint64_t>, std::equal_to<>, tags_allocator_type>;
        using ids_map_type = boost::unordered_flat_map<pair_type, std::uint64_t,
            boost::hash<pair_type>, std::equal_to<>, ids_allocator_type>;

        tag_storage() {
            _prepare_allocator();
            _create();
        }

        constexpr tag_storage(const tag_storage &other) = delete;

        constexpr tag_storage(tag_storage &&other) noexcept
            : _upstream{std::move(other._upstream)}, _memory{std::move(other._memory)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _id_key_map{std::move(other._id_key_map)}, _key_id_map{std::move(other._key_id_map)},
            _id_value_map{std::move(other._id_value_map)}, _value_id_map{std::move(other._value_id_map)},
            _tags_map{std::move(other._tags_map)}, _ids_map{std::move(other._ids_map)} {}

        constexpr tag_storage &operator=(const tag_storage &other) = delete;

        constexpr tag_storage &operator=(tag_storage &&other) noexcept {
            if (this != &other) {
                _upstream = std::move(other._upstream);
                _memory = std::move(other._memory);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _id_key_map = std::move(other._id_key_map);
                _key_id_map = std::move(other._key_id_map);
                _id_value_map = std::move(other._id_value_map);
                _value_id_map = std::move(other._value_id_map);
                _tags_map = std::move(other._tags_map);
                _ids_map = std::move(other._ids_map);
            }
            return *this;
        }

        constexpr ~tag_storage() = default;

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

        [[nodiscard]] constexpr string_type string(const std::string &str) {
            return string_type(str.c_str(), *_allocator);
        }

        [[nodiscard]] constexpr const key_map_type &id_key_map() const {
            return *_id_key_map;
        }

        [[nodiscard]] constexpr key_map_type &id_key_map() {
            return *_id_key_map;
        }

        [[nodiscard]] constexpr const value_map_type &key_id_map() const {
            return *_key_id_map;
        }

        [[nodiscard]] constexpr value_map_type &key_id_map() {
            return *_key_id_map;
        }

        [[nodiscard]] constexpr const key_map_type &id_value_map() const {
            return *_id_value_map;
        }

        [[nodiscard]] constexpr key_map_type &id_value_map() {
            return *_id_value_map;
        }

        [[nodiscard]] constexpr const value_map_type &value_id_map() const {
            return *_value_id_map;
        }

        [[nodiscard]] constexpr value_map_type &value_id_map() {
            return *_value_id_map;
        }

        [[nodiscard]] constexpr const tags_map_type &tags_map() const {
            return *_tags_map;
        }

        [[nodiscard]] constexpr tags_map_type &tags_map() {
            return *_tags_map;
        }

        [[nodiscard]] constexpr const ids_map_type &ids_map() const {
            return *_ids_map;
        }

        [[nodiscard]] constexpr ids_map_type &ids_map() {
            return *_ids_map;
        }

        [[nodiscard]] constexpr bool is_finalized() const {
            return _finalized;
        }

        [[nodiscard]] std::size_t size() const {
            return _allocation_counter->bytes();
        }

        void finalize() {
            _key_id_map.reset();
            _value_id_map.reset();
            _ids_map.reset();
            _finalized = true;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & *_id_key_map;
            ar & *_key_id_map;
            ar & *_id_value_map;
            ar & *_value_id_map;
            ar & *_tags_map;
            ar & *_ids_map;
            ar & _finalized;
        }

    private:
        std::unique_ptr<std::pmr::monotonic_buffer_resource> _upstream{};
        std::unique_ptr<std::pmr::unsynchronized_pool_resource> _memory{};
        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        std::unique_ptr<key_map_type> _id_key_map{};
        std::unique_ptr<value_map_type> _key_id_map{};
        std::unique_ptr<key_map_type> _id_value_map{};
        std::unique_ptr<value_map_type> _value_id_map{};
        std::unique_ptr<tags_map_type> _tags_map{};
        std::unique_ptr<ids_map_type> _ids_map{};
        bool _finalized{false};

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
            _id_key_map = std::make_unique<key_map_type>(*_allocator);
            _key_id_map = std::make_unique<value_map_type>(*_allocator);
            _id_value_map = std::make_unique<key_map_type>(*_allocator);
            _value_id_map = std::make_unique<value_map_type>(*_allocator);
            _tags_map = std::make_unique<tags_map_type>(*_allocator);
            _ids_map = std::make_unique<ids_map_type>(*_allocator);
        }

    };

    template<typename Types, typename Allocator> requires
        (is_mmap_types<Types>)
    class tag_storage<Types, Allocator> : public mmap_base<Types> {

    public:
        using types = Types;
        using base_type = mmap_base<types>;
        using mmap_container_type = typename base_type::mmap_container_type;

        using allocator_type = Allocator;

        using allocation_counter_type = typename types::allocation_counter_type;

        using string_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<char>;
        using string_type = boost::interprocess::basic_string<char, std::char_traits<char>, string_allocator_type>;

        using pair_type = std::pair<std::uint64_t, std::uint64_t>;

        using key_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<std::pair<const std::uint64_t, string_type>>;
        using value_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<std::pair<const string_type, std::uint64_t>>;
        using tags_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<std::pair<const std::uint64_t, pair_type>>;
        using ids_allocator_type = typename types::template allocator_traits_type<
            allocator_type>::template rebind_alloc<std::pair<const pair_type, std::uint64_t>>;

        using key_map_type = boost::unordered_flat_map<std::uint64_t, string_type,
            boost::hash<std::uint64_t>, std::equal_to<>, key_allocator_type>;
        using value_map_type = boost::unordered_flat_map<string_type, std::uint64_t,
            tag_hash<std::string, string_type>, tag_comparator<std::string, string_type>, value_allocator_type>;
        using tags_map_type = boost::unordered_flat_map<std::uint64_t, pair_type,
            boost::hash<std::uint64_t>, std::equal_to<>, tags_allocator_type>;
        using ids_map_type = boost::unordered_flat_map<pair_type, std::uint64_t,
            boost::hash<pair_type>, std::equal_to<>, ids_allocator_type>;

        constexpr static const char *tag_id_key_identifier = "tag-id-key";
        constexpr static const char *tag_key_id_identifier = "tag-key-id";
        constexpr static const char *tag_id_value_identifier = "tag-id-value";
        constexpr static const char *tag_value_id_identifier = "tag-value-id";
        constexpr static const char *tag_tags_identifier = "tag-tags";
        constexpr static const char *tag_ids_identifier = "tag-ids";
        constexpr static const char *finalized_identifier = "finalized";

        explicit tag_storage(
                const std::string &directory = (std::filesystem::temp_directory_path() / "map_matching_2").string(),
                const std::string &tags_file = "tags.bin",
                const open_or_create_tag open_or_create = OPEN_READ_ONLY,
                const std::size_t tags_size = 1024 * 1024,
                const destroy_on_close_tag destroy_on_close = DO_NOT_DESTROY_ON_CLOSE,
                const shrink_on_close_tag shrink_on_close = DO_NOT_SHRINK_ON_CLOSE)
            : base_type{directory}, _container{
                    open_or_create == DESTROY_AND_CREATE
                    ? this->destroy_and_create(tags_file, tags_size)
                    : this->open_read_only(tags_file)
            }, _open_or_create{open_or_create}, _destroy_on_close{destroy_on_close}, _shrink_on_close{shrink_on_close} {
            _prepare_allocator();
            if (_open_or_create == DESTROY_AND_CREATE) {
                _create();
            } else {
                _open();
            }
        }

        constexpr tag_storage(const tag_storage &other) = delete;

        constexpr tag_storage(tag_storage &&other) noexcept
            : base_type{std::move(other)}, _container{std::move(other._container)},
            _open_or_create{std::move(other._open_or_create)},
            _destroy_on_close{std::move(other._destroy_on_close)},
            _shrink_on_close{std::move(other._shrink_on_close)},
            _allocation_counter{std::move(other._allocation_counter)}, _allocator{std::move(other._allocator)},
            _id_key_map{std::move(other._id_key_map)}, _key_id_map{std::move(other._key_id_map)},
            _id_value_map{std::move(other._id_value_map)}, _value_id_map{std::move(other._value_id_map)},
            _tags_map{std::move(other._tags_map)}, _ids_map{std::move(other._ids_map)},
            _finalized{std::move(other._finalized)} {
            other._id_key_map = nullptr;
            other._key_id_map = nullptr;
            other._id_value_map = nullptr;
            other._value_id_map = nullptr;
            other._tags_map = nullptr;
            other._ids_map = nullptr;
            other._finalized = nullptr;
        }

        constexpr tag_storage &operator=(const tag_storage &other) = delete;

        constexpr tag_storage &operator=(tag_storage &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::move(other));
                _container = std::move(other._container);
                _open_or_create = std::move(other._open_or_create);
                _destroy_on_close = std::move(other._destroy_on_close);
                _shrink_on_close = std::move(other._shrink_on_close);
                _allocation_counter = std::move(other._allocation_counter);
                _allocator = std::move(other._allocator);
                _id_key_map = std::move(other._id_key_map);
                _key_id_map = std::move(other._key_id_map);
                _id_value_map = std::move(other._id_value_map);
                _value_id_map = std::move(other._value_id_map);
                _tags_map = std::move(other._tags_map);
                _ids_map = std::move(other._ids_map);
                _finalized = std::move(other._finalized);
                other._id_key_map = nullptr;
                other._key_id_map = nullptr;
                other._id_value_map = nullptr;
                other._value_id_map = nullptr;
                other._tags_map = nullptr;
                other._ids_map = nullptr;
                other._finalized = nullptr;
            }
            return *this;
        }

        ~tag_storage() {
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

            if (not _id_key_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_id_key_identifier));
            }
            if (not _key_id_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_key_id_identifier));
            }
            if (not _id_value_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_id_value_identifier));
            }
            if (not _value_id_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_value_id_identifier));
            }
            if (not _tags_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_tags_identifier));
            }
            if (not _ids_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_ids_identifier));
            }
            if (not _finalized) {
                throw std::runtime_error(std::format("could not find {}.", finalized_identifier));
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

            if (not _id_key_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_id_key_identifier));
            }
            if (not _key_id_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_key_id_identifier));
            }
            if (not _id_value_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_id_value_identifier));
            }
            if (not _value_id_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_value_id_identifier));
            }
            if (not _tags_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_tags_identifier));
            }
            if (not _ids_map) {
                throw std::runtime_error(std::format("could not find {}.", tag_ids_identifier));
            }
            if (not _finalized) {
                throw std::runtime_error(std::format("could not find {}.", finalized_identifier));
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

        [[nodiscard]] constexpr string_type string(const std::string &str) {
            return string_type(str.c_str(), *_allocator);
        }

        [[nodiscard]] constexpr const key_map_type &id_key_map() const {
            return *_id_key_map;
        }

        [[nodiscard]] constexpr key_map_type &id_key_map() {
            return *_id_key_map;
        }

        [[nodiscard]] constexpr const value_map_type &key_id_map() const {
            return *_key_id_map;
        }

        [[nodiscard]] constexpr value_map_type &key_id_map() {
            return *_key_id_map;
        }

        [[nodiscard]] constexpr const key_map_type &id_value_map() const {
            return *_id_value_map;
        }

        [[nodiscard]] constexpr key_map_type &id_value_map() {
            return *_id_value_map;
        }

        [[nodiscard]] constexpr const value_map_type &value_id_map() const {
            return *_value_id_map;
        }

        [[nodiscard]] constexpr value_map_type &value_id_map() {
            return *_value_id_map;
        }

        [[nodiscard]] constexpr const tags_map_type &tags_map() const {
            return *_tags_map;
        }

        [[nodiscard]] constexpr tags_map_type &tags_map() {
            return *_tags_map;
        }

        [[nodiscard]] constexpr const ids_map_type &ids_map() const {
            return *_ids_map;
        }

        [[nodiscard]] constexpr ids_map_type &ids_map() {
            return *_ids_map;
        }

        [[nodiscard]] constexpr bool is_finalized() const {
            return *_finalized;
        }

        [[nodiscard]] std::size_t size() const {
            return _container.size();
        }

        void finalize() {
            _container.storage().template destroy<value_map_type>(tag_key_id_identifier);
            _container.storage().template destroy<value_map_type>(tag_value_id_identifier);
            _container.storage().template destroy<ids_map_type>(tag_ids_identifier);
            *_finalized = true;
        }

    private:
        mmap_container_type _container;
        open_or_create_tag _open_or_create{OPEN_READ_ONLY};
        destroy_on_close_tag _destroy_on_close{DO_NOT_DESTROY_ON_CLOSE};
        shrink_on_close_tag _shrink_on_close{DO_NOT_SHRINK_ON_CLOSE};

        std::unique_ptr<allocation_counter_type> _allocation_counter{};
        std::unique_ptr<allocator_type> _allocator{};

        key_map_type *_id_key_map{nullptr};
        value_map_type *_key_id_map{nullptr};
        key_map_type *_id_value_map{nullptr};
        value_map_type *_value_id_map{nullptr};
        tags_map_type *_tags_map{nullptr};
        ids_map_type *_ids_map{nullptr};
        bool *_finalized{nullptr};

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
            _id_key_map = _container.storage().template construct<key_map_type>
                    (tag_id_key_identifier)(key_map_type{*_allocator});
            _key_id_map = _container.storage().template construct<value_map_type>
                    (tag_key_id_identifier)(value_map_type{*_allocator});
            _id_value_map = _container.storage().template construct<key_map_type>
                    (tag_id_value_identifier)(key_map_type{*_allocator});
            _value_id_map = _container.storage().template construct<value_map_type>
                    (tag_value_id_identifier)(value_map_type{*_allocator});
            _tags_map = _container.storage().template construct<tags_map_type>
                    (tag_tags_identifier)(tags_map_type{*_allocator});
            _ids_map = _container.storage().template construct<ids_map_type>
                    (tag_ids_identifier)(ids_map_type{*_allocator});
            _finalized = _container.storage().template construct<bool>
                    (finalized_identifier)(false);
        }

        void _open() {
            _id_key_map = _container.storage().template find<key_map_type>(tag_id_key_identifier).first;
            _key_id_map = _container.storage().template find<value_map_type>(tag_key_id_identifier).first;
            _id_value_map = _container.storage().template find<key_map_type>(tag_id_value_identifier).first;
            _value_id_map = _container.storage().template find<value_map_type>(tag_value_id_identifier).first;
            _tags_map = _container.storage().template find<tags_map_type>(tag_tags_identifier).first;
            _ids_map = _container.storage().template find<ids_map_type>(tag_ids_identifier).first;
            _finalized = _container.storage().template find<bool>(finalized_identifier).first;
        }

    };

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_TAG_STORAGE_HPP
