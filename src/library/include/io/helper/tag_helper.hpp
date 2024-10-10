// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_HELPER_TAG_HELPER_HPP
#define MAP_MATCHING_2_IO_HELPER_TAG_HELPER_HPP

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/utility.hpp>

#include "io/memory_mapped/concepts.hpp"
#include "io/memory_mapped/utils.hpp"

namespace map_matching_2::io::helper {

    struct tag_helper_dummy {

        [[nodiscard]] std::pair<std::string, std::string> tag(std::uint64_t tag_id) const {
            return {};
        }

    };

    template<typename TagHelper>
    concept is_memory_tag_helper = io::memory_mapped::is_memory_types<typename TagHelper::tag_storage_types>;

    template<typename TagHelper>
    concept is_mmap_tag_helper = io::memory_mapped::is_mmap_types<typename TagHelper::tag_storage_types>;

    template<typename TagStorage>
    class tag_helper {

    public:
        using tag_storage_type = TagStorage;
        using tag_storage_types = typename tag_storage_type::types;

        using key_map_type = typename tag_storage_type::key_map_type;
        using value_map_type = typename tag_storage_type::value_map_type;
        using pair_type = typename tag_storage_type::pair_type;
        using tags_map_type = typename tag_storage_type::tags_map_type;
        using ids_map_type = typename tag_storage_type::ids_map_type;

        using key_type = typename key_map_type::mapped_type;
        using value_type = typename value_map_type::key_type;

        constexpr tag_helper(tag_storage_type tag_storage = {})
            : _tag_storage{std::move(tag_storage)} {}

        [[nodiscard]] constexpr const tag_storage_type &tag_storage() const {
            return _tag_storage;
        }

        [[nodiscard]] constexpr tag_storage_type &tag_storage() {
            return _tag_storage;
        }

        [[nodiscard]] std::uint64_t id(const std::string &key, const std::string &value) {
            if (_tag_storage.is_finalized()) {
                throw std::runtime_error{"cannot add new tag, tag_storage is finalized"};
            }

            const auto &_id = [this, &key, &value]() -> std::uint64_t {
                // key_type _key = _tag_storage.string(key);
                // value_type _value = _tag_storage.string(value);

                std::uint64_t key_id, value_id, tag_id;

                auto key_search = _tag_storage.key_id_map().find(key);
                if (key_search == _tag_storage.key_id_map().end()) {
                    key_id = _tag_storage.key_id_map().size();
                    try {
                        _tag_storage.key_id_map().emplace(_tag_storage.string(key), key_id);
                        _tag_storage.id_key_map().emplace(key_id, _tag_storage.string(key));
                    } catch (...) {
                        _tag_storage.key_id_map().erase(key);
                        _tag_storage.id_key_map().erase(key_id);
                        std::rethrow_exception(std::current_exception());
                    }
                } else {
                    key_id = key_search->second;
                }

                auto value_search = _tag_storage.value_id_map().find(value);
                if (value_search == _tag_storage.value_id_map().end()) {
                    value_id = _tag_storage.value_id_map().size();
                    try {
                        _tag_storage.value_id_map().emplace(_tag_storage.string(value), value_id);
                        _tag_storage.id_value_map().emplace(value_id, _tag_storage.string(value));
                    } catch (...) {
                        _tag_storage.value_id_map().erase(value);
                        _tag_storage.id_value_map().erase(value_id);
                        std::rethrow_exception(std::current_exception());
                    }
                } else {
                    value_id = value_search->second;
                }

                std::pair tag_ids{key_id, value_id};
                auto tag_search = _tag_storage.ids_map().find(tag_ids);
                if (tag_search == _tag_storage.ids_map().end()) {
                    tag_id = _tag_storage.ids_map().size();
                    try {
                        _tag_storage.ids_map().emplace(tag_ids, tag_id);
                        _tag_storage.tags_map().emplace(tag_id, tag_ids);
                    } catch (...) {
                        _tag_storage.ids_map().erase(tag_ids);
                        _tag_storage.tags_map().erase(tag_id);
                        std::rethrow_exception(std::current_exception());
                    }
                } else {
                    tag_id = tag_search->second;
                }

                return tag_id;
            };

            if constexpr (is_mmap_tag_helper<tag_helper>) {
                static_assert(not std::is_same_v<key_type, std::string> or
                        not std::is_same_v<value_type, std::string>,
                        "mmap key_type and value_type cannot be std::string, does not work with mmap");
                return memory_mapped::retry_alloc([&]() {
                            return _id();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else {
                return _id();
            }
        }

        [[nodiscard]] std::pair<std::string, std::string> tag(std::uint64_t tag_id) const {
            auto tags_search = _tag_storage.tags_map().find(tag_id);
            if (tags_search == _tag_storage.tags_map().end()) {
                throw std::invalid_argument{"could not find tag with id " + std::to_string(tag_id)};
            }

            const auto &tag_ids = tags_search->second;
            const auto &key = _tag_storage.id_key_map().at(tag_ids.first);
            const auto &value = _tag_storage.id_value_map().at(tag_ids.second);
            return std::pair<std::string, std::string>{key, value};
        }

        [[nodiscard]] std::size_t size() const {
            return _tag_storage.size();
        }

        void finalize() {
            _tag_storage.finalize();
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) requires
            (is_memory_tag_helper<tag_helper>) {
            ar & _tag_storage;
        }

    private:
        tag_storage_type _tag_storage;

        void handle_bad_alloc(auto &ex) {
            if constexpr (memory_mapped::has_grow<tag_storage_type>) {
                _tag_storage.grow();
            } else {
                std::rethrow_exception(std::current_exception());
            }
        }

    };

}

#endif //MAP_MATCHING_2_IO_HELPER_TAG_HELPER_HPP
