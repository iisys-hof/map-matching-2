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

#include <osmium/tags/taglist.hpp>

#include "io/memory_mapped/concepts.hpp"
#include "io/memory_mapped/utils.hpp"

namespace map_matching_2::io::helper {

    class tag_helper_dummy {

    public:
        [[nodiscard]] const std::vector<std::uint64_t> &node_tags(const std::uint64_t id) const {
            return _empty;
        }

        [[nodiscard]] const std::vector<std::uint64_t> &edge_tags(const std::uint64_t id) const {
            return _empty;
        }

        [[nodiscard]] std::pair<std::string, std::string> tag(const std::uint64_t tag_id) const {
            return {};
        }

    private:
        std::vector<std::uint64_t> _empty{};

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
        using vector_map_type = typename tag_storage_type::vector_map_type;

        using key_type = typename key_map_type::mapped_type;
        using value_type = typename value_map_type::key_type;
        using tag_vector_type = typename vector_map_type::mapped_type;

        constexpr tag_helper(tag_storage_type tag_storage = {})
            : _tag_storage{std::move(tag_storage)} {}

        [[nodiscard]] constexpr const tag_storage_type &tag_storage() const {
            return _tag_storage;
        }

        [[nodiscard]] constexpr tag_storage_type &tag_storage() {
            return _tag_storage;
        }

        [[nodiscard]] std::uint64_t id(const std::string &key, const std::string &value, bool retry = true) {
            if (_tag_storage.is_finalized()) {
                throw std::runtime_error{"cannot add new tag, tag_storage is finalized"};
            }

            const auto &_id = [&]() -> std::uint64_t {
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
                        throw;
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
                        throw;
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
                        throw;
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
                if (retry) {
                    return memory_mapped::retry_alloc([&]() {
                                return _id();
                            }, [&](auto &ex) {
                                handle_bad_alloc(ex);
                            });
                } else {
                    return _id();
                }
            } else {
                return _id();
            }
        }

        void set_node_tags(const std::uint64_t id, tag_vector_type &&tags) {
            set_map_tags(id, std::move(tags), MAP_TAGS::NODES_MAP);
        }

        void set_node_tags(const std::uint64_t id, const osmium::TagList &tag_list) {
            set_map_tags(id, tag_list, MAP_TAGS::NODES_MAP);
        }

        template<typename TagsT>
        void copy_node_tags(const std::uint64_t id, const TagsT &tags) {
            copy_map_tags(id, tags, MAP_TAGS::NODES_MAP);
        }

        template<typename TagHelperT>
        void clone_node_tags(const std::uint64_t id, const TagHelperT &other) {
            clone_map_tags(id, other.node_tags(id), other, MAP_TAGS::NODES_MAP);
        }

        void set_edge_tags(const std::uint64_t id, tag_vector_type &&tags) {
            set_map_tags(id, std::move(tags), MAP_TAGS::EDGES_MAP);
        }

        void set_edge_tags(const std::uint64_t id, const osmium::TagList &tag_list) {
            set_map_tags(id, tag_list, MAP_TAGS::EDGES_MAP);
        }

        template<typename TagsT>
        void copy_edge_tags(const std::uint64_t id, const TagsT &tags) {
            copy_map_tags(id, tags, MAP_TAGS::EDGES_MAP);
        }

        template<typename TagHelperT>
        void clone_edge_tags(const std::uint64_t id, const TagHelperT &other) {
            clone_map_tags(id, other.edge_tags(id), other, MAP_TAGS::EDGES_MAP);
        }

        void merge_edge_tags(const std::uint64_t id, const std::uint64_t &other_id) {
            merge_map_tags(id, other_id, MAP_TAGS::EDGES_MAP);
        }

        [[nodiscard]] const tag_vector_type &node_tags(const std::uint64_t id) const {
            return map_tags(id, MAP_TAGS::NODES_MAP);
        }

        [[nodiscard]] const tag_vector_type &edge_tags(const std::uint64_t id) const {
            return map_tags(id, MAP_TAGS::EDGES_MAP);
        }

        [[nodiscard]] std::pair<std::string, std::string> tag(const std::uint64_t tag_id) const {
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

        enum MAP_TAGS {
            NODES_MAP, EDGES_MAP
        };

        void handle_bad_alloc(auto &ex) {
            if constexpr (memory_mapped::has_grow<tag_storage_type>) {
                _tag_storage.grow();
            } else {
                std::rethrow_exception(std::current_exception());
            }
        }

        [[nodiscard]] vector_map_type &get_map(MAP_TAGS map_tag) {
            if (map_tag == MAP_TAGS::NODES_MAP) {
                return _tag_storage.nodes_map();
            } else {
                return _tag_storage.edges_map();
            }
        }

        [[nodiscard]] const vector_map_type &get_map(MAP_TAGS map_tag) const {
            if (map_tag == MAP_TAGS::NODES_MAP) {
                return _tag_storage.nodes_map();
            } else {
                return _tag_storage.edges_map();
            }
        }

        [[nodiscard]] tag_vector_type tags(const osmium::TagList &tag_list) {
            const auto &_tags = [&]() -> tag_vector_type {
                tag_vector_type tags_container = _tag_storage.tags();
                tags_container.reserve(tag_list.size());
                for (const auto &tag : tag_list) {
                    tags_container.emplace_back(id(tag.key(), tag.value(), false));
                }
                return tags_container;
            };

            if constexpr (is_mmap_tag_helper<tag_helper>) {
                return memory_mapped::retry_alloc([&]()-> tag_vector_type {
                            return _tags();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else {
                return _tags();
            }
        }

        template<typename TagsT>
        [[nodiscard]] tag_vector_type copy_tags(const TagsT &tags) {
            const auto &_copy_tags = [&]() -> tag_vector_type {
                tag_vector_type tags_container = _tag_storage.tags();
                tags_container.reserve(tags.size());
                std::copy(std::cbegin(tags), std::cend(tags), std::back_inserter(tags_container));
                return tags_container;
            };

            if constexpr (is_mmap_tag_helper<tag_helper>) {
                return memory_mapped::retry_alloc([&]()-> tag_vector_type {
                            return _copy_tags();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else {
                return _copy_tags();
            }
        }

        template<typename TagsT, typename TagHelperT>
        [[nodiscard]] tag_vector_type clone_tags(const TagsT &tags, const TagHelperT &other) {
            const auto &_clone_tags = [&]() -> tag_vector_type {
                tag_vector_type tags_container = _tag_storage.tags();
                tags_container.reserve(tags.size());
                for (const auto &tag_id : tags) {
                    // clones tag data from other tag_storage to this tag_storage
                    auto tag = other.tag(tag_id);
                    tags_container.emplace_back(id(tag.first, tag.second, false));
                }
                return tags_container;
            };

            if constexpr (is_mmap_tag_helper<tag_helper>) {
                return memory_mapped::retry_alloc([&]()-> tag_vector_type {
                            return _clone_tags();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else {
                return _clone_tags();
            }
        }

        void set_map_tags(const std::uint64_t id, tag_vector_type &&tags, MAP_TAGS map_tag) {
            const auto &_set_map_tags = [&]() {
                auto &map = get_map(map_tag);
                map.emplace(id, std::move(tags));
            };

            if (not tags.empty()) {
                if constexpr (is_mmap_tag_helper<tag_helper>) {
                    memory_mapped::retry_alloc([&]() {
                                _set_map_tags();
                            }, [&](auto &ex) {
                                handle_bad_alloc(ex);
                            });
                } else {
                    _set_map_tags();
                }
            }
        }

        void set_map_tags(const std::uint64_t id, const osmium::TagList &tag_list, MAP_TAGS map_tag) {
            if (not tag_list.empty()) {
                set_map_tags(id, tags(tag_list), map_tag);
            }
        }

        template<typename TagsT>
        void copy_map_tags(const std::uint64_t id, const TagsT &tags, MAP_TAGS map_tag) {
            const auto &_copy_map_tags = [&]() {
                if (not tags.empty()) {
                    auto &map = get_map(map_tag);
                    map.emplace(id, copy_tags(tags));
                }
            };

            if constexpr (is_mmap_tag_helper<tag_helper>) {
                memory_mapped::retry_alloc([&]() {
                            _copy_map_tags();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else {
                _copy_map_tags();
            }
        }

        template<typename TagT, typename TagHelperT>
        void clone_map_tags(const std::uint64_t id, const TagT &tags, const TagHelperT &other, MAP_TAGS map_tag) {
            const auto &_clone_map_tags = [&]() {
                if (not tags.empty()) {
                    auto &map = get_map(map_tag);
                    map.emplace(id, clone_tags(tags, other));
                }
            };

            if constexpr (is_mmap_tag_helper<tag_helper>) {
                memory_mapped::retry_alloc([&]() {
                            _clone_map_tags();
                        }, [&](auto &ex) {
                            handle_bad_alloc(ex);
                        });
            } else {
                _clone_map_tags();
            }
        }

        void merge_map_tags(const std::uint64_t id, const std::uint64_t &other_id, MAP_TAGS map_tag) {
            if (id != other_id) {
                const auto &_merge_map_tags = [&]() {
                    auto &map = get_map(map_tag);
                    using tag_type = typename tag_vector_type::value_type;

                    auto tags_search = map.find(id);
                    bool has_tags = tags_search != map.end();

                    auto other_tags_search = map.find(other_id);
                    bool has_other_tags = other_tags_search != map.end();

                    if (has_other_tags) {
                        if (has_tags) {
                            auto &tags = tags_search->second;
                            const auto &other_tags = other_tags_search->second;

                            std::vector<tag_type> _missing_tags;
                            _missing_tags.reserve(other_tags.size());
                            std::remove_copy_if(std::cbegin(other_tags), std::cend(other_tags),
                                    std::back_inserter(_missing_tags), [&tags](tag_type tag) {
                                        return std::find(std::cbegin(tags), std::cend(tags), tag) != std::cend(tags);
                                    });
                            tags.reserve(tags.size() + _missing_tags.size());
                            std::move(std::begin(_missing_tags), std::end(_missing_tags), std::back_inserter(tags));
                        } else {
                            const auto &other_tags = other_tags_search->second;
                            map.emplace(id, copy_tags(other_tags));
                        }
                    }
                };

                if constexpr (is_mmap_tag_helper<tag_helper>) {
                    memory_mapped::retry_alloc([&]() {
                                _merge_map_tags();
                            }, [&](auto &ex) {
                                handle_bad_alloc(ex);
                            });
                } else {
                    _merge_map_tags();
                }
            }
        }

        [[nodiscard]] const tag_vector_type &map_tags(const std::uint64_t id, MAP_TAGS map_tag) const {
            const auto &map = get_map(map_tag);
            const auto tags_search = map.find(id);
            if (tags_search == map.end()) {
                return _tag_storage.empty_tags();
            }
            return tags_search->second;
        }

    };

}

#endif //MAP_MATCHING_2_IO_HELPER_TAG_HELPER_HPP
