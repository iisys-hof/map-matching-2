// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TAG_HELPER_HPP
#define MAP_MATCHING_2_TAG_HELPER_HPP

#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/unordered_map.hpp>

namespace map_matching_2::geometry::network {

    class tag_helper {

    private:
        using id_key_map_type = boost::bimap<boost::bimaps::unordered_set_of<std::uint64_t>,
                boost::bimaps::unordered_set_of<std::string>>;
        using id_value_map_type = boost::bimap<boost::bimaps::unordered_set_of<std::uint64_t>,
                boost::bimaps::unordered_set_of<std::string>>;
        using pair_type = std::pair<std::uint64_t, std::uint64_t>;
        using tags_map_type = boost::bimap<boost::bimaps::unordered_set_of<std::uint64_t>,
                boost::bimaps::unordered_set_of<pair_type>>;

        id_key_map_type _id_key_map;
        id_value_map_type _id_value_map;
        tags_map_type _tags_map;

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _id_key_map;
            ar & _id_value_map;
            ar & _tags_map;
        }

    public:
        std::uint64_t id(const std::string &key, const std::string &value) {
            std::uint64_t key_id, value_id, tag_id;

            auto key_search = _id_key_map.right.find(key);
            if (key_search == _id_key_map.right.end()) {
                key_id = _id_key_map.size();
                _id_key_map.insert(id_key_map_type::value_type(key_id, key));
            } else {
                key_id = key_search->get_left();
            }

            auto value_search = _id_value_map.right.find(value);
            if (value_search == _id_value_map.right.end()) {
                value_id = _id_value_map.size();
                _id_value_map.insert(id_value_map_type::value_type(value_id, value));
            } else {
                value_id = value_search->get_left();
            }

            std::pair tag_ids{key_id, value_id};
            auto tags_search = _tags_map.right.find(tag_ids);
            if (tags_search == _tags_map.right.end()) {
                tag_id = _tags_map.size();
                _tags_map.insert(tags_map_type::value_type(tag_id, tag_ids));
            } else {
                tag_id = tags_search->get_left();
            }

            return tag_id;
        }

        [[nodiscard]] std::pair<std::string, std::string> tag(std::uint64_t tag_id) const {
            auto tags_search = _tags_map.left.find(tag_id);
            if (tags_search == _tags_map.left.end()) {
                throw std::invalid_argument{"could not find tag with id " + std::to_string(tag_id)};
            }

            const auto &tag_ids = _tags_map.left.at(tag_id);
            const auto &key = _id_key_map.left.at(tag_ids.first);
            const auto &value = _id_value_map.left.at(tag_ids.second);
            return std::pair{key, value};
        }

    };

}

#endif //MAP_MATCHING_2_TAG_HELPER_HPP
