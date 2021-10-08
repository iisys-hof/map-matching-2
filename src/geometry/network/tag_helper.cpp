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

#include "tag_helper.hpp"

namespace map_matching_2::geometry::network {

    std::uint64_t tag_helper::id(const std::string &key, const std::string &value) {
        std::uint64_t key_id, value_id, tag_id;

        if (not _key_id_map.contains(key)) {
            key_id = _key_id_map.size();
            _key_id_map.emplace(key, key_id);
            _id_key_map.emplace(key_id, key);
        } else {
            key_id = _key_id_map[key];
        }

        if (not _value_id_map.contains(value)) {
            value_id = _value_id_map.size();
            _value_id_map.emplace(value, value_id);
            _id_value_map.emplace(value_id, value);
        } else {
            value_id = _value_id_map[value];
        }

        std::pair tag_ids{key_id, value_id};
        if (not _ids_map.contains(tag_ids)) {
            tag_id = _ids_map.size();
            _ids_map.emplace(tag_ids, tag_id);
            _tags_map.emplace(tag_id, std::move(tag_ids));
        } else {
            tag_id = _ids_map[tag_ids];
        }

        return tag_id;
    }

    std::pair<std::string, std::string> tag_helper::tag(const std::uint64_t tag_id) const {
        if (not _tags_map.contains(tag_id)) {
            throw std::invalid_argument{"could not find tag with id " + std::to_string(tag_id)};
        }

        const auto &tag_ids = _tags_map.at(tag_id);
        const auto &key = _id_key_map.at(tag_ids.first);
        const auto &value = _id_value_map.at(tag_ids.second);
        return std::pair{key, value};
    }

}
