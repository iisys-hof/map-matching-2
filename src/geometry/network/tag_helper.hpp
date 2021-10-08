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

#include <unordered_map>

#include <boost/functional/hash.hpp>

namespace map_matching_2::geometry::network {

    class tag_helper {

    private:
        using pair_type = std::pair<std::uint64_t, std::uint64_t>;

        std::unordered_map<std::uint64_t, std::string> _id_key_map;
        std::unordered_map<std::string, std::uint64_t> _key_id_map;

        std::unordered_map<std::uint64_t, std::string> _id_value_map;
        std::unordered_map<std::string, std::uint64_t> _value_id_map;

        std::unordered_map<std::uint64_t, pair_type> _tags_map;
        std::unordered_map<pair_type, std::uint64_t, boost::hash<pair_type>> _ids_map;

    public:
        std::uint64_t id(const std::string &key, const std::string &value);

        std::pair<std::string, std::string> tag(const std::uint64_t tag_id) const;

    };

}

#endif //MAP_MATCHING_2_TAG_HELPER_HPP
