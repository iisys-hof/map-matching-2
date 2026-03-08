// Copyright (C) 2026 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_UTIL_STRING_HPP
#define MAP_MATCHING_2_UTIL_STRING_HPP

#include <string_view>
#include <charconv>
#include <stdexcept>
#include <format>

#include <boost/type_index.hpp>

#include "concepts.hpp"

namespace map_matching_2::util {

    template<numeric T>
    [[nodiscard]] T parse_number(const std::string_view str) {
        using type = std::remove_cvref_t<T>;

        type value{};
        auto [ptr, ec] = std::from_chars(str.data(), str.data() + str.size(), value);
        if (ec == std::errc()) {
            if (ptr == str.data() + str.size()) {
                return value;
            } else {
                throw std::invalid_argument{std::format("string contains more than only a number: {}", str)};
            }
        } else if (ec == std::errc::invalid_argument) {
            throw std::invalid_argument{std::format("string is not a number: {}", str)};
        } else if (ec == std::errc::result_out_of_range) {
            throw std::out_of_range{
                    std::format("number is larger than can be stored in given type {}: {}",
                            boost::typeindex::type_id<type>().pretty_name(), str)
            };
        }

        auto err = std::make_error_condition(ec);
        throw std::runtime_error{
                std::format("unknown error {} -> '{}' while parsing string: {}", err.value(), err.message(), str)
        };
    }

    [[nodiscard]] bool parse_bool(std::string_view str);

}

#endif //MAP_MATCHING_2_UTIL_STRING_HPP
