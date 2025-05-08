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

#ifndef MAP_MATCHING_2_IO_IMPORTER_HPP
#define MAP_MATCHING_2_IO_IMPORTER_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace map_matching_2::io {

    class importer {

    public:
        explicit constexpr importer(std::string filename)
            : importer{std::vector<std::string>{std::move(filename)}} {}

        explicit constexpr importer(std::vector<std::string> filenames)
            : _filenames{std::move(filenames)} {}

        virtual constexpr ~importer() = default;

        virtual void read() = 0;

        [[nodiscard]] constexpr const std::vector<std::string> &filenames() const {
            return _filenames;
        }

    protected:
        [[nodiscard]] static std::vector<char> parse_delimiter(const std::string &delimiter);

    private:
        const std::vector<std::string> _filenames;

    };

}

#endif //MAP_MATCHING_2_IO_IMPORTER_HPP
