// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_FILE_EXPORTER_HPP
#define MAP_MATCHING_2_IO_FILE_EXPORTER_HPP

#include <fstream>

#include "exporter.hpp"

namespace map_matching_2::io {

    class file_exporter : public exporter {

    public:
        explicit file_exporter(std::string filename);

        constexpr file_exporter() = delete;

        constexpr file_exporter(const file_exporter &other) = delete;

        file_exporter(file_exporter &&other) noexcept = default;

        constexpr file_exporter &operator=(const file_exporter &other) = delete;

        file_exporter &operator=(file_exporter &&other) noexcept = default;

        ~file_exporter() override;

        [[nodiscard]] bool is_writable() const;

    protected:
        void open();

        [[nodiscard]] constexpr std::ofstream &out() {
            return _out;
        }

        void close();

    private:
        std::ofstream _out;

    };

}

#endif //MAP_MATCHING_2_IO_FILE_EXPORTER_HPP
