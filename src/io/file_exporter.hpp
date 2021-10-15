// Copyright (C) 2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_FILE_EXPORTER_HPP
#define MAP_MATCHING_2_FILE_EXPORTER_HPP

#include "exporter.hpp"

#include <fstream>

namespace map_matching_2::io {

    class file_exporter : public exporter {

    private:
        std::ofstream _out;

    protected:
        void open();

        [[nodiscard]] std::ofstream &out();

        void close();

    public:
        explicit file_exporter(std::string filename);

        file_exporter() = default;

        file_exporter(const file_exporter &other) = delete;

        file_exporter(file_exporter &&other) noexcept = default;

        file_exporter &operator=(const file_exporter &other) = delete;

        file_exporter &operator=(file_exporter &&other) noexcept = default;

        ~file_exporter();

        [[nodiscard]] bool is_writable() const;

    };

}

#endif //MAP_MATCHING_2_FILE_EXPORTER_HPP
