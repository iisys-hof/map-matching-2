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
        void open() {
            if (not filename().empty()) {
                _out = std::ofstream{filename(), std::ofstream::out | std::ofstream::trunc};
            }
        }

        [[nodiscard]] std::ofstream &out() {
            return _out;
        }

        void close() {
            if (_out.is_open()) {
                try {
                    _out.close();
                } catch (std::exception &e) {}
            }
        }

    public:
        explicit file_exporter(std::string filename)
                : exporter{std::move(filename)} {
            open();
        }

        file_exporter() = delete;

        file_exporter(const file_exporter &other) = delete;

        file_exporter(file_exporter &&other) noexcept = default;

        file_exporter &operator=(const file_exporter &other) = delete;

        file_exporter &operator=(file_exporter &&other) noexcept = default;

        ~file_exporter() {
            close();
        }

        [[nodiscard]] bool is_writable() const {
            return _out.is_open() and _out.good() and not filename().empty();
        }

    };

}

#endif //MAP_MATCHING_2_FILE_EXPORTER_HPP
