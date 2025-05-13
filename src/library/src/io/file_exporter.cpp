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

#include "io/file_exporter.hpp"

namespace map_matching_2::io {

    void file_exporter::open() {
        if (not filename().empty()) {
            _out = std::ofstream{filename(), std::ofstream::out | std::ofstream::trunc};
        }
    }

    void file_exporter::close() {
        if (_out.is_open()) {
            try {
                _out.close();
            } catch (std::exception &e) {}
        }
    }

    file_exporter::file_exporter(std::string filename)
        : exporter{std::move(filename)} {
        open();
    }

    file_exporter::~file_exporter() {
        close();
    }

    [[nodiscard]] bool file_exporter::is_writable() const {
        return _out.is_open() and _out.good() and not filename().empty();
    }

}
