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

#include "exporter.hpp"

namespace map_matching_2::io {

    void exporter::open() {
        if (not _filename.empty()) {
            _out = std::ofstream{_filename, std::ofstream::out | std::ofstream::trunc};
        }
    }

    std::ofstream &exporter::out() {
        return _out;
    }

    void exporter::close() {
        if (_out.is_open()) {
            try {
                _out.close();
            } catch (std::exception &e) {}
        }
    }

    exporter::exporter(std::string filename) : _filename{std::move(filename)} {
        open();
    }

    exporter::~exporter() {
        close();
    }

    bool exporter::is_writable() const {
        return _out.is_open() and not _filename.empty();
    }

    const std::string &exporter::filename() const {
        return _filename;
    }

}
