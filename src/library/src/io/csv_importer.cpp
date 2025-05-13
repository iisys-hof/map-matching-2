// Copyright (C) 2020-2025 Adrian Wöltche
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

#include "io/csv_importer.hpp"

#include <filesystem>
#include <format>

namespace map_matching_2::io {

    void csv_importer::configure_format(csv::CSVFormat &format) {}

    void csv_importer::finish_import() {}

    void csv_importer::read() {
        csv::CSVFormat format;
        configure_format(format);

        const auto &filenames = this->filenames();
        for (std::size_t i = 0; i < filenames.size(); ++i) {
            const auto &filename = filenames[i];

            if (not std::filesystem::exists(filename)) {
                std::cerr << std::format("file '{}' does not exist, skipping ...", filename) << std::endl;
                continue;
            }

            csv::CSVReader csv{filename, format};
            _read_csv(csv, i);
        }

        finish_import();
    }

    void csv_importer::_read_csv(csv::CSVReader &csv, const std::size_t file_num) {
        _columns = csv.get_col_names();

        std::size_t l = 0, r = 0;
        for (const auto &row : csv) {
            if (l >= _skip_lines) {
                process_row(file_num, r++, row);
            }
            l++;
        }
    }

}
