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

#ifndef MAP_MATCHING_2_IO_CSV_IMPORTER_HPP
#define MAP_MATCHING_2_IO_CSV_IMPORTER_HPP

#include <vector>

#include <csv.hpp>

#include "importer.hpp"

namespace map_matching_2::io {

    class csv_importer : public importer {

    public:
        constexpr csv_importer(std::string filename, std::size_t skip_lines)
            : importer{std::move(filename)}, _skip_lines{skip_lines} {}

        constexpr csv_importer(std::vector<std::string> filenames, std::size_t skip_lines)
            : importer{std::move(filenames)}, _skip_lines{skip_lines} {}

        void read() override;

        [[nodiscard]] constexpr const std::vector<std::string> &columns() const {
            return _columns;
        }

    protected:
        virtual void configure_format(csv::CSVFormat &format);

        virtual void process_row(std::size_t file_num, std::size_t row_num, const csv::CSVRow &row) = 0;

        virtual void finish_import();

    private:
        std::size_t _skip_lines;
        std::vector<std::string> _columns;

        void _read_csv(csv::CSVReader &csv, std::size_t file_num);

    };

}

#endif //MAP_MATCHING_2_IO_CSV_IMPORTER_HPP
