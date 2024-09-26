// Copyright (C) 2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_RESULTS_CSV_RESULTS_EXPORTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_CSV_RESULTS_EXPORTER_HPP

#include <vector>

#include <boost/algorithm/string.hpp>

#include "util/coordinator.hpp"

#include "../csv_exporter.hpp"

namespace map_matching_2::io::results {

    template<typename CSVTask, char Delimiter = ',', char Quote = '"', bool Flush = false>
    class csv_results_exporter : public util::coordinator<CSVTask>, public csv_exporter<Delimiter, Quote, Flush> {

    public:
        using task_type = CSVTask;
        using coordinator_type = util::coordinator<CSVTask>;
        using base_type = csv_exporter<Delimiter, Quote, Flush>;

        explicit csv_results_exporter(std::string filename, const std::string &columns)
            : coordinator_type{1, 1024}, base_type{std::move(filename)} {
            boost::split(_column_strings, columns, boost::is_any_of(","));
        }

        ~csv_results_exporter() override = default;

        [[nodiscard]] bool is_active() const {
            return this->is_writable();
        }

    protected:
        bool _header_written{false};
        std::vector<std::string> _column_strings{};

        void write_header() {
            if (is_active()) {
                this->operator<<(_column_strings);
            }
        }

        void write_result(std::vector<std::string> &&result) {
            if (not _header_written) [[unlikely]] {
                write_header();
                _header_written = true;
            }
            if (is_active()) [[likely]] {
                this->operator<<(result);
            }
        }

    };

}

#endif //MAP_MATCHING_2_IO_RESULTS_CSV_RESULTS_EXPORTER_HPP
