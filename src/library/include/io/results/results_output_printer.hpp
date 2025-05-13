// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_RESULTS_RESULTS_OUTPUT_PRINTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_RESULTS_OUTPUT_PRINTER_HPP

#include <string>
#include <format>
#include <iostream>

#include "util/coordinator.hpp"

namespace map_matching_2::io::results {

    template<typename OutputTask>
    class results_output_printer : public util::coordinator<OutputTask> {

    public:
        using task_type = OutputTask;
        using base_type = util::coordinator<OutputTask>;

        results_output_printer(const bool console, const bool verbose)
            : base_type{1, 1024}, _console{console}, _verbose{verbose} {}

        ~results_output_printer() override = default;

        [[nodiscard]] constexpr bool is_active() const {
            return _console or _verbose;
        }

        [[nodiscard]] constexpr bool is_console() const {
            return _console;
        }

    protected:
        const bool _console{false}, _verbose{false};
        std::size_t _counter{};
        double _duration{};

        void print_result(std::string &&result) const {
            if (is_active() and not result.empty()) {
                std::cout << result << std::endl;
            }
        }

    };

}

#endif //MAP_MATCHING_2_IO_RESULTS_RESULTS_OUTPUT_PRINTER_HPP
