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

#ifndef MAP_MATCHING_2_APP_COMPARE_HPP
#define MAP_MATCHING_2_APP_COMPARE_HPP

#include <memory>

#include "global.hpp"
#include "options.hpp"

#include "compare/comparator.hpp"

namespace map_matching_2::app {

    [[nodiscard]] std::unique_ptr<compare::comparator> _start_comparator(const compare_data &data);

    void _join_comparator(std::unique_ptr<compare::comparator> &comparator);

    void _read_compares(std::unique_ptr<compare::comparator> &comparator, const compare_data &data);

    void _start_comparison(const compare_data &data);

    void _compare(const compare_data &data);

    int compare(int argc, char *argv[]);

}

#endif //MAP_MATCHING_2_APP_COMPARE_HPP
