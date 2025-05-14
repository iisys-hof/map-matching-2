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

#include "compare/comparator/compare_task.hpp"

namespace map_matching_2::compare {

    compare_task::compare_task(import_multi_track_variant_type match, import_multi_track_variant_type ground_truth,
            compare::settings compare_settings)
        : match{std::move(match)}, ground_truth{std::move(ground_truth)},
        compare_settings{std::move(compare_settings)} {}

}
