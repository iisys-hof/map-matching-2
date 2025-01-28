// Copyright (C) 2022-2024 Adrian Wöltche
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

#ifdef MM2_ENABLE_RPMALLOC
#include <rpmalloc.h>
#endif

#include "app/mode.hpp"

int main(int argc, char *argv[]) {
#ifdef MM2_ENABLE_RPMALLOC
    rpmalloc_linker_reference();
#endif
    return map_matching_2::app::mode(argc, argv);
}
