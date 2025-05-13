// Copyright (C) 2022-2025 Adrian Wöltche
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

#include "app/global.hpp"

#include "util/macros.hpp"

#if defined(MM2_WINDOWS)
#include <windows.h>
#elif defined(MM2_LINUX)
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace map_matching_2::app {

    global_storage global;

    console_size get_console_size() {
#if defined(MM2_WINDOWS)
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
        std::size_t columns = csbi.srWindow.Right - csbi.srWindow.Left + 1;
        std::size_t rows = csbi.srWindow.Bottom - csbi.srWindow.Top + 1;
#elif defined(MM2_LINUX)
        struct winsize w;
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
        std::size_t columns = w.ws_col;
        std::size_t rows = w.ws_row;
#else
        std::size_t columns = 80;
        std::size_t rows = 24;
#endif
        if (columns < 80) {
            columns = 80;
        }
        if (rows < 24) {
            rows = 24;
        }

        return console_size{columns, rows};
    }

}
