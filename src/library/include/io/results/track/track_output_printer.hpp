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

#ifndef MAP_MATCHING_2_IO_RESULTS_TRACK_TRACK_OUTPUT_PRINTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_TRACK_TRACK_OUTPUT_PRINTER_HPP

#include "../results_output_printer.hpp"

namespace map_matching_2::io::results {

    struct track_output_task {
        std::string id, track;
        std::size_t track_points;
    };

    class track_output_printer : public results_output_printer<track_output_task> {

    public:
        using task_type = track_output_task;
        using base_type = results_output_printer<task_type>;

        track_output_printer(const bool console, const bool verbose)
            : base_type{console, verbose} {}

        ~track_output_printer() override {
            this->print_result(parse_aggregated_result());
        }

        template<typename MultiTrack>
        [[nodiscard]] task_type parse_result(const MultiTrack &multi_track) const {
            std::string wkt{};
            std::size_t sizes{0};

            if (this->is_active()) {
                if (this->is_console()) {
                    wkt = multi_track.wkt();
                } else {
                    sizes = multi_track.multi_rich_line.sizes();
                }
            }

            return {multi_track.id, std::move(wkt), sizes};
        }

        void pass(task_type &&task) {
            this->queue(std::move(task));
        }

    protected:
        std::size_t _track_points{};

        [[nodiscard]] std::string parse_task(task_type &&task) {
            if (this->is_active()) {
                if (this->is_console()) {
                    return task.track;
                } else {
                    _track_points += task.track_points;

                    return std::format(
                            "Track {} with id '{}' done, track points: {}",
                            ++this->_counter, task.id, task.track_points);
                }
            }
            return "";
        }

        [[nodiscard]] std::string parse_aggregated_result() {
            if (this->is_active()) {
                if (this->_counter > 0) {
                    return std::format(
                            "Tracks count: {}, tracks points: {}",
                            this->_counter, _track_points);
                }
            }
            return "";
        }

        void execute(task_type &&task) override {
            this->print_result(parse_task(std::move(task)));
        }

    };

}

#endif //MAP_MATCHING_2_IO_RESULTS_TRACK_TRACK_OUTPUT_PRINTER_HPP
