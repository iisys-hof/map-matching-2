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

#ifndef MAP_MATCHING_2_IO_RESULTS_MATCH_MATCH_OUTPUT_PRINTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_MATCH_MATCH_OUTPUT_PRINTER_HPP

#include "../results_output_printer.hpp"

namespace map_matching_2::io::results {

    struct match_output_task {
        std::string id, match;
        std::size_t track_points, prepared_points, edges, combinations;
        bool aborted;
        double duration;
    };

    class match_output_printer : public results_output_printer<match_output_task> {

    public:
        using task_type = match_output_task;
        using base_type = results_output_printer<task_type>;

        match_output_printer(const bool console, const bool verbose)
            : base_type{console, verbose} {}

        ~match_output_printer() override {
            this->print_result(parse_aggregated_result());
        }

        template<typename MatchResult>
        [[nodiscard]] task_type parse_result(const MatchResult &match_result) const {
            std::size_t track_sizes{0}, prepared_sizes{0}, edges{0}, combinations{0};
            std::string wkt{};

            if (this->is_active()) {
                if (this->is_console()) {
                    wkt = match_result.match.wkt();
                } else {
                    track_sizes = match_result.track.multi_rich_line.sizes();
                    prepared_sizes = match_result.prepared.multi_rich_line.sizes();
                    for (std::size_t i = 0; i < match_result.environments.size(); ++i) {
                        edges += match_result.environments[i].edges();
                        combinations += match_result.environments[i].states();
                    }
                }
            }

            return {
                    match_result.track.id, std::move(wkt),
                    track_sizes, prepared_sizes, edges, combinations,
                    match_result.aborted, match_result.duration
            };
        }

        void pass(task_type &&match_result) {
            this->queue(std::move(match_result));
        }

    protected:
        std::size_t _aborted{}, _track_points{}, _prepared_points{}, _candidates{}, _combinations{};

        [[nodiscard]] std::string parse_task(task_type &&task) {
            if (this->is_active()) {
                if (this->is_console()) {
                    return task.match;
                } else {
                    this->_duration += task.duration;
                    _track_points += task.track_points;
                    _prepared_points += task.prepared_points;
                    _candidates += task.edges;
                    _combinations += task.combinations;

                    if (task.aborted) {
                        ++_aborted;
                    }

                    return std::format(
                            "Match {} with id '{}' {} {:.3f} s, "
                            "track points: {}, prepared points: {}, candidates: {}, combinations: {}",
                            ++this->_counter, task.id, task.aborted ? "aborted after" : "done in", task.duration,
                            task.track_points, task.prepared_points, task.edges, task.combinations);
                }
            }
            return "";
        }

        [[nodiscard]] std::string parse_aggregated_result() {
            if (this->is_active()) {
                if (this->_counter > 0) {
                    _aborted = _aborted > this->_counter ? this->_counter : _aborted;
                    return std::format(
                            "Matches count: {}, finished: {}, aborted: {}, total duration: {:.3f} s, "
                            "tracks points: {}, prepared points: {}, candidates: {}, combinations: {}",
                            this->_counter, this->_counter - _aborted, _aborted, this->_duration,
                            _track_points, _prepared_points, _candidates, _combinations);
                }
            }
            return "";
        }

        void execute(task_type &&task) override {
            this->print_result(parse_task(std::move(task)));
        }

    };

}

#endif //MAP_MATCHING_2_IO_RESULTS_MATCH_MATCH_OUTPUT_PRINTER_HPP
