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

#ifndef MAP_MATCHING_2_IO_RESULTS_COMPARE_CSV_COMPARE_EXPORTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_COMPARE_CSV_COMPARE_EXPORTER_HPP

#include "../csv_results_exporter.hpp"

#include "geometry/common.hpp"
#include "geometry/algorithm/wkt.hpp"

namespace map_matching_2::io::results {

    struct compare_csv_task {
        std::string id, match, ground_truth, corrects, error_adds, error_misses;
        double duration, match_length, ground_truth_length,
                correct_fraction, error_fraction, correct, error_added, error_missed;
    };

    template<char Delimiter = ',', char Quote = '"', bool Flush = false>
    class csv_compare_exporter : public csv_results_exporter<compare_csv_task, Delimiter, Quote, Flush> {

    public:
        using task_type = compare_csv_task;
        using base_type = csv_results_exporter<task_type, Delimiter, Quote, Flush>;

        explicit csv_compare_exporter(std::string filename, const std::string &columns)
            : base_type{std::move(filename), columns} {
            _parse_columns();
        }

        ~csv_compare_exporter() override = default;

        template<typename CompareResult>
        [[nodiscard]] task_type parse_result(const CompareResult &compare_result) const {
            using point_type = typename CompareResult::multi_track_type::point_type;
            using length_type = typename CompareResult::multi_track_type::length_type;
            using line_type = typename geometry::models<point_type>::template line_type<>;
            using multi_line_type = typename geometry::models<point_type>::template multi_line_type<line_type>;

            const auto length_zero = geometry::default_float_type<length_type>::v0;

            multi_line_type corrects, error_adds, error_misses;
            length_type match_length{length_zero}, ground_truth_length{length_zero};
            std::string match_wkt{}, ground_truth_wkt{}, corrects_wkt{}, error_adds_wkt{}, error_misses_wkt{};

            if (this->is_active()) {
                for (const auto &_column : _columns) {
                    switch (_column) {
                        case column::MATCH:
                            match_wkt = compare_result.match.wkt();
                            break;
                        case column::GROUND_TRUTH:
                            ground_truth_wkt = compare_result.ground_truth.wkt();
                            break;
                        case column::MATCH_LENGTH:
                            if (compare_result.match.multi_rich_line.has_length()) {
                                match_length = compare_result.match.multi_rich_line.length();
                            }
                            break;
                        case column::GROUND_TRUTH_LENGTH:
                            if (compare_result.ground_truth.multi_rich_line.has_length()) {
                                ground_truth_length = compare_result.ground_truth.multi_rich_line.length();
                            }
                            break;
                        case column::CORRECTS:
                            corrects.reserve(compare_result.comparison.corrects.size());
                            for (const auto &correct : compare_result.comparison.corrects) {
                                corrects.emplace_back(line_type{correct.first(), correct.second()});
                            }
                            corrects_wkt = geometry::to_wkt(corrects);
                            break;
                        case column::ERROR_ADDS:
                            error_adds.reserve(compare_result.comparison.error_adds.size());
                            for (const auto &error_add : compare_result.comparison.error_adds) {
                                error_adds.emplace_back(line_type{error_add.first(), error_add.second()});
                            }
                            error_adds_wkt = geometry::to_wkt(error_adds);
                            break;
                        case column::ERROR_MISSES:
                            error_misses.reserve(compare_result.comparison.error_misses.size());
                            for (const auto &error_miss : compare_result.comparison.error_misses) {
                                error_misses.emplace_back(line_type{error_miss.first(), error_miss.second()});
                            }
                            error_misses_wkt = geometry::to_wkt(error_misses);
                            break;
                        default:
                            break;
                    }
                }
            }

            return {
                    compare_result.ground_truth.id,
                    std::move(match_wkt), std::move(ground_truth_wkt),
                    std::move(corrects_wkt), std::move(error_adds_wkt), std::move(error_misses_wkt),
                    compare_result.duration,
                    match_length, ground_truth_length,
                    compare_result.comparison.correct_fraction,
                    compare_result.comparison.error_fraction,
                    compare_result.comparison.correct,
                    compare_result.comparison.error_added,
                    compare_result.comparison.error_missed
            };
        }

        void pass(task_type &&task) {
            this->queue(std::move(task));
        }

    protected:
        enum class column {
            ID,
            DURATION,
            MATCH,
            GROUND_TRUTH,
            MATCH_LENGTH,
            GROUND_TRUTH_LENGTH,
            CORRECT_FRACTION,
            ERROR_FRACTION,
            CORRECT,
            ERROR_ADDED,
            ERROR_MISSED,
            CORRECTS,
            ERROR_ADDS,
            ERROR_MISSES,
            UNKNOWN
        };

        std::vector<column> _columns{};

        void _parse_columns() {
            _columns.reserve(this->_column_strings.size());
            for (const auto &_column_string : this->_column_strings) {
                if (_column_string == "id") {
                    _columns.emplace_back(column::ID);
                } else if (_column_string == "duration") {
                    _columns.emplace_back(column::DURATION);
                } else if (_column_string == "match") {
                    _columns.emplace_back(column::MATCH);
                } else if (_column_string == "ground_truth") {
                    _columns.emplace_back(column::GROUND_TRUTH);
                } else if (_column_string == "match_length") {
                    _columns.emplace_back(column::MATCH_LENGTH);
                } else if (_column_string == "ground_truth_length") {
                    _columns.emplace_back(column::GROUND_TRUTH_LENGTH);
                } else if (_column_string == "correct_fraction") {
                    _columns.emplace_back(column::CORRECT_FRACTION);
                } else if (_column_string == "error_fraction") {
                    _columns.emplace_back(column::ERROR_FRACTION);
                } else if (_column_string == "correct") {
                    _columns.emplace_back(column::CORRECT);
                } else if (_column_string == "error_added") {
                    _columns.emplace_back(column::ERROR_ADDED);
                } else if (_column_string == "error_missed") {
                    _columns.emplace_back(column::ERROR_MISSED);
                } else if (_column_string == "corrects") {
                    _columns.emplace_back(column::CORRECTS);
                } else if (_column_string == "error_adds") {
                    _columns.emplace_back(column::ERROR_ADDS);
                } else if (_column_string == "error_misses") {
                    _columns.emplace_back(column::ERROR_MISSES);
                } else {
                    _columns.emplace_back(column::UNKNOWN);
                }
            }
        }

        [[nodiscard]] std::vector<std::string> parse_task(task_type &&task) const {
            std::vector<std::string> result;
            result.reserve(_columns.size());
            for (const auto &_column : _columns) {
                switch (_column) {
                    case column::ID:
                        result.emplace_back(task.id);
                        break;
                    case column::DURATION:
                        result.emplace_back(std::to_string(task.duration));
                        break;
                    case column::MATCH:
                        result.emplace_back(task.match);
                        break;
                    case column::GROUND_TRUTH:
                        result.emplace_back(task.ground_truth);
                        break;
                    case column::MATCH_LENGTH:
                        result.emplace_back(std::to_string(task.match_length));
                        break;
                    case column::GROUND_TRUTH_LENGTH:
                        result.emplace_back(std::to_string(task.ground_truth_length));
                        break;
                    case column::CORRECT_FRACTION:
                        result.emplace_back(std::to_string(task.correct_fraction));
                        break;
                    case column::ERROR_FRACTION:
                        result.emplace_back(std::to_string(task.error_fraction));
                        break;
                    case column::CORRECT:
                        result.emplace_back(std::to_string(task.correct));
                        break;
                    case column::ERROR_ADDED:
                        result.emplace_back(std::to_string(task.error_added));
                        break;
                    case column::ERROR_MISSED:
                        result.emplace_back(std::to_string(task.error_missed));
                        break;
                    case column::CORRECTS:
                        result.emplace_back(task.corrects);
                        break;
                    case column::ERROR_ADDS:
                        result.emplace_back(task.error_adds);
                        break;
                    case column::ERROR_MISSES:
                        result.emplace_back(task.error_misses);
                        break;
                    default:
                        break;
                }
            }
            return result;
        }

        void execute(task_type &&task) override {
            this->write_result(parse_task(std::move(task)));
        }

    };

}

#endif //MAP_MATCHING_2_IO_RESULTS_COMPARE_CSV_COMPARE_EXPORTER_HPP
