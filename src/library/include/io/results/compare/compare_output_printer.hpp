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

#ifndef MAP_MATCHING_2_IO_RESULTS_COMPARE_COMPARE_OUTPUT_PRINTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_COMPARE_COMPARE_OUTPUT_PRINTER_HPP

#include "../results_output_printer.hpp"

namespace map_matching_2::io::results {

    struct compare_output_task {
        std::string id, corrects, error_adds, error_misses;
        std::size_t match_points, ground_truth_points;
        double duration, match_length, ground_truth_length,
                correct_fraction, error_fraction, correct, error_added, error_missed;
    };

    class compare_output_printer : public results_output_printer<compare_output_task> {

    public:
        using task_type = compare_output_task;
        using base_type = results_output_printer<task_type>;

        compare_output_printer(const bool console, const bool verbose)
            : base_type{console, verbose} {}

        ~compare_output_printer() override {
            this->print_result(parse_aggregated_result());
        }

        template<typename CompareResult>
        [[nodiscard]] task_type parse_result(const CompareResult &compare_result) {
            using length_type = typename CompareResult::multi_track_type::length_type;
            static constexpr auto length_zero = geometry::default_float_type<length_type>::v0;

            std::size_t match_sizes{0}, ground_truth_sizes{0};
            length_type match_length{length_zero}, ground_truth_length{length_zero};
            std::string corrects{}, error_adds{}, error_misses{};

            if (this->is_active()) {
                if (this->is_console()) {
                    for (const auto &correct : compare_result.comparison.corrects) {
                        corrects.append(correct.wkt()).append("\n");
                    }
                    for (const auto &error_add : compare_result.comparison.error_adds) {
                        error_adds.append(error_add.wkt()).append("\n");
                    }
                    for (const auto &error_miss : compare_result.comparison.error_misses) {
                        error_misses.append(error_miss.wkt()).append("\n");
                    }
                } else {
                    match_sizes = compare_result.match.multi_rich_line.sizes();
                    ground_truth_sizes = compare_result.ground_truth.multi_rich_line.sizes();
                    if (compare_result.match.multi_rich_line.has_length()) {
                        match_length = compare_result.match.multi_rich_line.length();
                    }
                    if (compare_result.ground_truth.multi_rich_line.has_length()) {
                        ground_truth_length = compare_result.ground_truth.multi_rich_line.length();
                    }
                }
            }

            return {
                    compare_result.ground_truth.id,
                    std::move(corrects), std::move(error_adds), std::move(error_misses),
                    match_sizes, ground_truth_sizes,
                    compare_result.duration, match_length, ground_truth_length,
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
        std::size_t _ground_truth_points{}, _match_points{};
        double _match_length_sum{}, _ground_truth_length_sum{},
                _correct_fraction_sum{}, _error_fraction_sum{},
                _correct_sum{}, _error_added_sum{}, _error_missed_sum{};

        [[nodiscard]] std::string parse_task(task_type &&task) {
            if (this->is_active()) {
                if (this->is_console()) {
                    return std::format("Comparison '{}':\nCorrects:\n{}Error Adds:\n{}Error Misses:{}\n",
                            task.id, task.corrects, task.error_adds, task.error_misses);
                } else {
                    this->_duration += task.duration;
                    _ground_truth_points += task.ground_truth_points;
                    _match_points += task.match_points;
                    _match_length_sum += task.match_length;
                    _ground_truth_length_sum += task.ground_truth_length;
                    _correct_fraction_sum += task.correct_fraction;
                    _error_fraction_sum += task.error_fraction;
                    _correct_sum += task.correct;
                    _error_added_sum += task.error_added;
                    _error_missed_sum += task.error_missed;

                    return std::format(
                            "Comparison {} with id '{}' done in {:.3f} s, "
                            "ground truth points: {}, match points: {}, "
                            "correct fraction: {:.5f}, error fraction: {:.5f}, "
                            "correct: {:.2f}, error added: {:.2f}, error missed: {:.2f}",
                            ++this->_counter, task.id, task.duration,
                            task.ground_truth_points, task.match_points,
                            task.correct_fraction, task.error_fraction,
                            task.correct, task.error_added, task.error_missed);
                }
            }
            return "";
        }

        [[nodiscard]] std::pair<double, double> _compute_weighted_errors() const {
            // for computation details see geometry/algorithm/compare.hpp

            double weighted_mean_correct = 0.0;
            double weighted_mean_error = 0.0;

            if (_error_added_sum != 0.0 or _error_missed_sum != 0.0) {
                if (_ground_truth_length_sum > 0.0) {
                    weighted_mean_error = (_error_added_sum + _error_missed_sum) / _ground_truth_length_sum;
                } else {
                    weighted_mean_error = std::numeric_limits<double>::infinity();
                }
            }

            if (weighted_mean_error == 0.0) {
                weighted_mean_correct = 1.0;
            } else {
                if (_ground_truth_length_sum > 0.0 and _match_length_sum > 0.0) {
                    weighted_mean_correct = _correct_sum / std::max(_match_length_sum, _ground_truth_length_sum);
                } else if (_ground_truth_length_sum == 0.0 and _match_length_sum == 0.0) {
                    weighted_mean_correct = 1.0;
                } else {
                    weighted_mean_correct = 0.0;
                }

                weighted_mean_correct = std::min(1.0, std::max(0.0, weighted_mean_correct));
            }

            return {weighted_mean_correct, weighted_mean_error};
        }

        [[nodiscard]] std::string parse_aggregated_result() {
            if (this->is_active()) {
                if (this->_counter > 0) {
                    auto [weighted_mean_correct, weighted_mean_error] = _compute_weighted_errors();

                    return std::format(
                            "Compares count: {}, total duration: {:.3f} s, "
                            "ground truths points: {}, matches points: {}, "
                            "ground truths length: {:.2f}, matches length: {:.2f}, "
                            "mean correct fraction: {:.5f}, weighted mean correct fraction: {:.5f}, "
                            "mean error fraction: {:.5f}, weighted mean error fraction: {:.5f}, "
                            "correct length: {:.2f}, errors added: {:.2f}, errors missed: {:.2f}",
                            this->_counter, this->_duration, _ground_truth_points, _match_points,
                            _ground_truth_length_sum, _match_length_sum,
                            _correct_fraction_sum / this->_counter, weighted_mean_correct,
                            _error_fraction_sum / this->_counter, weighted_mean_error,
                            _correct_sum, _error_added_sum, _error_missed_sum);
                }
            }
            return "";
        }

        void execute(task_type &&task) override {
            this->print_result(parse_task(std::move(task)));
        }

    };

}

#endif //MAP_MATCHING_2_IO_RESULTS_COMPARE_COMPARE_OUTPUT_PRINTER_HPP
