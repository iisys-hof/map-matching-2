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

#ifndef MAP_MATCHING_2_IO_RESULTS_MATCH_CSV_MATCH_EXPORTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_MATCH_CSV_MATCH_EXPORTER_HPP

#include <sstream>

#include "../csv_results_exporter.hpp"

#include "geometry/common.hpp"

namespace map_matching_2::io::results {

    struct match_csv_task {
        std::string id, track, prepared, match, edge_ids;
        bool aborted;
        double duration, track_length, prepared_length, match_length;
    };

    template<char Delimiter = ',', char Quote = '"', bool Flush = false>
    class csv_match_exporter : public csv_results_exporter<match_csv_task, Delimiter, Quote, Flush> {

    public:
        using task_type = match_csv_task;
        using base_type = csv_results_exporter<task_type, Delimiter, Quote, Flush>;

        explicit csv_match_exporter(std::string filename, const std::string &columns)
            : base_type{std::move(filename), columns} {
            _parse_columns();
        }

        ~csv_match_exporter() override = default;

        template<typename MatchResult>
        [[nodiscard]] task_type parse_result(const MatchResult &match_result) const {
            using length_type = typename MatchResult::multi_track_type::length_type;
            static constexpr auto length_zero = geometry::default_float_type<length_type>::v0;

            std::string track_wkt{}, prepared_wkt{}, match_wkt{}, edge_ids{};
            length_type track_length{length_zero}, prepared_length{length_zero}, match_length{length_zero};

            if (this->is_active()) {
                for (const auto &_column : _columns) {
                    switch (_column) {
                        case column::TRACK:
                            track_wkt = match_result.track.wkt();
                            break;
                        case column::PREPARED:
                            prepared_wkt = match_result.prepared.wkt();
                            break;
                        case column::MATCH:
                            match_wkt = match_result.match.wkt();
                            break;
                        case column::TRACK_LENGTH:
                            if (match_result.track.multi_rich_line.has_length()) {
                                track_length = match_result.track.multi_rich_line.length();
                            }
                            break;
                        case column::PREPARED_LENGTH:
                            if (match_result.prepared.multi_rich_line.has_length()) {
                                prepared_length = match_result.prepared.multi_rich_line.length();
                            }
                            break;
                        case column::MATCH_LENGTH:
                            if (match_result.match.has_length()) {
                                match_length = match_result.match.length();
                            }
                            break;
                        case column::EDGE_IDS:
                            if (not match_result.edge_ids.empty()) {
                                std::ostringstream oss;
                                bool first = true;
                                for (const auto &edge_id : match_result.edge_ids) {
                                    if (not first) {
                                        oss << ',';
                                    }
                                    first = false;
                                    oss << edge_id;
                                }
                                edge_ids = oss.str();
                            }
                            break;
                        default:
                            break;
                    }
                }
            }

            return {
                    match_result.track.id,
                    std::move(track_wkt), std::move(prepared_wkt), std::move(match_wkt),
                    std::move(edge_ids),
                    match_result.aborted, match_result.duration,
                    track_length, prepared_length, match_length,
            };
        }

        void pass(task_type &&task) {
            this->queue(std::move(task));
        }

    protected:
        enum class column {
            ID,
            ABORTED,
            DURATION,
            TRACK,
            PREPARED,
            MATCH,
            TRACK_LENGTH,
            PREPARED_LENGTH,
            MATCH_LENGTH,
            EDGE_IDS,
            UNKNOWN
        };

        std::vector<column> _columns{};

        void _parse_columns() {
            _columns.reserve(this->_column_strings.size());
            for (const auto &_column_string : this->_column_strings) {
                if (_column_string == "id") {
                    _columns.emplace_back(column::ID);
                } else if (_column_string == "aborted") {
                    _columns.emplace_back(column::ABORTED);
                } else if (_column_string == "duration") {
                    _columns.emplace_back(column::DURATION);
                } else if (_column_string == "track") {
                    _columns.emplace_back(column::TRACK);
                } else if (_column_string == "prepared") {
                    _columns.emplace_back(column::PREPARED);
                } else if (_column_string == "match") {
                    _columns.emplace_back(column::MATCH);
                } else if (_column_string == "track_length") {
                    _columns.emplace_back(column::TRACK_LENGTH);
                } else if (_column_string == "prepared_length") {
                    _columns.emplace_back(column::PREPARED_LENGTH);
                } else if (_column_string == "match_length") {
                    _columns.emplace_back(column::MATCH_LENGTH);
                } else if (_column_string == "edge_ids") {
                    _columns.emplace_back(column::EDGE_IDS);
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
                    case column::ABORTED:
                        result.emplace_back(task.aborted ? "yes" : "no");
                        break;
                    case column::DURATION:
                        result.emplace_back(std::to_string(task.duration));
                        break;
                    case column::TRACK:
                        result.emplace_back(task.track);
                        break;
                    case column::PREPARED:
                        result.emplace_back(task.prepared);
                        break;
                    case column::MATCH:
                        result.emplace_back(task.match);
                        break;
                    case column::TRACK_LENGTH:
                        result.emplace_back(std::to_string(task.track_length));
                        break;
                    case column::PREPARED_LENGTH:
                        result.emplace_back(std::to_string(task.prepared_length));
                        break;
                    case column::MATCH_LENGTH:
                        result.emplace_back(std::to_string(task.match_length));
                        break;
                    case column::EDGE_IDS:
                        result.emplace_back(task.edge_ids);
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

#endif //MAP_MATCHING_2_IO_RESULTS_MATCH_CSV_MATCH_EXPORTER_HPP
