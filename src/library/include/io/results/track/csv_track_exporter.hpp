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

#ifndef MAP_MATCHING_2_IO_RESULTS_TRACK_CSV_TRACK_EXPORTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_TRACK_CSV_TRACK_EXPORTER_HPP

#include "../csv_results_exporter.hpp"

namespace map_matching_2::io::results {

    struct track_csv_task {
        std::string id, track;
    };

    template<char Delimiter = ',', char Quote = '"', bool Flush = false>
    class csv_track_exporter : public csv_results_exporter<track_csv_task, Delimiter, Quote, Flush> {

    public:
        using task_type = track_csv_task;
        using base_type = csv_results_exporter<task_type, Delimiter, Quote, Flush>;

        explicit csv_track_exporter(std::string filename)
            : base_type{std::move(filename), "id,track"} {
            _parse_columns();
        }

        ~csv_track_exporter() override = default;

        template<typename MultiTrack>
        [[nodiscard]] task_type parse_result(const MultiTrack &multi_track) const {
            std::string wkt{};

            if (this->is_active()) {
                for (const auto &_column : _columns) {
                    switch (_column) {
                        case column::TRACK:
                            wkt = multi_track.wkt();
                            break;
                        default:
                            break;
                    }
                }
            }

            return {multi_track.id, std::move(wkt)};
        }

        void pass(task_type &&task) {
            this->queue(std::move(task));
        }

    protected:
        enum class column {
            ID,
            TRACK,
            UNKNOWN
        };

        std::vector<column> _columns{};

        void _parse_columns() {
            _columns.reserve(this->_column_strings.size());
            for (const auto &_column_string : this->_column_strings) {
                if (_column_string == "id") {
                    _columns.emplace_back(column::ID);
                } else if (_column_string == "track") {
                    _columns.emplace_back(column::TRACK);
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
                    case column::TRACK:
                        result.emplace_back(task.track);
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

#endif //MAP_MATCHING_2_IO_RESULTS_TRACK_CSV_TRACK_EXPORTER_HPP
