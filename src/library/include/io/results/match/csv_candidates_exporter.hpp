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

#ifndef MAP_MATCHING_2_IO_RESULTS_MATCH_CSV_CANDIDATES_EXPORTER_HPP
#define MAP_MATCHING_2_IO_RESULTS_MATCH_CSV_CANDIDATES_EXPORTER_HPP

#include <numeric>
#include <algorithm>

#include <boost/unordered/unordered_flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>

#include "../csv_results_exporter.hpp"

#include "geometry/types.hpp"

#include "geometry/algorithm/wkt.hpp"

namespace map_matching_2::io::results {

    struct candidate_result {
        std::string id, time, projection, from, to;
        std::size_t part_index, set_index, candidate_index;
        double distance;
        std::uint64_t edge_id;
        bool is_result;
    };

    struct candidates_csv_task {
        std::vector<candidate_result> candidates;
    };

    template<char Delimiter = ',', char Quote = '"', bool Flush = false>
    class csv_candidates_exporter : public csv_results_exporter<candidates_csv_task, Delimiter, Quote, Flush> {

    public:
        using task_type = candidates_csv_task;
        using base_type = csv_results_exporter<task_type, Delimiter, Quote, Flush>;

        explicit csv_candidates_exporter(std::string filename, const std::string &columns)
            : base_type{std::move(filename), columns} {
            _parse_columns();
        }

        ~csv_candidates_exporter() override = default;

        template<typename Network, typename MatchResult>
        [[nodiscard]] task_type parse_result(const Network &network, const MatchResult &match_result) const {
            using candidate_type = typename MatchResult::environment_type::candidate_type;
            using point_type = typename candidate_type::point_type;
            using segment_type = typename geometry::models<point_type>::segment_type;

            std::vector<candidate_result> results;

            if (this->is_active()) {
                std::vector<std::size_t> counts;
                for (const auto &environment : match_result.environments) {
                    counts.emplace_back(environment.edges());
                }
                results.reserve(std::reduce(std::cbegin(counts), std::cend(counts)));

                for (std::size_t i = 0; i < match_result.environments.size(); ++i) {
                    const auto &environment = match_result.environments.at(i);
                    const auto &candidates = environment.candidates();
                    const auto &policy = match_result.policies.at(i);

                    boost::unordered_flat_map<std::size_t, boost::unordered_flat_set<std::size_t>> policy_map;
                    for (auto [candidate_index, edge_index] : policy) {
                        policy_map[candidate_index].insert(edge_index);
                    }

                    std::vector<candidate_result> _results;
                    _results.reserve(counts.at(i));

                    for (std::size_t j = 0; j < candidates.size(); ++j) {
                        const auto &candidate = candidates[j];
                        for (std::size_t k = 0; k < candidate.edges.size(); ++k) {
                            const auto &candidate_edge = candidate.edges[k];
                            const auto &edge = network.edge(candidate_edge.edge_desc);

                            bool is_result{false};
                            std::string time{}, projection_wkt{}, from_wkt{}, to_wkt{};

                            for (const auto &_column : _columns) {
                                switch (_column) {
                                    case column::TIME: {
                                        const auto _time = candidate.point().timestamp();
                                        if (match_result.match_settings.export_timestamps) {
                                            time = std::to_string(_time);
                                        } else {
                                            time = util::format_time(_time,
                                                    match_result.match_settings.export_time_zone);
                                        }
                                    }
                                    break;
                                    case column::PROJECTION:
                                        projection_wkt = geometry::to_wkt(segment_type{
                                                candidate.point(), candidate_edge.projection_point
                                        });
                                        break;
                                    case column::FROM:
                                        from_wkt = geometry::to_wkt(candidate_edge.from->line());
                                        break;
                                    case column::TO:
                                        to_wkt = geometry::to_wkt(candidate_edge.to->line());
                                        break;
                                    case column::IS_RESULT:
                                        if (auto policy_search = policy_map.find(j);
                                            policy_search != policy_map.end()) {
                                            const auto &policy_edges_set = policy_search->second;
                                            is_result = policy_edges_set.find(k) != policy_edges_set.end();
                                        }
                                        break;
                                    default:
                                        break;
                                }
                            }

                            _results.emplace_back(candidate_result{
                                    match_result.track.id, time,
                                    std::move(projection_wkt), std::move(from_wkt), std::move(to_wkt),
                                    i, j, k,
                                    candidate_edge.distance,
                                    edge.id,
                                    is_result
                            });
                        }
                    }

                    std::move(std::begin(_results), std::end(_results), std::back_inserter(results));
                }
            }

            return {std::move(results)};
        }

        void pass(task_type &&task) {
            this->queue(std::move(task));
        }

    protected:
        enum class column {
            ID,
            PART_INDEX,
            SET_INDEX,
            CANDIDATE_INDEX,
            TIME,
            PROJECTION,
            DISTANCE,
            FROM,
            TO,
            EDGE_ID,
            IS_RESULT,
            UNKNOWN
        };

        std::vector<column> _columns{};

        void _parse_columns() {
            _columns.reserve(this->_column_strings.size());
            for (const auto &_column_string : this->_column_strings) {
                if (_column_string == "id") {
                    _columns.emplace_back(column::ID);
                } else if (_column_string == "part_index") {
                    _columns.emplace_back(column::PART_INDEX);
                } else if (_column_string == "set_index") {
                    _columns.emplace_back(column::SET_INDEX);
                } else if (_column_string == "candidate_index") {
                    _columns.emplace_back(column::CANDIDATE_INDEX);
                } else if (_column_string == "time") {
                    _columns.emplace_back(column::TIME);
                } else if (_column_string == "projection") {
                    _columns.emplace_back(column::PROJECTION);
                } else if (_column_string == "distance") {
                    _columns.emplace_back(column::DISTANCE);
                } else if (_column_string == "from") {
                    _columns.emplace_back(column::FROM);
                } else if (_column_string == "to") {
                    _columns.emplace_back(column::TO);
                } else if (_column_string == "edge_id") {
                    _columns.emplace_back(column::EDGE_ID);
                } else if (_column_string == "is_result") {
                    _columns.emplace_back(column::IS_RESULT);
                } else {
                    _columns.emplace_back(column::UNKNOWN);
                }
            }
        }

        [[nodiscard]] std::vector<std::vector<std::string>> parse_task(task_type &&task) const {
            std::vector<std::vector<std::string>> result;

            if (not this->filename().empty()) {
                result.reserve(task.candidates.size());

                for (const auto &_result : task.candidates) {
                    std::vector<std::string> row;
                    row.reserve(_columns.size());

                    for (const auto &_column : _columns) {
                        switch (_column) {
                            case column::ID:
                                row.emplace_back(_result.id);
                                break;
                            case column::PART_INDEX:
                                row.emplace_back(std::to_string(_result.part_index));
                                break;
                            case column::SET_INDEX:
                                row.emplace_back(std::to_string(_result.set_index));
                                break;
                            case column::CANDIDATE_INDEX:
                                row.emplace_back(std::to_string(_result.candidate_index));
                                break;
                            case column::TIME:
                                row.emplace_back(_result.time);
                                break;
                            case column::PROJECTION:
                                row.emplace_back(_result.projection);
                                break;
                            case column::DISTANCE:
                                row.emplace_back(std::to_string(_result.distance));
                                break;
                            case column::FROM:
                                row.emplace_back(_result.from);
                                break;
                            case column::TO:
                                row.emplace_back(_result.to);
                                break;
                            case column::EDGE_ID:
                                row.emplace_back(std::to_string(_result.edge_id));
                                break;
                            case column::IS_RESULT:
                                row.emplace_back(std::to_string(_result.is_result));
                                break;
                            default:
                                break;
                        }
                    }

                    result.emplace_back(std::move(row));
                }
            }

            return result;
        }

        void write_result(std::vector<std::vector<std::string>> &&results) {
            for (auto &&result : results) {
                base_type::write_result(std::move(result));
            }
        }

        void execute(task_type &&task) override {
            write_result(parse_task(std::move(task)));
        }

    };

}

#endif //MAP_MATCHING_2_IO_RESULTS_MATCH_CSV_CANDIDATES_EXPORTER_HPP
