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

#ifndef MAP_MATCHING_2_MATCHING_ALGORITHM_WRITE_HPP
#define MAP_MATCHING_2_MATCHING_ALGORITHM_WRITE_HPP

#include <string>
#include <vector>

#include "io/csv_exporter.hpp"

namespace map_matching_2::matching {

    template<typename Candidate>
    void save_candidates(std::string filename, const std::vector<Candidate> &candidates) {
        io::csv_exporter<',', '"'> csv{std::move(filename)};
        bool first = true;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            const auto &candidate = candidates[i];

            if (first) {
                auto candidate_header = candidate.header();
                std::vector<std::string> header;
                header.reserve(1 + candidate_header.size());
                header.emplace_back("id");
                std::move(candidate_header.begin(), candidate_header.end(), std::back_inserter(header));

                csv << header;
                first = false;
            }

            auto candidate_rows = candidate.rows();
            for (auto &candidate_row : candidate_rows) {
                std::vector<std::string> row;
                row.reserve(1 + candidate_row.size());
                row.emplace_back(std::to_string(i));
                std::move(candidate_row.begin(), candidate_row.end(), std::back_inserter(row));

                csv << row;
            }
        }
    }

}

#endif //MAP_MATCHING_2_MATCHING_ALGORITHM_WRITE_HPP
