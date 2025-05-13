// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_NETWORK_OSM_MANUAL_IMPORTER_HPP
#define MAP_MATCHING_2_IO_NETWORK_OSM_MANUAL_IMPORTER_HPP

#include <string>

#include "types/io/network/osm_importer.hpp"

#include "osm_pattern.hpp"

namespace map_matching_2::io::network {

    template<typename OSMHandler>
    class osm_manual_importer : public osm_importer<OSMHandler> {

    public:
        constexpr osm_manual_importer(std::string filename, OSMHandler &osm_handler,
                std::string query, std::string filter, std::string oneway, std::string oneway_reverse)
            : osm_importer<OSMHandler>{std::move(filename), osm_handler}, _query{std::move(query)},
            _filter{std::move(filter)}, _oneway{std::move(oneway)}, _oneway_reverse{std::move(oneway_reverse)} {}

        constexpr osm_manual_importer(std::vector<std::string> filenames, OSMHandler &osm_handler,
                std::string query, std::string filter, std::string oneway, std::string oneway_reverse)
            : osm_importer<OSMHandler>{std::move(filenames), osm_handler}, _query{std::move(query)},
            _filter{std::move(filter)}, _oneway{std::move(oneway)}, _oneway_reverse{std::move(oneway_reverse)} {}

    protected:
        std::string _query, _filter, _oneway, _oneway_reverse;

        struct rule {
            bool result{false}, has_value{true};
            std::string key{}, value{};
        };

        [[nodiscard]] static bool parse_bool(const std::string &str) {
            return str == "true";
        }

        [[nodiscard]] static std::vector<rule> parse_rules(const std::string &str) {
            std::vector<rule> rules;
            std::string::size_type start = 0;

            while (start < str.size()) {
                auto end = str.find('|', start);
                if (end == std::string::npos) {
                    end = str.size(); // end of string, adjust for last token
                }

                std::string token = str.substr(start, end - start);

                const auto comma_pos = token.find(',');
                const std::string default_value_str = token.substr(0, comma_pos);
                bool default_value = parse_bool(default_value_str);

                const auto equal_pos = token.find('=', comma_pos);
                bool has_value = equal_pos != std::string::npos;
                std::string tag, value;

                if (has_value) {
                    tag = token.substr(comma_pos + 1, equal_pos - comma_pos - 1);
                    value = token.substr(equal_pos + 1);
                } else {
                    tag = token.substr(comma_pos + 1);
                }

                rules.push_back({default_value, has_value, tag, value});

                start = end + 1; // move to next token
            }
            return rules;
        }

        [[nodiscard]] static osmium::TagsFilter create_tags_filter(const std::string &str) {
            auto rules = parse_rules(str);

            osmium::TagsFilter filter;
            for (std::size_t i = 0; i < rules.size(); ++i) {
                const auto &rule = rules[i];
                if (i == 0) {
                    filter.set_default_result(rule.result);
                } else {
                    if (rule.has_value) {
                        filter.add_rule(rule.result, rule.key, rule.value);
                    } else {
                        filter.add_rule(rule.result, rule.key);
                    }
                }
            }

            return filter;
        }

        osmium::TagsFilter query() override {
            return create_tags_filter(_query);
        }

        osmium::TagsFilter filter() override {
            return create_tags_filter(_filter);
        }

        osmium::TagsFilter oneway() override {
            return create_tags_filter(_oneway);
        }

        osmium::TagsFilter oneway_reverse() override {
            return create_tags_filter(_oneway_reverse);
        }

    };

}

#endif //MAP_MATCHING_2_IO_NETWORK_OSM_MANUAL_IMPORTER_HPP
