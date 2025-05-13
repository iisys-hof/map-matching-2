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

#ifndef MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_TIME_SPLITTER_HPP
#define MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_TIME_SPLITTER_HPP

namespace map_matching_2::io::track::filter {

    template<typename Forwarder>
    class track_time_splitter {

    public:
        using forwarder_type = Forwarder;
        using multi_track_variant_type = typename forwarder_type::multi_track_variant_type;

        constexpr explicit track_time_splitter(forwarder_type &forwarder, const double split_time,
                const std::string &aggregator)
            : _forwarder{forwarder}, _split_time{split_time}, _aggregator{aggregator}, _no_split{split_time <= 0} {}

        void pass(multi_track_variant_type multi_track) {
            if (_no_split) {
                _forwarder.pass(std::move(multi_track));
            } else {
                _split_track(std::move(multi_track));
            }
        }

    protected:
        forwarder_type &_forwarder;
        const double _split_time;
        const std::string _aggregator;
        const bool _no_split{false};

        void _split_track(multi_track_variant_type &&multi_track) {
            std::visit([this]<typename MultiTrack>(MultiTrack &&multi_track) {
                using multi_track_type = std::remove_reference_t<MultiTrack>;
                using multi_rich_line_type = typename multi_track_type::multi_rich_line_type;
                using time_point_type = typename multi_track_type::point_type;

                const auto &multi_rich_line = multi_track.multi_rich_line;

                bool splitted = false;
                std::size_t index{0};
                std::size_t start_outer = 0;
                std::size_t start_inner = 0;
                const time_point_type *curr = nullptr;
                const time_point_type *prev = nullptr;
                for (std::size_t i = 0; i < multi_rich_line.size(); ++i) {
                    const auto &rich_line = multi_rich_line.at(i);
                    for (std::size_t j = 0; j < rich_line.size(); ++j) {
                        curr = &rich_line.at(j);
                        if (curr and prev) {
                            double time_difference = curr->timestamp() - prev->timestamp();
                            if (time_difference >= _split_time) {
                                _forwarder.pass(std::forward<MultiTrack>(multi_track_type{
                                        std::format("{}{}{}", multi_track.id, _aggregator, index++),
                                        multi_rich_line.template extract<multi_rich_line_type>(start_outer, start_inner,
                                                j, i + 1)
                                }));
                                start_outer = i;
                                start_inner = j > 0 ? j - 1 : 0;
                                splitted = true;
                            }
                        }
                        prev = curr;
                    }
                }

                if (splitted) {
                    _forwarder.pass(std::forward<MultiTrack>(multi_track_type{
                            std::format("{}{}{}", multi_track.id, _aggregator, index++),
                            multi_rich_line.template extract<multi_rich_line_type>(start_outer, start_inner,
                                    multi_rich_line.back().size(), multi_rich_line.size())
                    }));
                } else {
                    _forwarder.pass(std::forward<MultiTrack>(multi_track));
                }
            }, std::move(multi_track));
        }

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_TIME_SPLITTER_HPP
