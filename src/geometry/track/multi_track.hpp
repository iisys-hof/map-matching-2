// Copyright (C) 2021-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MULTI_TRACK_HPP
#define MAP_MATCHING_2_MULTI_TRACK_HPP

#include "../multi_rich_line.hpp"
#include "track.hpp"

namespace map_matching_2::geometry::track {


    template<typename MultiTrack>
    [[nodiscard]] static typename MultiTrack::multi_line_type
    tracks2multi_line(const std::vector<typename MultiTrack::track_type> &tracks) {
        using multi_line_type = typename MultiTrack::multi_line_type;
        using track_type = typename MultiTrack::track_type;

        multi_line_type multi_line;
        multi_line.reserve(tracks.size());
        for (const track_type &track: tracks) {
            if (not track.empty()) {
                multi_line.emplace_back(track.line());
            }
        }
        return multi_line;
    }

    template<typename MultiTrack, template<typename> class Container>
    [[nodiscard]] static Container<typename MultiTrack::rich_line_type>
    tracks2rich_lines(const std::vector<typename MultiTrack::track_type> &tracks) {
        using rich_line_type = typename MultiTrack::rich_line_type;

        Container<rich_line_type> rich_lines;
        if (not tracks.empty()) {
            if constexpr (std::is_void_v<decltype(&Container<rich_line_type>::reserve)>) {
                rich_lines.reserve(tracks.size());
            }
            for (const auto &track: tracks) {
                if (not track.empty()) {
                    rich_lines.emplace_back(rich_line_type{track});
                }
            }
        }
        return rich_lines;
    }

    template<typename MultiTrack, template<typename> class Container>
    [[nodiscard]] static std::vector<typename MultiTrack::track_type>
    multi_line2tracks(const std::string &id, const typename MultiTrack::multi_line_type &multi_line) {
        using track_type = typename MultiTrack::track_type;

        Container<track_type> tracks;
        if (not multi_line.empty()) {
            if constexpr (std::is_void_v<decltype(&Container<track_type>::reserve)>) {
                tracks.reserve(multi_line.size());
            }
            for (const auto &line: multi_line) {
                if (not line.empty()) {
                    tracks.emplace_back(track_type{id, line});
                }
            }
        }
        return tracks;
    }

    template<typename MultiRichLine>
    struct multi_track : public MultiRichLine {

        using multi_rich_line_type = MultiRichLine;
        using point_type = typename multi_rich_line_traits<multi_rich_line_type>::point_type;
        using line_type = typename multi_rich_line_traits<multi_rich_line_type>::line_type;
        using multi_line_type = typename multi_rich_line_traits<multi_rich_line_type>::multi_line_type;
        using rich_line_type = typename multi_rich_line_traits<multi_rich_line_type>::rich_line_type;
        using distance_type = typename multi_rich_line_traits<multi_rich_line_type>::distance_type;
        using track_type = track<rich_line_type>;
        using measurement_type = measurement<point_type>;

        std::string id;

        multi_track()
                : multi_track{"", track_type{}} {}

        multi_track(std::string id, track_type track)
                : multi_track{std::move(id), std::vector{std::move(track)}} {}

        multi_track(std::string id, const line_type &line)
                : multi_track{id, track_type{std::move(id), line}} {}

        multi_track(std::string id, rich_line_type rich_line)
                : multi_track{id, track_type{std::move(id), std::move(rich_line)}} {}

        multi_track(std::string id, const std::vector<line_type> &lines)
                : multi_rich_line_type{lines}, id{std::move(id)} {}

        multi_track(std::string id, const multi_rich_line_type &multi_rich_line)
                : multi_rich_line_type{multi_rich_line}, id{std::move(id)} {}

        multi_track(std::string id, const std::vector<track_type> &tracks)
                : multi_rich_line_type{tracks2rich_lines<multi_track, std::vector>(tracks)}, id{std::move(id)} {}

        multi_track(std::string id, std::vector<rich_line_type> rich_lines)
                : multi_rich_line_type{std::move(rich_lines)}, id{std::move(id)} {}

        ~multi_track() = default;

        multi_track(const multi_track &other)
                : multi_rich_line_type{static_cast<const multi_rich_line_type &>(other)}, id{other.id} {}

        template<typename MultiRichLineT>
        multi_track(const multi_track<MultiRichLineT> &other)
                : multi_rich_line_type{static_cast<const MultiRichLineT &>(other)}, id{other.id} {}

        multi_track(multi_track &&other) noexcept
                : multi_rich_line_type{std::move(static_cast<multi_rich_line_type &&>(other))},
                  id{std::move(other.id)} {}

        template<typename MultiRichLineT>
        multi_track(multi_track<MultiRichLineT> &&other) noexcept
                : multi_rich_line_type{std::move(static_cast<MultiRichLineT &&>(other))},
                  id{std::move(other.id)} {}

        multi_track &operator=(const multi_track &other) {
            multi_rich_line_type::operator=(static_cast<const multi_rich_line_type &>(other));
            id = other.id;
            return *this;
        }

        template<typename MultiRichLineT>
        multi_track &operator=(const multi_track<MultiRichLineT> &other) {
            multi_rich_line_type::operator=(static_cast<const MultiRichLineT &>(other));
            id = other.id;
            return *this;
        }

        multi_track &operator=(multi_track &&other) noexcept {
            multi_rich_line_type::operator=(std::move(static_cast<multi_rich_line_type &&>(other)));
            id = std::move(other.id);
            return *this;
        }

        template<typename MultiRichLineT>
        multi_track &operator=(multi_track<MultiRichLineT> &&other) noexcept {
            multi_rich_line_type::operator=(std::move(static_cast<MultiRichLineT &&>(other)));
            id = std::move(other.id);
            return *this;
        }

        template<typename MultiTrackReprojected, typename SRSOriginal, typename SRSProjected>
        MultiTrackReprojected reproject(SRSOriginal from, SRSProjected to) const {
            using reprojected_track_type = typename MultiTrackReprojected::track_type;

            std::vector<reprojected_track_type> reprojected_tracks;
            reprojected_tracks.reserve(this->rich_lines().size());
            for (const rich_line_type &rich_line: this->rich_lines()) {
                track_type track{id, rich_line};
                reprojected_tracks.emplace_back(
                        track.template reproject<reprojected_track_type, SRSOriginal, SRSProjected>(from, to));
            }

            return MultiTrackReprojected{id, reprojected_tracks};
        }

        [[nodiscard]] std::vector<std::string> header() const {
            return {"id", "geometry"};
        }

        [[nodiscard]] std::vector<std::string> row() const {
            std::vector<std::string> row;
            row.reserve(2);
            row.emplace_back(this->id);
            row.emplace_back(this->wkt());
            return row;
        }

        [[nodiscard]] std::string str() const {
            const auto &fields = row();
            std::string str = "MultiTrack(";
            bool first = true;
            for (const auto &field: fields) {
                if (!first) {
                    str.append(",");
                }
                first = false;
                str.append(field);
            }
            str.append(")");
            return str;
        }

        static void sort_tracks(std::vector<multi_track> &multi_tracks, bool ascending = true) {
            if (ascending) {
                std::sort(multi_tracks.begin(), multi_tracks.end(), [&](const auto &left, const auto &right) {
                    const auto sizes = std::pair{left.sizes(), right.sizes()};
                    return sizes.first < sizes.second;
                });
            } else {
                std::sort(multi_tracks.begin(), multi_tracks.end(), [&](const auto &left, const auto &right) {
                    const auto sizes = std::pair{left.sizes(), right.sizes()};
                    return sizes.first > sizes.second;
                });
            }
        }

        friend std::ostream &operator<<(std::ostream &out, const multi_track &multi_track) {
            return out << multi_track.str();
        }

    };

    template<typename Point>
    using multi_track_type = multi_track<multi_rich_line_type<Point>>;

    template<typename Point>
    using eager_multi_track_type = multi_track<eager_multi_rich_line_type<Point>>;

    template<typename Point>
    using lazy_multi_track_type = multi_track<lazy_multi_rich_line_type<Point>>;

}

#endif //MAP_MATCHING_2_MULTI_TRACK_HPP
