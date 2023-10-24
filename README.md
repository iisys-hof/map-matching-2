# Map Matching 2

### High Performance Map Matching with Markov Decision Processes (MDPs) and Hidden Markov Models (HMMs)

Please read the official article for more information about this open source tool: \
**Open source map matching with Markov decision processes: A new method and a detailed benchmark with existing
approaches** (https://doi.org/10.1111/tgis.13107)

Currently the following models are implemented:

* Markov Decision Process (MDP) for single tracks. Highest accuracy and very high speed. **Default.**
* Hidden Markov Model (HMM) for single tracks. Slightly higher speed, but less accurate.

Currently the following algorithms are implemented:

* for Markov Decision Processes
    * Value Iteration. Highest accuracy and high speed. **Default.**
    * Policy Iteration. Performance equivalent to Value Iteration.
    * Q-learning. Reinforcement Learning algorithm, slightly slower and less accurate.
* for Hidden Markov Models
    * Viterbi algorithm. Classical approach.

Various advanced map matching algorithms are implemented, for example:

* OpenStreetMap `.osm.pbf` import of road network
* Tags for car-network currently implemented, other network types such as bycicles, walking, railroads will come in
  future updates, maybe also individual tag specifications
* `.csv` import of tracks, either as list of points or WKT lines or multilines
* `.gpx` import of tracks (from TRK records, multiple segments per track are supported)
* Multiple input files supported by specifying multiple track files, for example multiple `.gpx` files
* Import in shell pipeline mode of WKT lines or multilines (optional)
* Native import and matching of the map matching dataset
  at [Dataset for testing and training map-matching methods](https://zenodo.org/record/57731) (optional)
* Export of imported map data in various processing stages for easy view in [QGIS](https://www.qgis.org/)
* Export of processed map data in `.osm.pbf` format for faster reimport or usage of same map data in third-party tools
  (optional)
* Export of processed map data as native binary data file for about ten times faster reimport compared to
  using `.osm.pbf`  file again, depends on the speed of the underlying storage, SSD or NVMe recommended, also the binary
  files are about five to ten times larger than the `.osm.pbf` file (optional)
* Comparison methods for matching results of own results and results of external software (optional)
* Lossless Network simplification for reducing nodes and edges
* Baking of imported modifiable graph (as lists) to a static graph (as vectors) for faster traversal with Dijkstra
  algorithm
* Reprojections between geographical and cartesian spatial reference systems (SRSs) and matching in both SRSs (optional)
* Upper bound Dijkstra algorithm for routing
* Early stopping Dijkstra algorithm when finding all goals (we tried A-Star algorithm but removed it because A-Star only
  works between two points, Dijkstra works between a single source and many targets, which is better for map matching
  between one candidate and all next possible candidates)
* Fat initialized segments and lines for edges (e.g., network edges, candidate edges)
* Removing of weakly unconnected subgraphs (optional)
* Caches for length and bearing calculations
* Caches for routes from single-source Dijkstra algorithm
* Track sanitation with Douglas-Peucker algorithm
* Removing of spatially duplicate points
* Median merging of point clouds, with adaptive distance when the next road is far away
* Candidate search based on adaptive circle radius, k-nearest neighbor, and combined mode
* Candidate adoption of siblings and/or nearby candidates
* Within edge turn functionality for disabled candidate adoption
* Matching of tracks with gaps with MDP and skip-errors option (default)
* Tunable weights for HMM and MDP models
* Index mapping for MDP model and algorithms for rewards, V and Q tables (so rewards are also cached)
* In-Place versions of Value Iteration and Policy Iteration with beneficial ordering of the states for very quick
  convergence
* Tunable parameters for all algorithms, such as learning rate, discount factor, epsilon, etc.
* Output to `.csv` file for easy view in [QGIS](https://www.qgis.org/) or optional console for shell pipeline mode
* Quiet mode and verbose mode with benchmark timings (optional)
* Multi-threading which can optionally be disabled
* Improved memory management with [rpmalloc](https://github.com/mjansson/rpmalloc)
* *More features are in the making ...*

The provided source code is heavily tuned and optimized for fast and accurate map matching. \
We are currently working on more new research papers based on this approach.

Pull from Docker Hub: \
`docker pull addy90/map-matching-2`

Or build docker image: \
`docker build -t map-matching-2 .`

Run docker container: \
`docker run --rm -it -v $(pwd)/data:/app/data -u $(id -u ${USER}):$(id -g ${USER}) --name map-matching-2 map-matching-2`

Place input files in current data directory `$(pwd)/data` that you created beforehand. \
Within the Docker container, the directory is mounted under `/app/data`. \
Results that you output to `/app/data` will become available under your user in `$(pwd)/data`.

View Map Matching Help (or view [help.txt](help.txt)): \
`./map_matching_2 --help`

### Use

We recommend preparing the network graph as binary file for (approximately ten times) faster reimport in succeeding
runs:

```
./map_matching_2 \
  --network "/app/data/oberfranken-latest.osm.pbf" \
  --network-save "/app/data/oberfranken-latest.dat" \
  --verbose
```

The network can also be transformed into a cartesian coordinate system, for
example [WGS-84 Pseudo-Mercator](https://epsg.io/3857) by setting
`--network-transform-srs "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs"`
. The PROJ.4 string comes from the provided link.

| Mode                                     | Time (s) | CPU resources (s) | Max RAM (MB) | `.osm.pbf` size (MB) | `.dat` size (MB) |
|------------------------------------------|----------|-------------------|--------------|----------------------|------------------|
| Prepare Network                          | 12.46    | 14.82             | 1793         | 75                   | 254              |
| Prepare Network (Cartesian Reprojection) | 11.07    | 13.29             | 1792         | 75                   | 254              |

<details>
<summary>Prepare Commands</summary>

```
# Prepare Network
./map_matching_2 \
  --network "/app/data/oberfranken-latest.osm.pbf" \
  --network-save "/app/data/oberfranken-latest.dat" \
  --verbose
```

```
# Prepare Network (Cartesian Reprojection)
./map_matching_2 \
  --network "/app/data/oberfranken-latest.osm.pbf" \
  --network-transform-srs "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext  +no_defs" \
  --network-save "/app/data/oberfranken-latest-cartesian.dat" \
  --verbose
```

</details>

The times are measured with `/usr/bin/time -v` prepended to the commands above. The file sized were read from the data
folder via `ls -lah` command.

For using the exported network graph instead of original input file, simply replace `--network` with `--network-load`
and the previously exported file from `--network-save` (see above):

```
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  [... remaining options ...]
  --verbose
```

When you want to read the cartesian reprojected network, you need to set the network
`--network-srs "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs"`
. And maybe you also need to set the tracks via `--tracks-srs` or transform them via `--tracks-transform-srs`.

The reading time is the same in both cases.

| Mode                  | Time (s) | CPU resources (s) | Max RAM (MB) |
|-----------------------|----------|-------------------|--------------|
| Read Prepared Network | 2.74     | 2.74              | 806          |

<details>
<summary>Read Prepared Network Commands</summary>

The command that was used for reading this time is the following. The software normally requires tracks supplied, but
it can be tricked by supplying an empty track in readline mode in shell pipeline syntax:

```
# Read Prepared Network
echo "" | ./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  --readline --no-compare \
  --verbose
```

When you want to measure this via `/usr/bin/time -v` you need to put the command in `/bin/bash -c '<cmd>'` so that it is
measured completely, else only the echo is measured.

</details>

We can see that reading the prepared graph is a lot faster and needs less resources.

Now we can use the prepared network graph in real use-cases, for example with the provided floating car data, here it
takes up to 64 GB of RAM on 256 CPU threads:

```
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  --tracks "/app/data/points_anonymized.csv" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "/app/data/matches.csv" \
  --verbose
```

Instead of using the prepared network file with `--network-load`, the original `.osm.pbf` file can also be used with
`--network` directly, but then it is parsed from scratch on every run.

Concerning the cartesian use-case, after preparing the network, we load the corresponding file with `--network-srs` set
to the cartesian PROJ.4 string and we transform the tracks by setting `--tracks-transform-srs` to the same PROJ.4
string.

If you don't have that much RAM, disable candidate adoption (CA) with \
`--candidate-adoption-siblings off --candidate-adoption-nearby off` \
but this reduces the matching accuracy slightly.

Instead of regular adaptive circle based candidate search, also k-nearest neighbors (by default 16, but can be changed)
with `--candidate-search nearest` and a combined method of both by `--candidate-search combined` is possible. The
combined search first uses the adaptive circle, then only retains the k-nearest candidates. This reduces the amount of
candidates and combinations drastically, especially in dense areas. The accuracy is slightly reduced, however the
computational speed benefits drastically. Therefore, combined candidate search is currently the default method.

If you don't want to use multi-threading for map matching enable \
`--single-threading`. \
Without parallel matching, much less max memory is needed. Typically, computers with many cores also (should) have more
system memory available. If this is not the case, single-threading mode is highly advised.

The follwing examples were run on a dedicated server with 2x AMD EPYC 7742 64-Core Processor with 2x 128 Threads and
1024 MB DDR4 RAM on a local NVMe SSD. The default settings are the command above. The sanitized track points are after
track simplification and median-merge, which is on by default. The candidates are the amount of road positions for all
sanitized track points. Each sanitized track point has one set of candidates assigned. Candidate adoption merges
adjacent and nearby sets. A combination is a pair of adjacent candidates, for every two adjacent track points, all
candidates are evaluated in a cross-product table, so for every combination, a route is calculated and metrics are
computed and compared. This data is outputted by `--verbose` mode.

All times are measured with `Read Prepared Network` time above included. We can see that especially for the fast
variants, the preparation is highly recommended. This is just an example for the `1300` tracks from the
provided `points_anonymized.csv` file. Other data and hardware leads to other results.

| Mode                                        | Track points | Sanitized track points | Candidates |  Combinations | Time (s) | CPU resources (s) | Max RAM (MB) |
|:--------------------------------------------|-------------:|-----------------------:|-----------:|--------------:|---------:|------------------:|-------------:|
| Default settings                            |       88,807 |                 32,585 |  7,723,164 | 2,524,741,847 |      184 |             7,775 |       50,256 |
| Default settings cartesian                  |       88,807 |                 37,901 |  4,631,332 |   88,3948,571 |      102 |             2,671 |       23,638 |
| Disabled candidate adoption (no CA)         |       88,807 |                 32,585 |  2,319,264 |   201,870,553 |     27,8 |             2,700 |       16,212 |
| Combined candidate search (with CA)         |       88,807 |                 32,585 |    799,044 |    27,177,433 |     9,01 |               820 |        6,215 |
| Combined (no CA)                            |       88,807 |                 32,585 |    255,093 |     2,644,697 |     5,88 |               456 |        4,226 |
| Combined single threading (no CA)           |       88,807 |                 32,585 |    255,093 |     2,644,697 |     65,0 |              65,0 |          868 |
| Combined single threading cartesian (no CA) |       88,807 |                 37,901 |    298,084 |     3,142,440 |     24,5 |              18,9 |          878 |

<details>
<summary>Map Matching Benchmark Commands</summary>

```
# Default settings
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  --tracks "/app/data/points_anonymized.csv" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "/app/data/matches.csv" \
  --verbose
```

```
# Default settings cartesian
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest-cartesian.dat" \
  --network-srs "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs" \
  --tracks "/app/data/points_anonymized.csv" \
  --tracks-transform-srs "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "/app/data/matches.csv" \
  --verbose
```

```
# Disabled candidate adoption (no CA)
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  --tracks "/app/data/points_anonymized.csv" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "/app/data/matches.csv" \
  --candidate-adoption-siblings off \
  --candidate-adoption-nearby off \
  --verbose
```

```
# Combined candidate search (with CA)
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  --tracks "/app/data/points_anonymized.csv" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "/app/data/matches.csv" \
  --candidate-search combined \
  --verbose
```

```
# Combined (no CA)
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  --tracks "/app/data/points_anonymized.csv" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "/app/data/matches.csv" \
  --candidate-search combined \
  --candidate-adoption-siblings off \
  --candidate-adoption-nearby off \
  --verbose
```

```
# Combined single threading (no CA)
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  --tracks "/app/data/points_anonymized.csv" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "/app/data/matches.csv" \
  --candidate-search combined \
  --candidate-adoption-siblings off \
  --candidate-adoption-nearby off \
  --single-threading \
  --verbose
```

```
# Combined single threading cartesian (no CA)
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest-cartesian.dat" \
  --network-srs "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs" \
  --tracks "/app/data/points_anonymized.csv" \
  --tracks-transform-srs "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "/app/data/matches.csv" \
  --candidate-search combined \
  --candidate-adoption-siblings off \
  --candidate-adoption-nearby off \
  --single-threading \
  --verbose
```

</details>

We can see how reducing the amount of candidates and thus combinations reduces the CPU and memory load. The software
benefits a lot from multiple CPU cores, but needs more memory in its peak when multiple tracks are matched in parallel.
With single-threading and the complex candidate settings disabled, the software runs also well on a notebook. The major
RAM usage comes from the prepared graph then. Still, itscales well with desktop computers, workstations and
evendedicated servers. Concerning the accuracy, we are currently preparing another benchmark. At this moment, we can
tell that even with combined candidate search with candidate adoption disabled, the results are still good.

The cartesian variants are faster but slightly less accurate, the sanitation is also slightly different because the
distances are not calculated on the earth geodesic but on a cartesian coordinate system. Moreover, the adaptive circular
search has a smaller radius, because `200 m` flight direction is shorter than `200 m` distance on a cartographic map,
see: [Why Are Great Circles the Shortest Flight Path?](https://gisgeography.com/great-circle-geodesic-line-shortest-flight-path/)
. As such, `200 m` in a cartesian coordinate system are less than `200 m` in a geographic coordinate system, or `200 m`
in a geographic coordinate system are more than `200 m` in a cartesian coordinate system, depending on the location of
the world. For k-nearest neighbors, there is no difference.

### Build

If you want to build the software yourself, consult the `Dockerfile` for instructions. \
You need a recent and up-to-date Linux, for example Ubuntu 20.04 LTS and newer or Debian 11 and newer. \
At least GCC 9.4 is required.

Install the following prerequisites:

```
sudo apt-get install -y build-essential cmake git
sudo apt-get install -y zlib1g-dev libbz2-dev libexpat1-dev
```

You can build the software with the following commands:

```
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=OFF -B build
cmake --build build --parallel $(nproc) --target install
```

Then the built software can be found in the newly created `run/bin` directory. There is nothing installed outside of
this directory (so nothing is installed system-wide), uninstalling is simply deleting the directory.

For building under Windows you need Ubuntu 20.04 LTS and newer or Debian 11 and newer for the Windows Subsystem for
Linux (WSL). Building is the same as under Linux then. Native build currently is unsupported due to several dependencies
currently not building natively under Windows.

### Examples

Using `.gpx` files is as simple as `.csv` files. As the GPX scheme is well-defined, no additional configuration is
needed, see here an example with multiple `.gpx` files and previously exported network:

```
./map_matching_2 \
  --network-load "/app/data/oberfranken-latest.dat" \
  --tracks "/app/data/record_1.gpx" \
  --tracks "/app/data/record_2.gpx" \
  --tracks "/app/data/record_3.gpx" \
  --output "/app/data/matches.csv" \
  --verbose
```

This tool has native support for the [map matching dataset](https://doi.org/10.5281/zenodo.57731) from Kubicka, M. et
al.

For example, for matching and comparing the first track from the data set with the provided hand corrected result track,
extract the zip file and use the following command:

```
./map_matching_2 \
  --nodes "/app/data/map-matching-dataset/00000000/00000000.nodes" \
  --arcs "/app/data/map-matching-dataset/00000000/00000000.arcs" \
  --tracks "/app/data/map-matching-dataset/00000000/00000000.track" \
  --no-header --no-id --delimiter $'\t' --x 0 --y 1 --time 2 --no-parse-time \
  --compare "/app/data/map-matching-dataset/00000000/00000000.route" \
  --compare-edges-list-mode \
  --export-edges on \
  --output "/app/data/map-matching-dataset/00000000/00000000.result.csv" \
  --compare-output "/app/data/map-matching-dataset/00000000/00000000.compared.csv" \
  --model value-iteration \
  --verbose
```

The `*.result.csv` contains the id, matching duration, original track as WKT, the prepared track after Douglas-Peucker
and median merge simplification steps as WKT, and the matched result as WKT. You can load the results into QGIS. If you
want to see the network, use `--export-simplified-network nodes.csv --export-simplified-network edges.csv` arguments. If
you want to see the candidates, use `--export-candidates "existing-folder"` which exports the candidates and the
policy (the chosen candidates) in seperate files into the given folder. If you want to use a different model, consult
the `--help`, for example `--model viterbi` uses HMMs with Viterbi algorithm.

The `*.compared.csv` file contains again the id, track, prepared and result WKT strings the result file also contains
and additionally the ground truth track that the matched track was compared against. Moreover, the exact error
fraction (see paper for formula), added and missed lengths as well as WKT lines are provided for easy view in QGIS. Use
import of delimited text file for importing the WKT lines from the resulting csv files.

For comparing all 100 tracks from the dataset in one command and receive the average accuracy, use the following
(longer-running) script (can be made faster by using `--candidate-search combined`):

<details>
<summary>Kubicka et al. map matching dataset benchmark command</summary>

```
#!/bin/bash
path=$(realpath "/app/data/map-matching-dataset/")
folders=$(find "$path" -mindepth 1 -maxdepth 1 -type d | sort)
files=$(echo "$folders" | wc -l)
acc=0.0
for d in $folders; do
  n="${d##*/}"; f="${d}/${n}"
  printf "Matching $n ... "
  output=$(./map_matching_2 --nodes "${f}.nodes" --arcs "${f}.arcs" --tracks "${f}.track" \
    --no-header --no-id --delimiter $'\t' --x 0 --y 1 --time 2 --no-parse-time \
    --compare "${f}.route" --compare-edges-list-mode \
    --output "/app/data/kubicka_match.csv" --compare-output "/app/data/kubicka_compare.csv" \
    --verbose | grep "Correct Fraction:")
  echo "$output"
  res=($(echo "$output" | grep -Po '(\d+(\.\d+)?)'))
  acc=$(echo "$acc + ${res[0]}" | bc -l)
done
acc=$(echo "$acc / $files" | bc -l)
LC_NUMERIC="en_US.UTF-8" printf "\nMean accuracy: %.8f\n" $acc
```

</details>

With this command, a mean accuracy of approximately 98.0% is reached. We want to note that the map matching dataset form
Kubkicka et al. is not a ground truth dataset. However, it is the most diverse dataset.

This tool also has native support for
the [ground truth Seattle dataset](https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/)
from Newson and Krumm.

For matching the given track with the provided road network, use the text files, put them in the `/app/data/seattle/`
folder and use the following base command:

```
./map_matching_2 \
  --seattle /app/data/seattle/road_network.txt \
  --seattle-fix-connections on \
  --tracks /app/data/seattle/gps_data.txt \
  --delimiter $'\t' --no-id --x "Longitude" --y "Latitude" --no-parse-time \
  --compare /app/data/seattle/ground_truth_route.txt \
  --seattle-ground-truth-mode \
  --export-edges on \
  --compare-output /app/data/seattle_compare.csv \
  --output /app/data/seattle_match.csv \
  --verbose
```

With this command, an accuracy of approximately 99.6 % can be reached.
The road network has some connection issues, the `--seattle-fix-connections` argument is a workaround with side-effects
that tries to resolve these issues. Read the `help.txt` for more information or try out the command without this
parameter and review the comparison output in QGIS.

Moreover, this tool has native support for
the [ground truth Melbourne dataset](https://people.eng.unimelb.edu.au/henli/projects/map-matching/) from Hengfeng.

For matching the given track with the provided road network, use the text files, put them in `/app/data/seattle/` and
uncompress the network zip file into the subfolder `complete-osm-map`, then use the following base command:

```
./map_matching_2 \
  --melbourne-vertex /app/data/melbourne/complete-osm-map/vertex.txt \
  --melbourne-streets /app/data/melbourne/complete-osm-map/streets.txt \
  --melbourne-edges /app/data/melbourne/complete-osm-map/edges.txt \
  --tracks /app/data/melbourne/gps_track.txt \
  --delimiter ' ' --skip-lines 1 --no-id --no-header --x 2 --y 1 --no-parse-time
  --compare /app/data/melbourne/groundtruth.txt \
  --compare-edges-list-mode --compare-skip-lines 1 \
  --export-edges on \
  --compare-output /app/data/melbourne_compare.csv \
  --output /app/data/melbourne_match.csv \
  --verbose
```

With this command, an accuracy of approximately 99.8 % can be reached.

The following five examples come from the [map matching dataset](https://doi.org/10.5281/zenodo.57731) from Kubicka, M.
et al. All following examples were matched with our Markov Decision Process with Value Iteration and default parameter
settings, see `help.txt` or `./map_matching_2 --help`.

#### Example 1

![Example 1](docs/example-1.png "Example 1")
The original track is in red, the candidate policy projections are in orange, the matched result is in green and in gray
are the provided edges from the arcs-nodes network. You can see that there are fewer candidate projections than the
track has points (between each arrow there are two points). This comes from the track sanitation as unnecessary points
are removed before matching. In this example, the green match is perfectly matched in the circle concerning the
underlying road network in gray because it is slightly off the OpenStreetMap background image.

#### Example 2

![Example 2](docs/example-2.png "Example 2")
In this example a noise in the red track was correctly removed when taking a turn to the left. The noise might have come
from GPS point errors but clearly the red track does not exactly behave as expected at such a T-junction. With our
candidate-adoption approach, the algorithm was able to match multiple adjacent candidates to the same network edges,
which removed the noisy part in the green matched result.

#### Example 3

![Example 3](docs/example-3.png "Example 3")
Here the red track that comes from the north first goes to the south, then comes back later and then goes around a
residential district. It looks like the driver went off the car, walked around and later returned to the car. Though
this situation created very many GPS measurements and noise, our algorithm found a well fitting solution to this
situation on the underlying road network in the green matched result without any large detours. Median merge for the
point clouds as well as candidate adoption were key points for this solution.

#### Example 4

![Example 4](docs/example-4.png "Example 4")
In this extreme case, a detour around a park was made where no road network was provided in the underlying data. Our
nearby candidate adoption feature allowed the detour to project onto the same road position, so again no driving around
the noisy situation happened. The given road network did not allow the detour, as it can be seen that no gray lines lie
aside of the red track in the park area. Our algorithm was able to remove the detour from the match so that it fits well
to the provided road network.

#### Example 5

![Example 5](docs/example-5.png "Example 5")
Another example of a noisy part in the original red track that was completely removed by our track sanitation and
candidate adoption features that allowed our stochastic MDP process to find an overall optimal solution concerning its
defined metrics. The green match goes straight through without turning around multiple times in the high error
situation.

The following five examples come from the `points_anonymized.csv` matched with `oberfranken-latest.osm.pbf` as described
above in this readme.

#### Example 6

![Example 6](docs/example-6.png "Example 6")
Here is an example of the original candidates in light blue that our algorithm chose from. The chosen candidates are in
orange. We can see how the candidate adoption feature works, each road position is mapped to multiple track positions.
This enables the stochastic process to eliminate noisy parts as shown in the examples above by mapping multiple GPS
positions to the same road candidate positions. In this example however there was no such noise.

#### Example 7

![Example 7](docs/example-7.png "Example 7")
This is the same example as above but with the light blue candidates removed, only the orange selection remains. We can
see that the red track is not always matched to the nearest road edge position but to the position that fits best
concerning the metrics. In our paper, we describe this as typical road behavior. Currently our definition of typical
road behavior already leads to fine results as we can see in the benchmarks in our paper. Still there is room for
improvement, as usually with stochastic methods. In this case, we can all agree that the green matched result is
perfectly representing the red track.

#### Example 8

![Example 8](docs/example-8.png "Example 8")
This is the same example as above, just zoomed in to the positions where not the nearest road edge candidate was chosen
but the ones that fit best in the overall result. We can see that the chosen candidate depends on the overall best
route. If we only had chosen the best candidates for the track part that goes north (so only a local optimum), we might
have ended up in the road directly north to the green match. As the later position is clearly on the orange road, the
combination of the selected road positions is globally best in this way (global optimum).

#### Example 9

![Example 9](docs/example-9.png "Example 9")
Here is an example of a gap in the result. Not the track has the gap but the result was automatically split in the
Markov Decision Process. The track was recorded at a time when the bridge over the railways was open. The current
OpenStreetMap data has the bridge closed, so the track as seen is currently not possible to drive. We know this is true
because this is our hometown. Our algorithm (Value Iteration) decided not to route around the impossible situation but
to split the matching result at the closed bridge. Overall, this leads to a smaller error than driving around this
location far away, which people have to do in reality currently. For the old track this match is definitely fine, though
not perfect. Our implementation of the Viterbi algorithm with Hidden Markov Models does not allow this dynamic split to
happen because the Hidden Markov Model is a static model precomputed before the Viterbi algorithm solves it. The Markov
Decision Process however allows for dynamic action selection during the optimization, this is why it can decide to
introduce a gap (which means that it leads to new states at optimization time which the Viterbi algorithm cannot do)
when it notices that large detours are needed in a specific situation. It should be possible to detect and precompute
such situations for Hidden Markov Models as well but as our Markov Decision Process intrinsically enables such dynamic
solutions as we described, we did not implement it in our Hidden Markov Model.

#### Example 10

![Example 10](docs/example-10.png "Example 10")
This is the same example as above but the closed bridge is zoomed in. We can see that the green match result was split a
bit earlier than only around the closed part of the bridge. This is a result that can be discussed. We can see that the
orange candidate policy projections lie in a difficult situation. With the point points (at the red track arrows)
it is definitely difficult to choose optimal candidates in this situation. Though this result is not perfect, it is
quite good compared to a result that would make a large detour in this situation.

Of course the examples only show a fraction of what our open source software is able to do. More research in situations
that don't work this well already is going on. Please also review the paper for more extensive benchmarks, comparisons,
and explanations.

### References

*This is the successor to the deprecated repository [Map Matching](https://github.com/iisys-hof/map-matching).*

If you want to cite this work, please see the official publication: \
**Open source map matching with Markov decision processes: A new method and a detailed benchmark with existing
approaches** (https://doi.org/10.1111/tgis.13107)
