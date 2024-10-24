### Open source map matching with Markov decision processes: A new method and a detailed benchmark with existing approaches

A. Wöltche, "Open source map matching with Markov decision processes: A new method and a detailed benchmark with
existing approaches", Transactions in GIS, vol. 27, no. 7, pp. 1959–1991, Oct. 2023, doi: 10.1111/tgis.13107

Float Car Data (FCD) map matching dataset to download from: \
[https://doi.org/10.6084/m9.figshare.19785835](https://doi.org/10.6084/m9.figshare.19785835)

Place in this directory from the `benchmarks.zip` zip file \
from the `benchmarks/docker/initialize/data/` folder \
the `oberfranken-latest.osm.pbf` OSM file, please see `odbl-10.txt` for licence information, \
the `points.csv` csv file, and \
the `ground_truth.csv` csv file.

Additionally, we provide the anonymized `points_anonymized.csv` raw FCD file with permission
from [Map and Route GmbH & Co. KG](https://www.mapandroute.de/) for further testing purposes. \
The `points.csv` file from the data set above is a subset for that we provide hand-corrected results with the
`ground_truth.csv` file.
You need the same OpenStreetMap data as above from the region of Oberfranken, Bavaria, Germany.

In case you want to use a more recent version than the one from above, you can download the needed extract for
remplacement at
[Geofabrik / Oberfranken](https://download.geofabrik.de/europe/germany/bayern/oberfranken.html).

The following commands need to be run from the base `map_matching_2`, \
the path to the `map_matching_2` executable can be adjusted as needed, e.g., \
for GCC: `run/gcc/release/bin/map_matching_2`, \
for Clang: `run/clang/release/bin/map_matching_2`, \
for MSVC: `run\msvc\release\bin\map_matching_2.exe`.

```
# prepare network
./map_matching_2 --conf data/floating-car-data/conf/prepare.conf

# match tracks
./map_matching_2 --conf data/floating-car-data/conf/match.conf

# compare matches with ground truth
./map_matching_2 --conf data/floating-car-data/conf/compare.conf
```

The provided raw FCD tracks can be matched with the following command:

```
# match raw tracks
./map_matching_2 --conf data/floating-car-data/conf/match_raw.conf
```

Also given is an example on how to use custom tags for OpenStreetMap, for example for foot ways.

```
# prepare network using all roads, i.e., for foot
./map_matching_2 --conf data/floating-car-data/conf/prepare_all.conf
```

The results will be in the `results` folder that is automatically created.

Results on our test system (accuracy is the weighted mean correct fraction in percent):

| Mode            | Time (s) | Max RAM (MiB) | Accuracy (%) |
|:----------------|---------:|--------------:|-------------:|
| Prepare         |    32.80 |         2,271 |          N/A |
| Prepare *       |    12.87 |         1,543 |          N/A |
| Match           |     0.66 |           422 |          N/A |
| Compare         |     0.59 |            38 |        99.58 |
| Match (RAW)     |     5.56 |         1,620 |          N/A |
| Prepare (All)   |    37.38 |         2,602 |          N/A |
| Prepare (All) * |    14.48 |         1,688 |          N/A |

Max RAM contains shared memory (cached memory backed by disk, not actually used).\
The prepare runs marked with * were run with `--memory-mapped-preparation off`.
In this case, only the final data is written into memory mapped files and during the preparation,
non-cached RAM (actually used by the process) is used.
