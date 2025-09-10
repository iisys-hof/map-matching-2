### Spatio-temporal trajectory simplification for inferring travel paths

H. Li, L. Kulik, and K. Ramamohanarao, "Spatio-temporal trajectory simplification for inferring travel paths", in
Proceedings of the 22nd ACM SIGSPATIAL International Conference on Advances in Geographic Information Systems, Nov.
2014, pp. 63–72, doi: [10.1145/2666310.2666409](https://dx.doi.org/10.1145%2F2666310.2666409)

Melbourne ground truth dataset to download from: \
[https://web.archive.org/web/20170301001019/https://people.eng.unimelb.edu.au/henli/projects/map-matching/](https://web.archive.org/web/20170301001019/https://people.eng.unimelb.edu.au/henli/projects/map-matching/)

Place in this directory \
the `gps_track.txt` text file, \
the `groundtruth.txt` text file, \
unzip the `melbourne.osm.zip` zip file so that \
the `melbourne.osm` OSM file is in this folder, and \
unzip the `complete-osm-map.zip` zip so that \
the `complete-osm-map` folder is in this folder, which further contains \
the `edges.txt` text file, \
the `streets.txt` text file, and \
the `vertex.txt` text file.

The following commands need to be run from the base `map_matching_2`, \
the path to the `map_matching_2` executable can be adjusted as needed, e.g., \
for GCC: `run/gcc/release/bin/map_matching_2`, \
for Clang: `run/clang/release/bin/map_matching_2`, \
for MSVC: `run\msvc\release\bin\map_matching_2.exe`.

```
# prepare network
./map_matching_2 --conf data/hengfeng-li/conf/prepare.conf

# prepare osm network
./map_matching_2 --conf data/hengfeng-li/conf/prepare_osm.conf

# prepare ground truth network (not simplified!)
./map_matching_2 --conf data/hengfeng-li/conf/prepare_ground_truth.conf

# convert ground truth route
./map_matching_2 --conf data/hengfeng-li/conf/ground_truth.conf

# match tracks
./map_matching_2 --conf data/hengfeng-li/conf/match.conf

# match tracks on osm network
./map_matching_2 --conf data/hengfeng-li/conf/match_osm.conf

# compare matches with ground truth
./map_matching_2 --conf data/hengfeng-li/conf/compare.conf

# compare osm matches with ground truth
./map_matching_2 --conf data/hengfeng-li/conf/compare_osm.conf
```

The results will be in the `results` folder that is automatically created.

Results on our test system (accuracy is the weighted mean correct fraction in percent):

| Mode                     | Time (s) | Max RAM (MiB) | Accuracy (%) |
|:-------------------------|---------:|--------------:|-------------:|
| Prepare                  |     1.52 |           142 |          N/A |
| Prepare *                |     0.93 |           136 |          N/A |
| Prepare (OSM)            |     3.02 |           241 |          N/A |
| Prepare (OSM) *          |     2.24 |           229 |          N/A |
| Prepare (Ground Truth)   |     1.38 |           146 |          N/A |
| Prepare (Ground Truth) * |     1.09 |           170 |          N/A |
| Convert Ground Truth     |     0.14 |            78 |          N/A |
| Match                    |     0.42 |            62 |          N/A |
| Match (OSM)              |     0.75 |            74 |          N/A |
| Compare                  |     0.16 |            20 |        99.80 |
| Compare (OSM)            |     0.16 |            20 |        99.86 |

Max RAM contains shared memory (cached memory backed by disk, not actually used).\
The prepare runs marked with * were run with `--memory-mapped-preparation off`.
In this case, only the final data is written into memory mapped files and during the preparation,
non-cached RAM (actually used by the process) is used.
