### ACM SIGSPATIAL GIS Cup 2012

A. Mohamed, T. Rautman, J. Krumm, and A. Teredesai, "ACM SIGSPATIAL GIS Cup 2012", in ACM SIGSPATIAL GIS '12, November
6-9, 2012. Redondo Beach, CA, USA, Nov. 2012,
doi: [10.1145/2424321.2424426](https://dx.doi.org/10.1145%2F2424321.2424426).

GIS Cup 2012 map matching dataset to download from: \
[https://web.archive.org/web/20130127211936/http://depts.washington.edu/giscup/home](https://web.archive.org/web/20130127211936/http://depts.washington.edu/giscup/home) \
Click on the `Road Network Information` and `Training Data Sets` links.

Unzip the `RoadNetworkData.zip` and place in this directory \
the `WA_Nodes.txt` text file, \
the `WA_Edges.txt` text file, and \
the `WA_EdgeGeometry.txt` text file, \
unzip the `GIS_Contest_Training_Data.zip` zip file so that \
the `GisContestTrainingData` folder containing the `input` and `output` folders is this folder.

The "ground truth" routes `output_02.txt` and `output_07.txt` contain large errors, and as such the matching accuracy is
low although the match is correct in the respective sections. \
To correct the largest errors in the "ground truth", do the following: \
From `output_02.txt` delete the lines from id `701` to `760` with the wrong edge id `1263704` between the lines with the
correct edge id `180859` so that only that one remains without gaps. \
From `output_07.txt` delete the lines from id `873` to `932` with the wrong edge id `1263704` between the lines with the
correct edge id `180859` so that only that one remains without gaps. \
You can examine the issue for yourself when you match and compare the original results before the fix in QGIS and
examine the `error_misses` from the comparison. After the fix, the erroneous situation is correct.

There are, unfortunately, more "ground truth" errors in the data set, as you can see if you carefully inspect yourself,
but these were by far the largest ones.

The following commands need to be run from the base `map_matching_2`, \
the path to the `map_matching_2` executable can be adjusted as needed, e.g., \
for GCC: `run/gcc/release/bin/map_matching_2`, \
for Clang: `run/clang/release/bin/map_matching_2`, \
for MSVC: `run\msvc\release\bin\map_matching_2.exe`.

```
# prepare network
./map_matching_2 --conf data/gis-cup/conf/prepare.conf

# convert ground truth route
./map_matching_2 --conf data/gis-cup/conf/ground_truth.conf

# match tracks
./map_matching_2 --conf data/gis-cup/conf/match.conf

# compare matches with ground truth
./map_matching_2 --conf data/gis-cup/conf/compare.conf
```

The results will be in the `results` folder that is automatically created.

Results (with ground truth corrections from above applied) on our test system
(accuracy is the weighted mean correct fraction in percent):

| Mode                 | Time (s) | Max RAM (MiB) | Accuracy (%) |
|:---------------------|---------:|--------------:|-------------:|
| Prepare              |    20.63 |         1,269 |          N/A |
| Prepare *            |    17.29 |         1,493 |          N/A |
| Convert Ground Truth |     0.42 |           308 |          N/A |
| Match                |     0.73 |           250 |          N/A |
| Compare              |     0.22 |            32 |        98.59 |

Max RAM contains shared memory (cached memory backed by disk, not actually used).\
The prepare runs marked with * were run with `--memory-mapped-preparation off`.
In this case, only the final data is written into memory mapped files and during the preparation,
non-cached RAM (actually used by the process) is used.
