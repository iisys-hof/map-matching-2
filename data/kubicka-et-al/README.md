### Dataset for testing and training of map-matching algorithms

M. Kubicka, A. Cela, P. Moulin, H. Mounier, and S. I. Niculescu, "Dataset for testing and training of map-matching
algorithms", in 2015 IEEE Intelligent Vehicles Symposium (IV), Jun. 2015,
doi: [10.1109/ivs.2015.7225829](https://dx.doi.org/10.1109%2Fivs.2015.7225829)

Worldwide map matching dataset to download from: \
[https://doi.org/10.5281/zenodo.57731](https://doi.org/10.5281/zenodo.57731)

Unzip the `map-matching-dataset.zip` zip file and place in this directory \
the `map-matching-dataset` folder that contains everything.

The data set is hand-corrected and thus contains some matching errors.

The following commands need to be run from the base `map_matching_2`, \
the path to the `map_matching_2` executable can be adjusted as needed, e.g., \
for GCC: `run/gcc/release/bin/map_matching_2`, \
for Clang: `run/clang/release/bin/map_matching_2`, \
for MSVC: `run\msvc\release\bin\map_matching_2.exe`.

```
# prepare networks
./map_matching_2 --conf data/kubicka-et-al/conf/prepare.conf

# prepare ground truth networks (not simplified!)
./map_matching_2 --conf data/kubicka-et-al/conf/prepare_ground_truth.conf

# convert ground truth routes
./map_matching_2 --conf data/kubicka-et-al/conf/ground_truth.conf

# match tracks
./map_matching_2 --conf data/kubicka-et-al/conf/match.conf

# match tracks with export-edges mode (uses the slower ground truth network!)
./map_matching_2 --conf data/kubicka-et-al/conf/match_edges.conf

# compare matches with ground truth
./map_matching_2 --conf data/kubicka-et-al/conf/compare.conf

# compare export-edges matches with ground truth
./map_matching_2 --conf data/kubicka-et-al/conf/compare_edges.conf
```

The results will be in the `results` folder that is automatically created.

Results on our test system (accuracy is the weighted mean correct fraction in percent):

| Mode                     | Time (s) | Max RAM (MiB) | Accuracy (%) |
|:-------------------------|---------:|--------------:|-------------:|
| Prepare                  |    32.80 |         2,712 |          N/A |
| Prepare *                |    16.05 |         1,760 |          N/A |
| Prepare (Ground Truth)   |    26.50 |         2,639 |          N/A |
| Prepare (Ground Truth) * |    20.17 |         2,553 |          N/A |
| Convert Ground Truth     |     7.69 |         1,285 |          N/A |
| Match                    |     8.20 |         1,570 |          N/A |
| Match (Export-Edges)     |    31.27 |         4,486 |          N/A |
| Compare                  |     0.84 |           210 |        98.73 |
| Compare (Export-Edges)   |     0.83 |           193 |        98.93 |

Max RAM contains shared memory (cached memory backed by disk, not actually used).\
The prepare runs marked with * were run with `--memory-mapped-preparation off`.
In this case, only the final data is written into memory mapped files and during the preparation,
non-cached RAM (actually used by the process) is used.
