### Hidden Markov Map Matching Through Noise and Sparseness

P. Newson and J. Krumm, "Hidden Markov Map Matching through Noise and Sparseness", in Proceedings of the 17th ACM
SIGSPATIAL International Conference on Advances in Geographic Information Systems, New York, NY, USA: Association for
Computing Machinery, 2009, pp. 336–343, doi: [10.1145/1653771.1653818](https://dx.doi.org/10.1145%2F1653771.1653818).

Ground truth dataset to download from: \
[https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/](https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/)

Place in this directory \
the `gps_data.txt` text file, \
the `ground_truth_route.txt` text file, and \
unzip the `road_network.zip` zip file so that \
the `road_network.txt` text file is in this folder.

The following commands need to be run from the base `map_matching_2`, \
the path to the `map_matching_2` executable can be adjusted as needed, e.g., \
for GCC: `run/gcc/release/bin/map_matching_2`, \
for Clang: `run/clang/release/bin/map_matching_2`, \
for MSVC: `run\msvc\release\bin\map_matching_2.exe`.

```
# prepare network
./map_matching_2 --conf data/newson-krumm/conf/prepare.conf

# convert ground truth route
./map_matching_2 --conf data/newson-krumm/conf/ground_truth.conf

# match tracks
./map_matching_2 --conf data/newson-krumm/conf/match.conf

# compare matches with ground truth
./map_matching_2 --conf data/newson-krumm/conf/compare.conf
```

The results will be in the `results` folder that is automatically created.

Results on our test system (accuracy is the weighted mean correct fraction in percent):

| Mode                 | Time (s) | Max RAM (MiB) | Accuracy (%) |
|:---------------------|---------:|--------------:|-------------:|
| Prepare              |     4.02 |           254 |          N/A |
| Convert Ground Truth |     1.32 |            50 |          N/A |
| Match                |     2.09 |            98 |          N/A |
| Compare              |     0.51 |            22 |        99.89 |
