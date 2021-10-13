# Map Matching 2 [Work-in-progress]

**Map Matching based on Markov Decision Processes (MDPs) and Hidden Markov Models (HMMs).**

Currently the following models are implemented:

* Markov Decision Process (MDP) for single tracks.
* Hidden Markov Model (HMM) for single tracks.

Currently the following algorithms are implemented:

* for Hidden Markov Models
    * Viterbi algorithm
* for Markov Decision Processes
    * Policy Iteration
    * Value Iteration
    * Q-learning

Various advanced map matching algorithms are implemented, for example:

* OpenStreetMap `.osm.pbf` import of road network
* `.csv` import of tracks, either as list of points or WKT lines or multilines
* Import in shell pipeline mode of WKT lines or multilines (optional)
* Native import and matching of the map matching dataset
  at [Dataset for testing and training map-matching methods](https://zenodo.org/record/57731) (optional)
* Export of imported map data in various processing stages for easy view in [QGIS](https://www.qgis.org/)
* Comparison methods for matching results of own results and results of external software (optional)
* Lossless Network simplification for reducing nodes and edges
* Reprojections between geographical and cartesian spatial reference systems (SRSs) and matching in both SRSs (optional)
* Upper bound Dijkstra algorithm for routing
* Fat initialized segments and lines for edges (e.g., network edges, candidate edges)
* Removing of weakly unconnected subgraphs (optional)
* Caches for length and bearing calculations
* Caches for routes from single-source Dijkstra algorithm
* Track sanitation with Douglas-Peucker algorithm
* Removing of spatially duplicate points
* Merging of point clouds, with adaptive distance when the next road is far away
* Candidate search based on adaptive circle radius or k-nearest neighbor
* Candidate adoption of siblings and/or nearby candidates
* Matching of tracks with gaps with MDP and skip-errors option (default)
* Tunable weights for HMM and MDP models
* Index mapping for MDP model and algorithms for rewards, V and Q tables
* Tunable parameters for all algorithms, such as learning rate, discount factor, epsilon, etc.
* Output to `.csv` file for easy view in [QGIS](https://www.qgis.org/) or optional console for shell pipeline mode
* Quiet mode and verbose mode with benchmark timings (optional)
* Multi-threading which can optionally be disabled
* Improved memory management with [rpmalloc](https://github.com/mjansson/rpmalloc)
* *More features are in the making ...*

The provided source code is heavily tuned and optimized for fast and accurate map matching. \
Please read the
paper [Evolving map matching with markov decision processes](https://opus4.kobv.de/opus4-hof/frontdoor/index/index/docId/119)
for more information about this open source tool. \
We are currently working on several new research papers based on this approach.

Build docker image: \
`docker build -t map-matching-2 .`

Run docker container: \
`docker run --rm -it -v $(pwd)/data:/app/data -u $(id -u ${USER}):$(id -g ${USER}) --name map-matching-2 map-matching-2`

View Map Matching Help (or view [help.txt](help.txt)): \
`./map_matching_2 --help`

Example (be aware, uses as much CPU cores as available, even scales with 128 cores, and in this example needs at least
64 GB of available system memory):

```
./map_matching_2 \
  --network "data/oberfranken-latest.osm.pbf" \
  --tracks "data/points_anonymized.csv" \
  --delimiter ";" \
  --id "device" --id "subid" \
  --x "lon" --y "lat" \
  --time "timestamp" --time-format "%F %T%Oz" \
  --output "data/matches.csv" \
  --verbose
```

If you don't have that much RAM, disable candidate adoption with \
`--candidate-adoption-siblings off --candidate-adoption-nearby off` \
but be aware that this reduces the map matching accuracy drastically. \
Works with about 16 GB of available system memory then, maybe less, depending on the amount of CPU cores available.

If you don't want to use multi-threading for map matching enable \
`--single-threading`. \
This also reduces global memory usage as parallel matching does not occur.

*This is the successor to the deprecated repository [Map Matching](https://github.com/iisys-hof/map-matching).*
