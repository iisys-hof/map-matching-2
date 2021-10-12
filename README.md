# Map-Matching 2

Build docker image: \
`docker build -t map-matching-2 .`

Run docker container: \
`docker run --rm -it -v $(pwd)/data:/app/data -u $(id -u ${USER}):$(id -g ${USER}) --name map-matching-2 map-matching-2`

View Map Matching Help: \
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
