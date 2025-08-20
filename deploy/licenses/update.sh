#!/bin/bash
# this file is for updating the license information in the external folder from the current release build

# check if commands exists
if ! command -v docker &> /dev/null; then
    echo "docker is missing."
fi

# change to script dir
currentdir="$(realpath .)"
pwd="$(dirname "$0")"
cd "$pwd" || exit 1

# build app
dir="../../"
if [ ! -d "$dir" ]; then
  echo "Directory $dir does not exist."
  exit 1
fi

cd "$dir" || exit 1
dockerfile="docker/build/gcc/15-ubuntu-2204/Dockerfile"
if [ ! -e "$dockerfile" ]; then
  echo "Dockerfile $dockerfile does not exist."
  exit 1
fi

if ! docker images -q "map_matching_2-gcc-ubuntu-2204-build" > /dev/null 2>&1; then
  docker build -t map_matching_2-gcc-ubuntu-2204-build -f "$dockerfile" . || exit 1
fi
docker run -u $(id -u):$(id -g) -v .:/tmp/map-matching-2 -it --rm map_matching_2-gcc-ubuntu-2204-build \
  /bin/bash -c "cmake --build /tmp/map-matching-2/cmake-build-release --target install -j $(nproc)"

# update licenses
directories=("boost" "bz2" "csv_parser" "date" "expat" "gpx" "libosmium" "protozero" "rpmalloc" "zlib")
for dir in "${directories[@]}"; do
  cp -p "run/gcc/release/doc/${dir}/"* "external/${dir}/"
done

printf "\n"
echo "Updated license information."
