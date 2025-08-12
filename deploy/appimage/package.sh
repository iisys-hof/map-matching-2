#!/bin/bash
# this file is for creating the AppImage distribution package

# check if commands exists
if ! command -v wget &> /dev/null; then
    echo "wget is missing."
fi
if ! command -v grep &> /dev/null; then
    echo "grep is missing."
fi
if ! command -v docker &> /dev/null; then
    echo "docker is missing."
fi
if ! command -v patchelf &> /dev/null; then
    echo "patchelf is missing."
fi

# change to script dir
currentdir="$(realpath .)"
pwd="$(dirname "$0")"
cd "$pwd" || exit 1

# delete old AppDir if exists
appdir="AppDir"
if [ -d "$appdir" ]; then
  rm -r "$appdir"
fi

# build AppDir
dir="../../"
if [ ! -d "$dir" ]; then
  echo "Directory $dir does not exist."
  exit 1
fi

cd "$dir" || exit 1

# get version string
version=$(grep -Pzo '(?s)project\(map_matching_2.*?VERSION \K\d+(\.\d+)+' CMakeLists.txt | tr -d '\0')
if [[ ! "$version" =~ ^[0-9]+(\.[0-9]+)+$ ]]; then
  echo "Version invalid: $version"
  exit 1
fi

dockerfile="docker/build/gcc/15-ubuntu-2004/Dockerfile"
if [ ! -e "$dockerfile" ]; then
  echo "Dockerfile $dockerfile does not exist."
  exit 1
fi

if ! docker images -q "map_matching_2-gcc-ubuntu-2004-build" > /dev/null 2>&1; then
  docker build -t map_matching_2-gcc-ubuntu-2004-build -f "$dockerfile" . || exit 1
fi

dockerfile="docker/appimage/Dockerfile"
if [ ! -e "$dockerfile" ]; then
  echo "Dockerfile $dockerfile does not exist."
  exit 1
fi

DOCKER_BUILDKIT=1 docker build -t map_matching_2-appimage -f "$dockerfile" --output "$pwd" . || exit 1
cd "$pwd" || exit 1

if [ ! -d "$appdir" ]; then
  echo "Directory $appdir does not exist."
  exit 1
fi


if [ ! -e "$appdir/usr/bin/map_matching_2" ]; then
  echo "Directory $appdir/usr/bin/map_matching_2 does not exist."
  exit 1
fi

# prepare AppDir
if [ -d "$appdir/usr/lib/x86_64-linux-gnu/" ]; then
  patchelf --set-rpath "\$ORIGIN/../lib/x86_64-linux-gnu/" "$appdir/usr/bin/map_matching_2"
fi
cd "$appdir" && ln -s "usr/bin/map_matching_2" "AppRun" && cd ".." || exit 1
cp -p data/map_matching_2.desktop "$appdir" || exit 1
cp -p data/map_matching_2.svg "$appdir" || exit 1

# download appimagetool
appimagetool="appimagetool-x86_64.AppImage"
if [ ! -e "$appimagetool" ]; then
  wget -O "$appimagetool" "https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage" || exit 1
  chmod +x "$appimagetool"
fi

# build appimage
VERSION="$version" "./$appimagetool" "$appdir" || exit 1
appimage=$(ls -rt map_matching_2*.AppImage | tail -n 1)
chmod +x "$appimage"

# clean up
if [ -d "$appdir" ]; then
  rm -r "$appdir"
fi

printf "\n"
echo "Built AppImage $(realpath --relative-to="$currentdir" "$appimage")"
