#!/bin/bash
# this file is for updating the help.txt

# argument AppImage
if [ -z "$1" ]; then
  echo "Usage: $0 map_matching_2.AppImage"
  exit 1
fi

appimage="$1"
appimage=$(realpath "$appimage")
if [ ! -f "$appimage" ]; then
  echo "The file $appimage does not exist."
  exit 1
fi

# change to script dir
currentdir="$(realpath .)"
pwd="$(dirname "$0")"
cd "$pwd" || exit 1

# build help.txt
dir="../../"
if [ ! -d "$dir" ]; then
  echo "Directory $dir does not exist."
  exit 1
fi

cd "$dir" || exit 1

# create help.txt
echo "# ./map_matching_2 --help" > help.txt
"$appimage" --help >> help.txt
echo "# ./map_matching_2 --prepare --help" >> help.txt
"$appimage" --prepare --help >> help.txt
echo "# ./map_matching_2 --match --help" >> help.txt
"$appimage" --match --help >> help.txt
echo "# ./map_matching_2 --compare --help" >> help.txt
"$appimage" --compare --help >> help.txt

echo "Finished creating help.txt."
