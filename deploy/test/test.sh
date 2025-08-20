#!/bin/bash

if [ -z "$1" ]; then
  echo "Usage: $0 <appimage_file>"
  exit 1
fi

# define distros to test
distros=(
    "ubuntu:20.04:Ubuntu 20.04"
    "ubuntu:22.04:Ubuntu 22.04"
    "ubuntu:24.04:Ubuntu 24.04"
    "debian:11:Debian 11"
    "debian:12:Debian 12"
    "debian:13:Debian 13"
)

# check if commands exists
if ! command -v docker &> /dev/null; then
    echo "docker is missing."
    exit 1
fi

appimage="$1"
appimage=$(realpath "$appimage")
appimage_dir=$(dirname "$appimage")
appimage_name=$(basename "$appimage")

echo "AppImage: $appimage"
echo "AppImage dir: $appimage_dir"
echo "AppImage name: $appimage_name"

if [ ! -f "$appimage" ]; then
    echo "Error: '$appimage' not found"
    exit 1
fi

# extract version from filename (assumes format: toolname-version-arch.AppImage)
filename_version=$(echo "$appimage_name" | sed -n 's/.*-\([0-9]\+\.[0-9]\+\.[0-9]\+\)-.*/\1/p')
if [ -z "$filename_version" ]; then
    echo "Warning: Could not extract version from filename '$appimage_name'"
else
    echo "Expected version from filename: $filename_version"
fi

echo "========================================="
echo "Testing AppImage on multiple distros"
echo "========================================="

failed_distros=()
passed_distros=()

for distro_info in "${distros[@]}"; do
    IFS=':' read -r base_image version display_name <<< "$distro_info"
    full_base_image="${base_image}:${version}"
    image_tag="appimage-runner-$(echo "$base_image-$version" | tr ':' '-')"
    
    echo
    echo "--- Testing on $display_name ---"
    echo "Building image: $image_tag (base: $full_base_image)"
    
    # build the Docker image
    if ! docker build --build-arg BASE_IMAGE="$full_base_image" -t "$image_tag" -f Dockerfile . >/dev/null 2>&1; then
        echo "✗ $display_name: Failed to build Docker image"
        failed_distros+=("$display_name")
        continue
    fi
    
    # run the AppImage in the container
    output=$(docker run --rm \
        -v "$appimage_dir":/app \
        --device /dev/fuse \
        --cap-add SYS_ADMIN \
        --security-opt apparmor:unconfined \
        "$image_tag" \
        /bin/bash -c "/app/$appimage_name --version" 2>&1)
    EXIT_CODE=$?
    
    echo "Output from $display_name:"
    echo "$output"
    
    if [ $EXIT_CODE -ne 1 ]; then
        echo "✗ $display_name: AppImage --version execution failed with exit code $EXIT_CODE"
        failed_distros+=("$display_name")
        continue
    fi
    
    # check if version matches (if we could extract it from filename)
    if [ -n "$filename_version" ]; then
        if echo "$output" | grep -q "$filename_version"; then
            echo "✓ $display_name: Version check passed (found $filename_version)"
            passed_distros+=("$display_name")
        else
            echo "✗ $display_name: Version check failed (expected $filename_version not found)"
            failed_distros+=("$display_name")
        fi
    else
        echo "✓ $display_name: AppImage executed successfully (no version check)"
        passed_distros+=("$display_name")
    fi
done

echo
echo "========================================="
echo "SUMMARY"
echo "========================================="

if [ ${#passed_distros[@]} -gt 0 ]; then
    echo "✓ PASSED (${#passed_distros[@]}):"
    for distro in "${passed_distros[@]}"; do
        echo "  - $distro"
    done
fi

if [ ${#failed_distros[@]} -gt 0 ]; then
    echo "✗ FAILED (${#failed_distros[@]}):"
    for distro in "${failed_distros[@]}"; do
        echo "  - $distro"
    done
    echo
    echo "Overall result: FAILED"
    exit 1
fi

echo
echo "Overall result: ALL TESTS PASSED"

# optional: clean up built images
echo
read -p "Clean up built Docker images? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Cleaning up images..."
    for distro_info in "${distros[@]}"; do
        IFS=':' read -r base_image version display_name <<< "$distro_info"
        image_tag="appimage-runner-$(echo "$base_image-$version" | tr ':' '-')"
        docker rmi "$image_tag" >/dev/null 2>&1 && echo "  Removed: $image_tag"
    done
fi

exit 0
