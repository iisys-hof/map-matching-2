#!/usr/bin/env bash

PROGRAM="./run/gcc/release/bin/map_matching_2"

run_test() {
    local folder_name="$1"
    local accuracy="$2"
    local cleanup="$3"

    "$PROGRAM" --conf "data/$folder_name/conf/prepare.conf" --memory-mapped-preparation off > /dev/null
    "$PROGRAM" --conf "data/$folder_name/conf/match.conf" > /dev/null
    output="$("$PROGRAM" --conf "data/$folder_name/conf/compare.conf")"

    if [[ $output =~ weighted[[:space:]]mean[[:space:]]correct[[:space:]]fraction:[[:space:]]*([0-9.]+) ]]; then
        local value="${BASH_REMATCH[1]}"

        if awk "BEGIN { exit !($value >= $accuracy) }"; then
            echo "$folder_name $value >= $accuracy PASS"
        else
            echo "$folder_name $value >= $accuracy FAIL"
        fi
    else
        echo "ERROR: Could not parse output"
    fi

    if [[ "$cleanup" == "true" && -d "data/$folder_name/network" ]]; then
        rm -rf "data/$folder_name/network"
    fi
}

run_test "floating-car-data" 0.99701 true
run_test "gis-cup" 0.98593 true
run_test "hengfeng-li" 0.99803 true
run_test "kubicka-et-al" 0.98736 true
run_test "newson-krumm" 0.99889 true
