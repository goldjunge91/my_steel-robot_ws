#!/bin/bash

start_date="2025-09-19"
end_date="2025-10-15"
current_date="$start_date"

while [[ "$current_date" < "$end_date" ]] || [[ "$current_date" == "$end_date" ]]; do
    echo "Checking runs for $current_date"
    runs=$(gh run list --created $current_date --json databaseId --jq '.[].databaseId')
    if [[ -n "$runs" ]]; then
        echo "Found runs for $current_date, deleting..."
        echo "$runs" | xargs -I {} gh run delete {}
    else
        echo "No runs found for $current_date"
    fi
    current_date=$(date -j -v+1d -f "%Y-%m-%d" "$current_date" +%Y-%m-%d)
done

echo "Done deleting runs from $start_date to $end_date"