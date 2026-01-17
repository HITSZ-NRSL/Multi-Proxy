#!/usr/bin/env bash
# docker_stats_recorder.sh
# Usage: ./docker_stats_recorder.sh output.csv [interval_seconds]
# Default interval = 1s

OUT=${1:-docker_stats.csv}
INTERVAL=${2:-1}

# Header: timestamp (sec), container_id, name, cpu_perc, mem_usage, mem_perc, net_io, block_io, pids
echo "timestamp,container_id,name,cpu_perc,mem_usage,mem_perc,net_io,block_io,pids" > "$OUT"

while true; do
  ts=$(date +%s.%N)   # high-precision timestamp
  # docker stats --no-stream prints one snapshot; format fields as CSV-compatible strings
  docker stats --no-stream --format '{{.Container}},{{.Name}},{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}},{{.NetIO}},{{.BlockIO}},{{.PIDs}}' \
    | sed "s/^/$ts,/" >> "$OUT"
  sleep "$INTERVAL"
done