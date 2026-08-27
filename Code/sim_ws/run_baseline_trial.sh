#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

SCENARIO="${1:-constant_speed}"
DURATION="${2:-60}"
RUN_ID="${3:-$(date -u +%Y%m%dT%H%M%SZ)}"
MAP_NAME="${4:-Melbourne_map}"

case "$SCENARIO" in
  constant_speed|accel_brake|turning|stop_go) ;;
  *)
    echo "Unknown scenario: $SCENARIO" >&2
    echo "Use constant_speed, accel_brake, turning, or stop_go." >&2
    exit 2
    ;;
esac

if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Duration must be a positive number of seconds." >&2
  exit 2
fi

mkdir -p results
set +e
timeout --signal=INT --kill-after=10s "${DURATION}s" \
  ./gym.sh \
  scenario:="$SCENARIO" \
  run_id:="$RUN_ID" \
  results_root:="$PWD/results" \
  map_name:="$MAP_NAME"
STATUS=$?
set -e

if [ "$STATUS" -ne 0 ] && [ "$STATUS" -ne 124 ] && [ "$STATUS" -ne 130 ]; then
  exit "$STATUS"
fi

SUMMARY="$PWD/results/$SCENARIO/$RUN_ID/summary.json"
if [ ! -f "$SUMMARY" ]; then
  echo "Trial ended without a summary: $SUMMARY" >&2
  exit 1
fi
echo "Trial complete: $SUMMARY"
