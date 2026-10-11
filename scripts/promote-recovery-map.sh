#!/usr/bin/env bash
set -euo pipefail

MAP_DIRECTORY="${MAP_DIRECTORY:-/maps}"
MAP_NAME="${MAP_NAME:?MAP_NAME is required}"
MARKER="$MAP_DIRECTORY/.${MAP_NAME}.recovery-latest"

if [ -f "$MAP_DIRECTORY/$MAP_NAME.yaml" ] && \
        [ -f "$MAP_DIRECTORY/${MAP_NAME}_serial.posegraph" ]; then
    exit 0
fi
if [ ! -f "$MARKER" ]; then
    echo "ERROR: No complete recovery map is available." >&2
    exit 1
fi

recovery_name="$(cat "$MARKER")"
case "$recovery_name" in
    ".${MAP_NAME}.recovery-a"|".${MAP_NAME}.recovery-b") ;;
    *)
        echo "ERROR: The recovery marker contains an invalid map name." >&2
        exit 1
        ;;
esac

for path in \
        "$MAP_DIRECTORY/$recovery_name.yaml" \
        "$MAP_DIRECTORY/$recovery_name.pgm" \
        "$MAP_DIRECTORY/${recovery_name}_serial.posegraph"; do
    if [ ! -f "$path" ]; then
        echo "ERROR: The recovery map is incomplete: $path" >&2
        exit 1
    fi
done

temporary_prefix="$MAP_DIRECTORY/.${MAP_NAME}.promote.$$"
trap 'rm -f "${temporary_prefix}"*' EXIT

cp "$MAP_DIRECTORY/$recovery_name.pgm" "${temporary_prefix}.pgm"
sed "s|^image:.*$|image: ${MAP_NAME}.pgm|" \
    "$MAP_DIRECTORY/$recovery_name.yaml" >"${temporary_prefix}.yaml"
cp "$MAP_DIRECTORY/${recovery_name}_serial.posegraph" \
    "${temporary_prefix}_serial.posegraph"
if [ -f "$MAP_DIRECTORY/${recovery_name}_serial.data" ]; then
    cp "$MAP_DIRECTORY/${recovery_name}_serial.data" \
        "${temporary_prefix}_serial.data"
fi

mv "${temporary_prefix}.pgm" "$MAP_DIRECTORY/$MAP_NAME.pgm"
mv "${temporary_prefix}_serial.posegraph" \
    "$MAP_DIRECTORY/${MAP_NAME}_serial.posegraph"
if [ -f "${temporary_prefix}_serial.data" ]; then
    mv "${temporary_prefix}_serial.data" \
        "$MAP_DIRECTORY/${MAP_NAME}_serial.data"
fi
mv "${temporary_prefix}.yaml" "$MAP_DIRECTORY/$MAP_NAME.yaml"

echo "Recovered map: $MAP_DIRECTORY/$MAP_NAME.yaml"
echo "Recovered pose graph: $MAP_DIRECTORY/${MAP_NAME}_serial.posegraph"
