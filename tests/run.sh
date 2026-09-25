#!/usr/bin/env sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$root/tests/parent_ack_test.c" -o "$work/parent_ack_test"
"$work/parent_ack_test"

cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$root/tests/status_probe_test.c" "$root/firmware/radio/src/status_probe.c" -o "$work/status_probe_test"
"$work/status_probe_test"
