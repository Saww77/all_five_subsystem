#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -f "$build/core" "$build/controller" "$build/motion"; rmdir "$build"' EXIT
CC=${CC:-cc}
CFLAGS="-std=c11 -Wall -Wextra -Werror -O2 -ffunction-sections -fdata-sections -Iapp_program"
$CC $CFLAGS testing/host/core_checks.c app_program/subsystem1/routes.c \
    app_program/subsystem1/telemetry.c app_program/subsystem3/barcode.c \
    app_program/subsystem4/hump.c app_program/subsystem4/imu_safety.c app_program/subsystem5/profile.c \
    -Wl,--gc-sections -o "$build/core"
$CC $CFLAGS testing/host/controller_checks.c app_program/subsystem1/routes.c \
    -Wl,--gc-sections -o "$build/controller"
$CC $CFLAGS testing/host/motion_checks.c -Wl,--gc-sections -o "$build/motion"
"$build/core"
"$build/controller"
"$build/motion"
