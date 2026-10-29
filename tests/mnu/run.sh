#!/bin/bash
# Native tests for the menu module (.mnu) mechanism. Builds mnu_path.cpp,
# mnu.cpp, the example module and a few purpose-built modules with the host
# compiler and exercises them without MiSTer hardware.
#
#   tests/mnu/run.sh            build dir: tests/mnu/build
#   MNU_TEST_BUILD=/x run.sh    build somewhere else
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
BUILD=${MNU_TEST_BUILD:-$HERE/build}
CXX=${CXX:-g++}

CXXFLAGS="-std=gnu++14 -O1 -g -Wall -Wextra -D_FILE_OFFSET_BITS=64 -D_LARGEFILE64_SOURCE -I$ROOT -I$ROOT/lib/libchdr/include -I$HERE"
MODFLAGS="-shared -fPIC -fvisibility=hidden -fno-gnu-unique -Wl,--no-undefined"

rm -rf "$BUILD"
mkdir -p "$BUILD/fixture/_Console" "$BUILD/fixture/_Other" "$BUILD/fixture/_Arcade/cores" "$BUILD/fixture/games/NES"

echo "== mnu_path_test"
$CXX $CXXFLAGS -o "$BUILD/mnu_path_test" "$HERE/mnu_path_test.cpp" "$ROOT/mnu_path.cpp"
"$BUILD/mnu_path_test"

echo "== modules"
$CXX $CXXFLAGS $MODFLAGS -o "$BUILD/example.so" "$ROOT/mnu_template/example.cpp"
# (cd.h, through minimig_config.h: a C-linkage typedef the host compiler warns about)
# The Maximig module lives in its core's repository (menu/ folder).
MAXIMIG_MENU=${MAXIMIG_MENU:-$ROOT/../../Maximig-AGA_MiSTer/menu}
if [ -f "$MAXIMIG_MENU/maximig.cpp" ]; then
	$CXX $CXXFLAGS -Wno-non-c-typedef-for-linkage $MODFLAGS -I"$MAXIMIG_MENU/../hps" -o "$BUILD/maximig.so" "$MAXIMIG_MENU/maximig.cpp"
fi
for m in bad_noentry bad_abi bad_init short_desc poller; do
	$CXX $CXXFLAGS $MODFLAGS -o "$BUILD/$m.so" "$HERE/mod_$m.cpp"
done

# A module must export exactly one symbol.
for m in example maximig; do
	nm -D --defined-only "$BUILD/$m.so" | awk '{print $3}' | grep -v '^_' > "$BUILD/$m.syms" || true
	if [ "$(cat "$BUILD/$m.syms")" != "mnu_entry" ]; then
		echo "FAIL: $m.so exports: $(tr '\n' ' ' < "$BUILD/$m.syms")"
		exit 1
	fi
	echo "$m.so exports only mnu_entry"
done

echo "== host_sim_test"
$CXX $CXXFLAGS -o "$BUILD/host_sim_test" "$HERE/host_sim_test.cpp" "$HERE/stubs.cpp" "$ROOT/mnu.cpp" "$ROOT/mnu_path.cpp" -ldl
cp "$BUILD/example.so" "$BUILD/fixture/_Console/NES.mnu"
cp "$BUILD/example.so" "$BUILD/fixture/_Arcade/cores/Arkanoid.mnu"
cp "$BUILD/example.so" "$BUILD/fixture/menu.mnu"
: > "$BUILD/fixture/menu.rbf"
: > "$BUILD/fixture/_Console/NES_20261010.rbf"
: > "$BUILD/fixture/_Other/Nothing_20260101.rbf"
: > "$BUILD/fixture/_Arcade/cores/Arkanoid.rbf"
: > "$BUILD/fixture/_Arcade/Arkanoid.mra"
: > "$BUILD/fixture/games/NES/game.bin"
"$BUILD/host_sim_test" "$BUILD/fixture" "$BUILD"

if [ -f "$BUILD/maximig.so" ]; then
echo "== family_test"
$CXX $CXXFLAGS -Wno-non-c-typedef-for-linkage -o "$BUILD/family_test" "$HERE/family_test.cpp" "$HERE/stubs.cpp" "$ROOT/mnu.cpp" "$ROOT/mnu_path.cpp" -ldl
F="$BUILD/fixture_family"
mkdir -p "$F/_Console" "$F/_Computer" "$F/config"
cp "$BUILD/example.so" "$F/_Console/NES.mnu"
cp "$BUILD/maximig.so" "$F/_Computer/Maximig_a_aa3000.mnu"
cp "$BUILD/maximig.so" "$F/_Computer/Maximig.mnu"
: > "$F/_Console/NES_20261010.rbf"
: > "$F/_Computer/Maximig_a_aa3000_20261029.rbf"
: > "$F/_Computer/Maximig_20261025.rbf"
"$BUILD/family_test" "$F"
else
echo "== family_test skipped: no Maximig checkout at $MAXIMIG_MENU"
fi

echo "all menu module tests passed"
