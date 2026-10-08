#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/../.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
python3 - "$build" <<'PY'
from pathlib import Path
import sys
s = Path('menu.cpp').read_text()
p = Path(sys.argv[1])
start = s.index('enum MENU\n')
end = s.index('\n};', start) + 3
(p / 'menu_enum.inc').write_text(s[start:end] + '\n')
start = s.index('\tcase MENU_FALCON_MAIN1:\n')
end = s.index('\tcase MENU_ST_MAIN1:\n', start)
(p / 'menu_cases.inc').write_text(s[start:end])
PY
mkdir "$build/storage"
${CXX:-g++} -std=c++14 -Wall -Wextra -Werror -Wno-format-truncation \
  -D_FILE_OFFSET_BITS=64 -D_LARGEFILE64_SOURCE -fsanitize=address,undefined -g \
  -I. -I"$build" tests/falcon_config/test.cpp support/falcon/falcon_config.cpp \
  -o "$build/test"
"$build/test" "$build/storage"
