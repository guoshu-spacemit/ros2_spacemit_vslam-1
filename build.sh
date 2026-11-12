#!/usr/bin/env bash
# SDK custom-package entry; the SDK already owns the install lock here.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PYTHON_BIN=${SROBOTIS_PYTHON:-python3}
if ! "$PYTHON_BIN" -c 'import sys; sys.exit(sys.version_info < (3, 6))' >/dev/null 2>&1; then
    printf 'spacemit-vslam: cannot run Python 3.6 or later using %s.\n' "$PYTHON_BIN" >&2
    printf 'Set SROBOTIS_PYTHON to a compatible Python executable.\n' >&2
    exit 2
fi
exec "$PYTHON_BIN" "$project_dir/tools/build.py" --sdk-entry "$@"
