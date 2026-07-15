#!/usr/bin/env bash
#
# Format all C/C++ sources in-tree with clang-format (uses ./.clang-format).
#
# Usage:
#   scripts/format.sh            # format in place
#   scripts/format.sh --check    # report files that need formatting; non-zero exit if any (for CI)
#
# Override the binary if clang-format is not on PATH:
#   CLANG_FORMAT=clang-format-21 scripts/format.sh
#
set -euo pipefail

# --- config -----------------------------------------------------------------
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLANG_FORMAT="${CLANG_FORMAT:-clang-format}"

# Directories that hold our sources. third_party/ and build/ are never touched.
SEARCH_DIRS=(cpp apps)

# File extensions to format.
EXTENSIONS=(c cc cxx cpp h hh hxx hpp)

# --- args -------------------------------------------------------------------
CHECK=0
case "${1:-}" in
  --check) CHECK=1 ;;
  "")      ;;
  *)       echo "unknown argument: $1" >&2; exit 2 ;;
esac

# --- collect files ----------------------------------------------------------
if ! command -v "$CLANG_FORMAT" >/dev/null 2>&1; then
  echo "error: '$CLANG_FORMAT' not found on PATH" >&2
  exit 1
fi

cd "$REPO_ROOT"

# Build a find expression: ( -name '*.c' -o -name '*.cc' ... )
name_expr=()
for ext in "${EXTENSIONS[@]}"; do
  name_expr+=(-o -name "*.${ext}")
done
name_expr=("${name_expr[@]:1}")  # drop leading -o

mapfile -d '' files < <(
  find "${SEARCH_DIRS[@]}" \
    \( -path '*/build' -o -path '*/third_party' \) -prune -o \
    -type f \( "${name_expr[@]}" \) -print0
)

if [[ ${#files[@]} -eq 0 ]]; then
  echo "no source files found under: ${SEARCH_DIRS[*]}"
  exit 0
fi

# --- run --------------------------------------------------------------------
if [[ $CHECK -eq 1 ]]; then
  bad=0
  for f in "${files[@]}"; do
    if ! "$CLANG_FORMAT" "$f" | diff -q "$f" - >/dev/null; then
      echo "needs formatting: $f"
      bad=1
    fi
  done
  if [[ $bad -eq 0 ]]; then
    echo "all ${#files[@]} files are properly formatted"
  fi
  exit $bad
fi

"$CLANG_FORMAT" -i "${files[@]}"
echo "formatted ${#files[@]} files"
