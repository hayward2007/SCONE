#!/usr/bin/env bash
# Build one manuscript. Prefers tectonic (single, hermetic pass); falls back to
# the local TeX Live xelatex + bibtex chain so the build works offline.
set -euo pipefail
MAIN="${1:?usage: build.sh <root.tex without extension>}"
OUTDIR="${2:-build}"
mkdir -p "$OUTDIR"
# Honour a user texmf tree (kotex/xeCJK support packages may live there).
export TEXMFHOME="${TEXMFHOME:-$HOME/texmf}"

if command -v tectonic >/dev/null 2>&1; then
  tectonic --keep-logs --keep-intermediates --outdir "$OUTDIR" "$MAIN.tex"
  exit 0
fi

XL=(xelatex -interaction=nonstopmode -halt-on-error -file-line-error
    -output-directory="$OUTDIR")
"${XL[@]}" "$MAIN.tex" >/dev/null
(cd "$OUTDIR" && BIBINPUTS=..: BSTINPUTS=..: bibtex "$MAIN" >/dev/null) || true
"${XL[@]}" "$MAIN.tex" >/dev/null
"${XL[@]}" "$MAIN.tex" >/dev/null
