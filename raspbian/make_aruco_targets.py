#!/usr/bin/env python3
"""Generate laser-cuttable ArUco targets as SVG for import into LightBurn.

Markers use the same dictionary as aruco.py (DICT_4X4_50), where marker id N is
named by the capital letter chr(ord('@') + N), i.e. A=1 ... Z=26.

Each target is a rectangular plate with
  * two 2.5mm mounting holes in the top corners   (cut layer,    black)
  * the marker pattern (black cells of the code)  (engrave layer, blue)
  * the capital letter below the marker           (engrave layer, red)

LightBurn maps SVG colours to layers on import, so black = cut, blue = marker
fill, red = label fill. Set the blue/red layers to "Fill" mode. Text is
converted to outlines, so no fonts are needed on the laser PC.

Examples:
  make_aruco_targets.py single C                    -> aruco_C.svg
  make_aruco_targets.py grid                        -> aruco_grid.svg (A-P and R-U, skipping Q; 4 columns x 5 rows)
  make_aruco_targets.py grid --first D --cols 2 --rows 3
"""
import argparse
from typing import List

import cv2
import numpy as np
from matplotlib.font_manager import FontProperties
from matplotlib.path import Path as MplPath
from matplotlib.textpath import TextPath

CUT = "#000000"
ENGRAVE_MARKER = "#0000FF"
ENGRAVE_LABEL = "#FF0000"

BORDER_BITS = 1
MARKER_BITS = 4
CELLS = MARKER_BITS + 2 * BORDER_BITS  # 6x6 cells including the black border


# Q is skipped: its pattern is just a single square, which is too easily falsely detected
# I is skipped: the label could be confused for numeral 1
SKIPPED = "IQ"
ALPHABET = [chr(c) for c in range(ord("A"), ord("Z") + 1) if chr(c) not in SKIPPED]


def letter_to_id(letter: str) -> int:
    letter = letter.upper()
    if letter in SKIPPED:
        raise ValueError(f"marker {letter} is not used (too easily falsely detected)")
    if len(letter) != 1 or not "A" <= letter <= "Z":
        raise ValueError(f"marker name must be a single capital letter, got {letter!r}")
    return ord(letter) - ord("@")


def marker_cells(marker_id: int) -> np.ndarray:
    """6x6 boolean array, True where the cell is black (including the border)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, CELLS * 10, borderBits=BORDER_BITS)
    # sample the centre of each cell
    centres = np.arange(CELLS) * 10 + 5
    return img[np.ix_(centres, centres)] < 128


def f(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".")


def rect_path(x: float, y: float, w: float, h: float) -> str:
    return f"M{f(x)} {f(y)}H{f(x + w)}V{f(y + h)}H{f(x)}Z"


def circle_path(cx: float, cy: float, r: float) -> str:
    # two arcs so it is a plain path in every importer
    return f"M{f(cx - r)} {f(cy)}A{f(r)} {f(r)} 0 1 0 {f(cx + r)} {f(cy)}A{f(r)} {f(r)} 0 1 0 {f(cx - r)} {f(cy)}Z"


def marker_path(cells: np.ndarray, x0: float, y0: float, cell: float) -> str:
    """Black cells as rectangles, merging horizontal runs."""
    parts = []
    for r, row in enumerate(cells):
        c = 0
        while c < len(row):
            if row[c]:
                start = c
                while c < len(row) and row[c]:
                    c += 1
                parts.append(rect_path(x0 + start * cell, y0 + r * cell, (c - start) * cell, cell))
            else:
                c += 1
    return "".join(parts)


def text_path(text: str, cx: float, top: float, height: float) -> str:
    """Outline of `text` (bold sans), horizontally centred on cx, cap-height `height`, top edge at `top` (mm)."""
    tp = TextPath((0, 0), text, size=100, prop=FontProperties(family="DejaVu Sans", weight="bold"))
    verts = tp.vertices
    xmin, ymin = verts.min(axis=0)
    xmax, ymax = verts.max(axis=0)
    scale = height / (ymax - ymin)
    d = []
    i = 0
    codes, vs = tp.codes, tp.vertices

    def pt(v):
        # font y is up, SVG y is down
        return f"{f(cx + (v[0] - (xmin + xmax) / 2) * scale)} {f(top + (ymax - v[1]) * scale)}"

    while i < len(codes):
        c = codes[i]
        if c == MplPath.MOVETO:
            d.append(f"M{pt(vs[i])}")
            i += 1
        elif c == MplPath.LINETO:
            d.append(f"L{pt(vs[i])}")
            i += 1
        elif c == MplPath.CURVE3:
            d.append(f"Q{pt(vs[i])} {pt(vs[i + 1])}")
            i += 2
        elif c == MplPath.CURVE4:
            d.append(f"C{pt(vs[i])} {pt(vs[i + 1])} {pt(vs[i + 2])}")
            i += 3
        elif c == MplPath.CLOSEPOLY:
            d.append("Z")
            i += 1
        else:
            raise ValueError(f"unexpected path code {c}")
    return "".join(d)


def target_elements(letter: str, ox: float, oy: float, size: float, pad: float, pad_top: float, label_h: float,
                    hole_d: float):
    """SVG elements for one plate whose top-left corner is at (ox, oy).
    The plate outline is not included (see outline_path); returns (elements, width, height)."""
    cell = size / CELLS
    width = size + 2 * pad
    height = pad_top + size + label_h
    hole_inset = min(pad, pad_top) / 2
    cut = circle_path(ox + hole_inset, oy + hole_inset, hole_d / 2)
    cut += circle_path(ox + width - hole_inset, oy + hole_inset, hole_d / 2)

    cells = marker_cells(letter_to_id(letter))
    marker = marker_path(cells, ox + pad, oy + pad_top, cell)
    # label is centred in the space between the marker and the bottom edge
    text_height = min(label_h * 0.6, 20.0)
    label = text_path(letter, ox + width / 2, oy + pad_top + size + (label_h - text_height) / 2, text_height)
    return [
        f'<path id="cut_{letter}" d="{cut}" fill="none" stroke="{CUT}" stroke-width="0.1"/>',
        f'<path id="marker_{letter}" d="{marker}" fill="{ENGRAVE_MARKER}" fill-rule="nonzero" stroke="{ENGRAVE_MARKER}" stroke-width="0.01"/>',
        f'<path id="label_{letter}" d="{label}" fill="{ENGRAVE_LABEL}" fill-rule="evenodd" stroke="{ENGRAVE_LABEL}" stroke-width="0.01"/>',
    ], width, height


def outline_path(n: int, cols: int, plate_w: float, plate_h: float) -> str:
    """Cut lines for n touching plates laid out in rows of `cols`. Each shared edge is drawn once and
    collinear segments are joined into single lines."""
    horiz, vert = set(), set()  # (line index, cell index)
    for i in range(n):
        r, c = divmod(i, cols)
        horiz.update({(r, c), (r + 1, c)})
        vert.update({(c, r), (c + 1, r)})

    def runs(edges):
        out = []
        for line in sorted({e[0] for e in edges}):
            cells = sorted(e[1] for e in edges if e[0] == line)
            start = prev = cells[0]
            for k in cells[1:] + [None]:
                if k is None or k != prev + 1:
                    out.append((line, start, prev + 1))
                    if k is not None:
                        start = k
                if k is not None:
                    prev = k
        return out

    d = []
    for line, a, b in runs(horiz):
        d.append(f"M{f(a * plate_w)} {f(line * plate_h)}H{f(b * plate_w)}")
    for line, a, b in runs(vert):
        d.append(f"M{f(line * plate_w)} {f(a * plate_h)}V{f(b * plate_h)}")
    return "".join(d)


def write_svg(filename: str, letters: List[str], cols: int, size: float, pad: float, pad_top: float, label_h: float,
              hole_d: float):
    rows = -(-len(letters) // cols)
    elements = []
    plate_w = size + 2 * pad
    plate_h = pad_top + size + label_h
    for i, letter in enumerate(letters):
        r, c = divmod(i, cols)
        els, _, _ = target_elements(letter, c * plate_w, r * plate_h, size, pad, pad_top, label_h, hole_d)
        elements.extend(els)
    elements.append(f'<path id="cut_outline" d="{outline_path(len(letters), cols, plate_w, plate_h)}" '
                    f'fill="none" stroke="{CUT}" stroke-width="0.1"/>')
    total_w = min(cols, len(letters)) * plate_w
    total_h = rows * plate_h
    svg = (f'<?xml version="1.0" encoding="UTF-8"?>\n'
           f'<svg xmlns="http://www.w3.org/2000/svg" width="{f(total_w)}mm" height="{f(total_h)}mm" '
           f'viewBox="0 0 {f(total_w)} {f(total_h)}">\n  ' + "\n  ".join(elements) + "\n</svg>\n")
    with open(filename, "w") as fh:
        fh.write(svg)
    print(f"wrote {filename}: {len(letters)} target(s), {f(total_w)} x {f(total_h)} mm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("single", help="one marker")
    s.add_argument("letter", help="capital letter A-Z (marker id = position in alphabet)")
    g = sub.add_parser("grid", help="a grid of consecutive markers")
    g.add_argument("--first", default="A", help="first letter (default A)")
    g.add_argument("--cols", type=int, default=4)
    g.add_argument("--rows", type=int, default=5)
    for p in (s, g):
        p.add_argument("-o", "--output", help="output SVG filename")
        p.add_argument("--size", type=float, default=84.0, help="marker side in mm including its black border (default 84)")
        p.add_argument("--pad", type=float, default=8.0, help="white margin left/right of the marker in mm (default 8)")
        p.add_argument("--pad-top", type=float, default=8.0, help="white margin above the marker in mm (default 8)")
        p.add_argument("--label-height", type=float, default=8.0, help="margin under the marker, holding the label, in mm (default 8)")
        p.add_argument("--hole", type=float, default=2.5, help="hole diameter in mm")
    a = ap.parse_args()

    if a.mode == "single":
        letters = [a.letter.upper()]
        cols = 1
        out = a.output or f"aruco_{letters[0]}.svg"
    else:
        n = a.cols * a.rows
        available = [l for l in ALPHABET if l >= a.first.upper()]
        if len(available) < n:
            ap.error(f"{n} markers starting at {a.first.upper()} runs past Z")
        letters = available[:n]
        cols = a.cols
        out = a.output or "aruco_grid.svg"
    for l in letters:
        letter_to_id(l)
    write_svg(out, letters, cols, a.size, a.pad, a.pad_top, a.label_height, a.hole)


if __name__ == "__main__":
    main()
