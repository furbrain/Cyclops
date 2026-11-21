#!/usr/bin/env python3
"""Write survey.json for the model viewer from a survex .3d file: the endpoints of every underground leg,
in pairs (points 0-1 are one leg, 2-3 the next, and so on)"""
import json
import re
import subprocess
from pathlib import Path
from typing import List

# dump3d prints each traverse as a MOVE to its start then a LINE to each following station
COORDS_RE = re.compile(r"^(MOVE|LINE)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)(.*)$")
# leg flags dump3d prints after the label: these legs aren't part of the cave's centreline
SKIP_FLAGS = {"SPLAY", "SURFACE", "DUPLICATE"}


def parse_dump3d_legs(text: str) -> List[List[float]]:
    points: List[List[float]] = []
    last = None
    for line in text.splitlines():
        m = COORDS_RE.match(line)
        if not m:
            continue
        pos = [float(m.group(i)) for i in (2, 3, 4)]
        if m.group(1) == "LINE" and last is not None and not SKIP_FLAGS & set(m.group(5).split()):
            points += [last, pos]
        last = pos
    return points


def survey_legs(survey_3d: Path) -> List[List[float]]:
    out = subprocess.run(["dump3d", str(survey_3d)], capture_output=True, text=True, check=True).stdout
    return parse_dump3d_legs(out)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("survey_3d", type=Path, help="survex .3d file to read")
    parser.add_argument("output", type=Path, help="json file to write")
    opts = parser.parse_args()
    points = survey_legs(opts.survey_3d)
    opts.output.write_text(json.dumps({"points": points}))
    print(f"Wrote {len(points) // 2} legs to {opts.output}")
