#!/usr/bin/env python3
"""Get a survex survey (survey.svx) into an atlas: cavern turns it into survey.3d, which holds the station
positions, and the legs are read from the .svx itself."""
import subprocess
from pathlib import Path
from typing import Dict, List, Optional

from atlas import Atlas, Survey, SurveyLeg, SurveyStation
from survey_morph import read_survey_stations

SURVEY_SVX = "survey.svx"
SURVEY_3D = "survey.3d"
SURVEY_ID = 0 # key in Atlas.surveys that the survey file is stored under


def run_cavern(svx: Path, out_3d: Path) -> None:
    """Compute the survey with cavern, writing out_3d"""
    result = subprocess.run(["cavern", "-q", "-o", str(out_3d), str(svx)], capture_output=True, text=True)
    if result.returncode != 0 or not out_3d.exists():
        raise RuntimeError(f"cavern failed on {svx}: {(result.stdout + result.stderr).strip()}")


def ensure_survey_3d(model_dir: Path) -> Optional[Path]:
    """survey.3d in model_dir, made (or remade, if survey.svx is newer) from survey.svx with cavern.
    Returns None if there is neither a .svx nor a .3d. If cavern fails an existing .3d is still used"""
    svx, d3 = model_dir / SURVEY_SVX, model_dir / SURVEY_3D
    if svx.exists() and (not d3.exists() or d3.stat().st_mtime < svx.stat().st_mtime):
        try:
            run_cavern(svx, d3)
            print(f"Made {d3} from {svx}")
        except (RuntimeError, OSError) as e:
            print(e)
    return d3 if d3.exists() else None


def parse_svx_legs(path: Path) -> List[SurveyLeg]:
    """Legs of a survex file, with stations named as in the .3d (full lower case label, e.g. "garden.d").
    Only understands the common case: *data normal with from/to/tape/compass/clino, and *begin/*end for prefixes"""
    default_order = ["from", "to", "tape", "compass", "clino"]
    order = default_order
    normal = True
    prefix: List[str] = []
    legs: List[SurveyLeg] = []

    def label(name: str) -> str:
        return ".".join(prefix + [name]).lower()

    for line in path.read_text(errors="replace").splitlines():
        line = line.split(";", 1)[0].strip()
        if not line:
            continue
        tokens = line.split()
        if line.startswith("*"):
            command = tokens[0].lower()
            if command == "*begin":
                prefix.append(tokens[1] if len(tokens) > 1 else "")
            elif command == "*end":
                if prefix:
                    prefix.pop()
            elif command == "*data":
                normal = len(tokens) > 1 and tokens[1].lower() == "normal"
                if normal:
                    order = [x.lower() for x in tokens[2:]] or default_order
            continue
        if not normal or not all(x in order for x in ("from", "to", "tape", "compass", "clino")):
            continue
        if len(tokens) < max(order.index(x) for x in ("from", "to", "tape", "compass", "clino")) + 1:
            continue # trailing columns such as ignoreall may be left out
        data = dict(zip(order, tokens))
        try:
            leg = SurveyLeg(label(data["from"]), label(data["to"]), float(data["compass"]),
                            float(data["clino"]), float(data["tape"]))
        except ValueError: # "-" for a missing reading, for example
            continue
        legs.append(leg)
    return legs


def add_survey_to_atlas(atlas: Atlas, svx: Path, survey_3d: Optional[Path]) -> Survey:
    """Store the survey in the atlas: stations at their positions in the .3d, and the legs of the .svx"""
    stations: Dict[str, SurveyStation] = {}
    if survey_3d is not None and survey_3d.exists():
        stations = {name: SurveyStation(name, tuple(pos.tolist())) for name, pos in read_survey_stations(survey_3d).items()}
    survey = Survey(stations=stations, legs=parse_svx_legs(svx))
    atlas.surveys[SURVEY_ID] = survey
    print(f"Added survey to atlas: {len(survey.legs)} legs, {len(stations)} stations")
    return survey
