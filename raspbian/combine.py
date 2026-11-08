#!/usr/bin/env python3
from dataclasses import dataclass
from pathlib import Path
from itertools import combinations
from typing import List, Optional, Tuple


from atlas import Atlas, SubAtlas, SUB_ATLAS_FILE
from merge_maps import InternalConnections, Connection, Link, CONNECTIONS_FILE
from survey_morph import morph_to_survey, parse_prefixes, bundle_adjust as bundle_adjust_atlas


ATLAS_CONNECTIONS_FILE = "atlas_connections.csv"
PRE_MERGE_ATLAS_FILE = "atlas_pre_merge.json.gz"


@dataclass
class SurveyLink:
    from_survey: str
    from_scope: Link
    to_survey: str
    to_scope: Link

    @classmethod
    def from_line(cls, text: str) -> "SurveyLink":
        fields = [x.strip() for x in text.split(',')]
        return cls(fields[0], Link.from_string(fields[1]), fields[2], Link.from_string(fields[3]))


@dataclass
class SurveyLinks:
    links: List[SurveyLink]

    @classmethod
    def from__file(cls, path: Path):
        with open(path) as f:
            return cls([SurveyLink.from_line(line) for line in f])

    def get_pre_merge_atlas(self, dir: Path, share_survey: bool = False) -> Tuple[Atlas, SubAtlas]:
        subatlas_names = {x.from_survey for x in self.links}
        subatlas_names |= {x.to_survey for x in self.links}
        subatlas_names = list(sorted(subatlas_names))
        sub_atlas = SubAtlas()
        main_atlas = Atlas(maps={}, map_points={}, cameras={})
        main_atlas.dir = dir
        for sub in subatlas_names:
            new_atlas = Atlas.from_dir(dir / sub)
            pre_maps = set(main_atlas.maps.keys())
            kf_offset = main_atlas.import_atlas(new_atlas, share_survey, survex_prefix=sub)
            post_maps = set(main_atlas.maps.keys())
            sub_atlas.sub_atlases[sub] = SubAtlas.from_path(dir / sub, kf_offset, post_maps - pre_maps)
        return main_atlas, sub_atlas

    def combine(self, atlas: Atlas, sub_atlas: SubAtlas) -> InternalConnections:
        connections = []
        for link in self.links:
            sub_a = sub_atlas.sub_atlases[link.from_survey]
            sub_b = sub_atlas.sub_atlases[link.to_survey]
            for map_a in (atlas.maps[x] for x in sub_a.maps):
                for map_b in (atlas.maps[x] for x in sub_b.maps):
                    connection = Connection(map_a.id, map_b.id)
                    connection.compare_maps(map_a, map_b, link.from_scope, link.to_scope)
                    if connection.is_valid():
                        connections.append(connection)
        return InternalConnections(connections, atlas)

def find_unlinked_connections(links: SurveyLinks, atlas: Atlas, sub_atlas: SubAtlas,
                              existing: List[Connection]) -> List[Connection]:
    """Look for connections between every pair of sub-atlases that isn't named in the links file, as
    sub-atlases can overlap in space without having been recorded one after the other"""
    linked = {frozenset((x.from_survey, x.to_survey)) for x in links.links}
    have = {frozenset((x.from_map, x.to_map)) for x in existing}
    found = []
    for (name_a, sub_a), (name_b, sub_b) in combinations(sorted(sub_atlas.sub_atlases.items()), 2):
        if frozenset((name_a, name_b)) in linked:
            continue
        for map_a in (atlas.maps[x] for x in sorted(sub_a.maps)):
            for map_b in (atlas.maps[x] for x in sorted(sub_b.maps)):
                if frozenset((map_a.id, map_b.id)) in have:
                    continue
                connection = Connection(map_a.id, map_b.id)
                connection.compare_maps(map_a, map_b)
                if connection.is_valid():
                    print(f"Found extra connection {name_a} <-> {name_b}: {len(connection.keypoint_matches)} matches")
                    found.append(connection)
    return found


def combine_atlases(model_dir: Path, share_survey: bool = False, survey_3d: Optional[Path] = None,
                    survey_prefixes: Optional[List[str]] = None, bundle_adjust: bool = True,
                    all_links: bool = False):
    links = SurveyLinks.from__file(model_dir / ATLAS_CONNECTIONS_FILE)
    if (model_dir / PRE_MERGE_ATLAS_FILE).exists() and (model_dir / SUB_ATLAS_FILE).exists():
        atlas = Atlas.from_json_file(model_dir / PRE_MERGE_ATLAS_FILE)
        with open(model_dir / SUB_ATLAS_FILE) as f:
            subatlasses = SubAtlas.from_json(f.read())
    else:
        atlas, subatlasses = links.get_pre_merge_atlas(model_dir, share_survey=share_survey)
        atlas.save(model_dir / PRE_MERGE_ATLAS_FILE)
        with open(model_dir / SUB_ATLAS_FILE, "w") as f:
            f.write(subatlasses.to_json(indent=2))
    if (model_dir / CONNECTIONS_FILE).exists():
        with open(model_dir / CONNECTIONS_FILE) as f:
            connections = InternalConnections.from_json(f.read())
        connections.atlas = atlas
    else:
        connections = links.combine(atlas, subatlasses)
    if all_links:
        connections.connections.extend(find_unlinked_connections(links, atlas, subatlasses, connections.connections))
    with open(model_dir / CONNECTIONS_FILE, "w") as f:
        f.write(connections.to_json(indent=2))
    connections.split_connections(distance=2.0)
    for conn in connections.connections:
        print(f"Connection: {conn.from_map} <-> {conn.to_map}, "
              f"matches: {len(conn.keypoint_matches)}, "
              f"ransac_transform: {conn.transform}") 
    connections.clean_connections()
    connections.get_ransac_transforms(translation_only=True)
    connections.apply_transform()
    if survey_3d is not None:
        prefixes = parse_prefixes(survey_prefixes or [], list(subatlasses.sub_atlases))
    else:
        prefixes = None
    survey_fit = morph_to_survey(atlas, subatlasses.sub_atlases, connections.connections, survey_3d, prefixes)
    connections.unify_maps()
    if bundle_adjust:
        bundle_adjust_atlas(atlas, survey_fit)
    # no atlas.optimize() here: it currently distorts the sub-surveys by up to several metres
    atlas.save(model_dir / "atlas.json.gz")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="combine several sub-surveys into one atlas")
    parser.add_argument('-d', '--dir', help="combined model directory, containing "
                        f"{ATLAS_CONNECTIONS_FILE} and a sub-directory per sub-survey", default=".")
    parser.add_argument('-s', '--share-survey', help="all sub-surveys share a single survey",
                        action="store_true")
    parser.add_argument('--survey', help="survex .3d file: aruco markers are fixed to its stations and the "
                        "sub-surveys are morphed to fit (needs survex's dump3d on the path)")
    parser.add_argument('--survey-prefix', action='append', default=[], metavar='[NAME=]PREFIX',
                        help="survex prefix the aruco markers are under, e.g. 'garden' for station garden.a. "
                        "Applies to every sub-survey, or just sub-survey NAME if given as NAME=PREFIX. "
                        "May be repeated. Default is each sub-survey's own name")
    parser.add_argument('--all-links', action='store_true',
                        help="also look for connections between sub-surveys that are not linked in "
                        f"{ATLAS_CONNECTIONS_FILE}, in case they overlap")
    parser.add_argument('--no-bundle', action='store_true',
                        help="with --survey, skip the final bundle adjustment of the combined atlas")
    opts = parser.parse_args()
    combine_atlases(Path(opts.dir), share_survey=opts.share_survey,
                    survey_3d=Path(opts.survey) if opts.survey else None,
                    survey_prefixes=opts.survey_prefix, bundle_adjust=not opts.no_bundle, all_links=opts.all_links)