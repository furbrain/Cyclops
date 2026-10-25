#!/usr/bin/env python3
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict, Optional, Tuple, Set

from dataclasses_json import DataClassJsonMixin

from atlas import Atlas
from merge_maps import InternalConnections, Connection, Link, CONNECTIONS_FILE

SUB_ATLAS_FILE = "subatlas.json"
ATLAS_CONNECTIONS_FILE = "atlas_connections.csv"
PRE_MERGE_ATLAS_FILE = "atlas_pre_merge.json.gz"


@dataclass
class SubAtlas(DataClassJsonMixin):
    kf_offset: int = 0
    maps: Set[int] = field(default_factory=set)
    sub_atlases: Dict[str, "SubAtlas"] = field(default_factory=dict)

    def get_kf_stem(self, kf_id: int) -> Path:
        offset_id = kf_id - self.kf_offset
        sorted_atlases = sorted(self.sub_atlases.items(),
                                key=lambda x: x[1].kf_offset,
                                reverse=True)
        if not self.sub_atlases:
            return Path("images") / f"{offset_id}"
        else:
            for name, atlas in sorted_atlases:
                if offset_id >= atlas.kf_offset:
                    return Path(name) / atlas.get_kf_stem(offset_id)
            raise ValueError(f"No matching subatlas found keyframe id: {offset_id}")

    @classmethod
    def from_path(cls, path: Path, key_offset: int = 0, maps: Optional[Set[int]] = None) -> "SubAtlas":
        if maps is None:
            maps = set()
        if (path / SUB_ATLAS_FILE).exists():
            with open(path / SUB_ATLAS_FILE) as f:
                sub_atlas = cls.from_json(f.read())
                sub_atlas.kf_offset = key_offset
                sub_atlas.maps = maps
                return sub_atlas
        else:
            return cls(key_offset, maps)

    def save(self, dir: Path):
        with open(dir / SUB_ATLAS_FILE, "w") as f:
            f.write(self.to_json(indent=2))


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

def combine_atlases(model_dir: Path, share_survey: bool = False):
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
    with open(model_dir / CONNECTIONS_FILE, "w") as f:
        f.write(connections.to_json(indent=2))
    connections.get_ransac_transforms(translation_only=True)
    connections.apply_transform()
    connections.unify_maps()
    atlas.optimize(include_survey=False)
    atlas.save(model_dir / "atlas.json.gz")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="combine several sub-surveys into one atlas")
    parser.add_argument('-d', '--dir', help="combined model directory, containing "
                        f"{ATLAS_CONNECTIONS_FILE} and a sub-directory per sub-survey", default=".")
    parser.add_argument('-s', '--share-survey', help="all sub-surveys share a single survey",
                        action="store_true")
    opts = parser.parse_args()
    combine_atlases(Path(opts.dir), share_survey=opts.share_survey)