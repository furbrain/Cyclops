#!/usr/bin/env python3
import argparse
import pathlib
import orb_slam3_py as op
parser = argparse.ArgumentParser(description="Convert binary osa to text file and vice versa")
parser.add_argument('-a', '--atlas', help="provide an atlas message file to use", required=True)
parser.add_argument('-o', '--output', help="provide an output file to use")
opts = parser.parse_args()

print(f"Loading {opts.atlas}")
if opts.atlas.endswith(".osa"):
    atlas = op.load_atlas(opts.atlas, binary=True)
    if opts.output:
        output = opts.output
    else:
        output = str(pathlib.Path(opts.atlas).with_suffix(".txt.gz"))
    print(f"Saving as {output}")
    op.save_atlas(atlas, output, binary=False)
else:
    atlas = op.load_atlas(opts.atlas, binary=False)
    if opts.output:
        output = opts.output
    else:
        output = str(pathlib.Path(opts.atlas).with_suffix(".osa"))
        output = output.replace(".txt.osa", ".osa")
    print(f"Saving as {output}")
    op.save_atlas(atlas, output, binary=True)
