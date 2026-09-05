#!/usr/bin/env python3
import argparse
import os
import subprocess
import threading
from pathlib import Path
import orb_slam3_py as op
from aruco import add_arucos
from atlas_tools import Atlas, run_optimisation
from merge_maps import merge_maps
from colmap import create_colmap
from ply_maker import PlyMaker
from survey import add_survey, initialise_stations, add_legs_to_graph, make_survey_ply
import keyring
import getpass
import fabric
import paramiko

FROG_HOST="cm5.local"
FROG_USER="pi"
PASSWORD_PATH=Path("/footage/creds.txt")
#get password

def resolve_mdns(hostname):
    result = subprocess.run(
        ["avahi-resolve", "-4", "-n", hostname],
        capture_output=True, text=True, check=True
    )
    return result.stdout.split()[1]

def get_connection():
    password = PASSWORD_PATH.read_text().strip()
    #ip_addr = resolve_mdns(FROG_HOST)
    try:
        conn = fabric.Connection(host=FROG_HOST, user=FROG_USER, connect_kwargs={'password': password})
        conn.open()
    except paramiko.ssh_exception.AuthenticationException:
        print(f"Incorrect password - please check it is correct")
        exit()
    except IOError:
        print("Frog not found on local network, is it turned on?")
        exit()
    return conn

def make_path(root: Path, name: str ) -> Path:
    if name.startswith("/"):
        return Path(name)
    else:
        return root / name

def fetch_images(name: str, path: Path):
    remote_dir = Path("/data/trips/") / name
    with get_connection() as conn:
        sftp = conn.sftp()
        file_list = sftp.list_dir(remote_dir)
        if file_list is None:
            conn.run()
def fetch_atlas(name: str, path: Path):
    remote_dir = Path("/data/trips/") / name
    with get_connection() as conn:
        print("fetching atlas")
        try:
            conn.get(str(remote_dir / "atlas.txt.gz"), str(path / "atlas.txt.gz"))
        except IOError:
            print("Portable atlas not found - converting")
            conn.run(f"/home/{FROG_USER}/Cyclops/raspbian/portable.py -a {remote_dir / 'atlas.osa'} "
                     f"-o {remote_dir / 'atlas.txt.gz'}")
            print("Conversion complete, uploading")
        conn.get(str(remote_dir / "atlas.txt.gz"), str(path / "atlas.txt.gz"))
    print("atlas upload complete")

def fetch_data(name: str, path: Path) -> threading.Thread:
    """Pulls atlas data and survey data if they exist, then returns a thread
    The thread is running a separate process which is creating the images remotely and then downloading them
    """
    fetch_atlas(name, path)
    print("fetch complete")
    return None

OPENMVS_ARGS = """
docker run --gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
    --volume=/var/run/docker.sock:/var/run/docker.sock
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw
    --volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw 
    --env=XAUTHORITY=/tmp/.docker.xauth
    --env=DISPLAY=unix:1
    --ipc=host
    --shm-size=4gb
    --volume=footage:/footage
""".split()
OPENMVS_IMAGE = "openmvs-ubuntu-cuda"


def run_openMVS_cmd(cmd: str, model_dir: Path):
    cmds = cmd.split()
    mounts = f"--workdir={(model_dir / 'map_1').absolute()}"
    subprocess.run(OPENMVS_ARGS + [mounts, OPENMVS_IMAGE] + cmds, check=True)

def make_track_ply(atlas: Atlas, pth: Path):
    ply = PlyMaker()
    kfs = list(sorted(atlas.kfs.values(), key=lambda k: k.id))
    for k0,k1 in zip(kfs[:-1], kfs[1:]):
        ply.make_cylinder(k0.orig_pose.translation(), k1.orig_pose.translation())
    ply.write_ply(pth)
parser = argparse.ArgumentParser(description="Create a colmap from a bag (ROS1 or ROS2)")
parser.add_argument('-n', '--name', help="Name of Recording, if not specified will work with contents of target directory")
parser.add_argument('-d', '--dir', help="directory to store data and models", required=True)
parser.add_argument('-r', '--refined', help="Create a refined model, rather than rough", action="store_true")
parser.add_argument('--merge-maps', action=argparse.BooleanOptionalAction,
                    help="If more than one map in the atlas, try to merge them together", default=True)
parser.add_argument('--fresh', help="Do all steps from the beginning (no skipping)", default=True)
parser.add_argument('-a', '--atlas', help="Name of atlas file", default="atlas.txt.gz")
parser.add_argument('-s', '--survey-file', help="Name of survey file", default="survey.svx")
parser.add_argument("-i", '--images-dir', help="Directory to store images", default="images")
parser.add_argument('--use-survey', action=argparse.BooleanOptionalAction,
                    help="Use Survey data to adjust model", default=True)
parser.add_argument('--build-model', action=argparse.BooleanOptionalAction,
                    help="Build a full 3d model from data", default=True)
parser.add_argument('--use_imu', action=argparse.BooleanOptionalAction,
                    help="Use the IMU data to orient the model", default=True)
parser.add_argument('--coarse', action=argparse.BooleanOptionalAction,
                    help="Make a coarse (but fast) model", default=False)
opts = parser.parse_args()

#create various paths
model_dir = Path(opts.dir)
atlas_file = make_path(model_dir, opts.atlas)
survey_file = make_path(model_dir,opts.survey_file)
images_dir = make_path(model_dir,opts.images_dir)
name_parts = atlas_file.name.split(".")
name_parts[0] += '_final'
output_name = atlas_file.with_name('.'.join(name_parts))

#Pull data from Frog
if opts.name:
    images_pull_thread = fetch_data(opts.name, model_dir)
else:
    images_pull_thread = None
exit()

#REBUILD ATLAS
print("Loading atlas")
atlas = Atlas.from_file(atlas_file)
needs_refine = False
if opts.merge_maps:
    needs_refine =  len(atlas.maps) > 1
    merge_maps(atlas)
else:
    print("Skipping map merge stage")

#turn off use_survey if survey file does not exist
opts.use_survey = opts.use_survey and survey_file.exists()

if opts.use_survey:
    print("pulling in survey data and adjusting results")
    if images_pull_thread:
        images_pull_thread.join() # wait for all images to load
    add_arucos(atlas, images_dir)
    add_survey(atlas, survey_file)
    atlas.reload()
    station_positions = initialise_stations(atlas)
    G, values = atlas.create_graph(include_survey=True)
    add_legs_to_graph(atlas, G, values)
    values.insert_or_assign(station_positions)
    needs_refine = True
else:
    print("No survey file or instructed to skip it")
    if needs_refine:
        G, values = atlas.create_graph(include_survey=False)
if needs_refine:
    result = run_optimisation(G, values, iters=30)
    for map in atlas.maps.values():
        map.update_values(result)
    atlas.reload()
    atlas.save_file(output_name)
    if opts.use_survey:
        make_survey_ply(model_dir / "survey.ply", atlas, result)
make_track_ply(atlas, model_dir / "track.ply")

if not opts.build_model:
    print("Skipping model build")
else:
    if images_pull_thread:
        images_pull_thread.join() # wait for all images to load
    create_colmap(atlas, model_dir, images_dir)
    run_openMVS_cmd("ls", model_dir)
    run_openMVS_cmd("InterfaceCOLMAP -i . -o scene.mvs", model_dir)
    if opts.coarse:
        run_openMVS_cmd("ReconstructMesh -i scene.mvs", model_dir)
        run_openMVS_cmd("TextureMesh -i scene.mvs -m scene_mesh.ply -o ../model.ply", model_dir)
    else:
        run_openMVS_cmd("DensifyPointCloud -i scene.mvs", model_dir)
        run_openMVS_cmd("ReconstructMesh -i scene_dense.mvs --decimate=0.25", model_dir)
        run_openMVS_cmd("TextureMesh -i scene.mvs -m scene_dense_mesh.ply -o ../model.ply", model_dir)

