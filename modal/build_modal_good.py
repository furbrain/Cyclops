import modal
import os

app = modal.App("openmvs")

image = (
    modal.Image.from_registry("furbrain/openmvs-cuda:2.4.0")
    .entrypoint([])
)

cpu_image = (
    modal.Image.from_registry("furbrain/openmvs-cuda:2.4.0")
    .apt_install("nvidia-cuda-toolkit")
    .entrypoint([])
    .env({
        "LD_LIBRARY_PATH": "/usr/local/cuda/lib64/stubs:$LD_LIBRARY_PATH"
    })
)

footage = modal.Volume.from_name("footage")

@app.function(image=image, gpu="L4", timeout=3600, volumes={"/footage":footage})
def check():
    import subprocess
    import glob
    os.chdir("/footage/modal_test/map_1")
    for f in glob.glob("*.dmap"):
        os.remove(f)
    os.remove("scene.mvs")
    subprocess.run("InterfaceCOLMAP -i . --image-folder ../images", shell=True)
    subprocess.run("DensifyPointCloud -i scene.mvs -v 2  --max-threads=2", shell=True)

@app.function(image=cpu_image, cpu=3.0, timeout=3600, volumes={"/footage":footage})
def texture():
    import subprocess
    os.chdir("/footage/modal_test/map_1")
    #subprocess.run("ReconstructMesh -i scene.mvs -v 2", shell=True)
    #subprocess.run("TextureMesh -i scene.mvs -m scene_mesh.ply  --global-seam-leveling 0 --local-seam-leveling 0",
    # shell=True)
    subprocess.run("TextureMesh -i scene.mvs -m scene_mesh.ply --global-seam-leveling 0", shell=True)



@app.local_entrypoint()
def main():
    texture.remote()
