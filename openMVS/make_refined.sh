#!/bin/sh
if [ ! -e scene.mvs ]
then
  echo "Converting to COLMAP Format"
  InterfaceCOLMAP -i . -o scene.mvs
else
  echo "COLMAP data found"
fi

if [ ! -e scene_dense.ply ]
then
  echo "Creating dense point cloud"
  DensifyPointCloud -i scene.mvs
else
  echo "Dense point cloud found"
fi

if [ ! -e scene_dense_mesh.ply ]
then
  echo "Creating mesh"
  ReconstructMesh -i scene_dense.mvs
else
  echo "Mesh found"
fi

if [ ! -e scene_dense_mesh_refine.ply ]
then
  echo "Refining Mesh"
  RefineMesh -i scene_dense.mvs -m scene_dense_mesh.ply -o scene_dense_mesh_refine.ply
else
  echo "Refined Mesh Found"
fi

if [ ! -e refined.ply ]
then
  echo "Texturing Mesh"
  TextureMesh -i scene_dense.mvs -m scene_dense_mesh_refine.ply -o refined.ply
else
  echo "Textured Mesh found"
fi
chmod 0644 *ply
echo "Tidying up"
rm -f *dmap
rm -f *log