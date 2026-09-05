#!/usr/bin/env python3
"""
ply_maker.py

Generate a standalone, vertex-colored, untextured PLY file representing
survey legs as triangulated cylinder tubes. Also can produce spheres

"""

import numpy as np
import struct
from typing import Union, Tuple

class PlyMaker:
    def __init__(self):
        self.verts = np.ones((0,3))
        self.faces = np.ones((0,3),dtype=int)
        self.colors = np.ones((0,3), dtype=int)

    def add_object(self, verts, faces, color:Union[np.ndarray,Tuple[int]]):
        offset = len(self.verts)
        self.verts = np.vstack([self.verts, np.array(verts)])
        self.faces = np.vstack([self.faces, np.array(faces)+offset])
        self.colors = np.vstack([self.colors, np.tile(color, (len(verts), 1))])

    def make_cylinder(self, p1, p2, radius=0.05, sides=8, color=(255,255,0)):
        """
        Generate a triangulated cylinder tube between two 3D points.
        """
        p1 = np.asarray(p1, dtype=np.float64)
        p2 = np.asarray(p2, dtype=np.float64)
        axis = p2 - p1
        length = np.linalg.norm(axis)
        if length < 1e-9:
            return np.empty((0, 3)), np.empty((0, 3), dtype=int)
        axis = axis / length

        # orthonormal basis around the axis
        arbitrary = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        u = np.cross(axis, arbitrary)
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)

        angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)
        ring = np.array([np.cos(a) * u + np.sin(a) * v for a in angles])  # (sides, 3)

        verts = np.vstack([p1 + radius * ring, p2 + radius * ring])  # (2*sides, 3)

        faces = []
        for i in range(sides):
            i2 = (i + 1) % sides
            faces.append([i, i2, sides + i2])
            faces.append([i, sides + i2, sides + i])
        faces = np.array(faces, dtype=int)
        self.add_object(verts,faces,color)


    def make_sphere(self, center, radius=0.1, lat_segments=8, lon_segments=12, color=(0,255,0)):
        """
        Generate a triangulated UV sphere at `center`.
        """
        center = np.asarray(center, dtype=np.float64)

        verts = []
        for i in range(lat_segments + 1):
            theta = np.pi * i / lat_segments  # 0 (top) .. pi (bottom)
            y = np.cos(theta)
            r_at_lat = np.sin(theta)
            for j in range(lon_segments):
                phi = 2 * np.pi * j / lon_segments
                x = r_at_lat * np.cos(phi)
                z = r_at_lat * np.sin(phi)
                verts.append(center + radius * np.array([x, y, z]))

        def idx(i, j):
            return i * lon_segments + (j % lon_segments)

        faces = []
        for i in range(lat_segments):
            for j in range(lon_segments):
                a = idx(i, j)
                b = idx(i + 1, j)
                c = idx(i + 1, j + 1)
                d = idx(i, j + 1)
                if i != 0:
                    faces.append([a, b, d])
                if i != lat_segments - 1:
                    faces.append([b, c, d])
        self.add_object(verts, faces, color)

    def write_ply(self, path, binary=True):
        """
        Write a vertex-colored, untextured PLY file.
        """
        n_verts = len(self.verts)
        n_faces = len(self.faces)

        header = [
            "ply",
            f"format {'binary_little_endian 1.0' if binary else 'ascii 1.0'}",
            f"element vertex {n_verts}",
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            f"element face {n_faces}",
            "property list uchar int vertex_indices",
            "end_header",
        ]
        header_bytes = ("\n".join(header) + "\n").encode("ascii")

        if binary:
            with open(path, "wb") as f:
                f.write(header_bytes)
                for (x, y, z), (r, g, b) in zip(self.verts, self.colors):
                    f.write(struct.pack("<fffBBB", float(x), float(y), float(z),
                                         int(r), int(g), int(b)))
                for face in self.faces:
                    f.write(struct.pack("<B", 3))  # triangle
                    f.write(struct.pack("<3i", int(face[0]), int(face[1]), int(face[2])))
        else:
            with open(path, "w") as f:
                f.write(header_bytes.decode("ascii"))
                for (x, y, z), (r, g, b) in zip(self.verts, self.colors):
                    f.write(f"{x} {y} {z} {int(r)} {int(g)} {int(b)}\n")
                for face in self.faces:
                    f.write(f"3 {int(face[0])} {int(face[1])} {int(face[2])}\n")



if __name__ == "__main__":
    # Example usage with dummy data - replace with your actual leg endpoints
    example_legs = [
        ((0.0, 0.0, 0.0), (1.5, 0.2, -0.3)),
        ((1.5, 0.2, -0.3), (2.8, 0.9, -0.1)),
        ((2.8, 0.9, -0.1), (3.0, 2.1, 0.4)),
    ]

    # Example markers - e.g. survey stations or ArUco marker positions,
    # each with its own colour and (optionally) its own radius
    example_sphere_points = [
        (0.0, 0.0, 0.0),
        (2.8, 0.9, -0.1),
        (3.0, 2.1, 0.4),
    ]
    example_sphere_colors = [
        (0, 255, 0),    # green - start station
        (255, 255, 0),  # yellow - intermediate
        (0, 128, 255),  # blue - end station
    ]

    pb = PlyMaker()
    for leg in example_legs:
        pb.make_cylinder(leg[0], leg[1], color=(0,0,255))
    for point, colour in zip(example_sphere_points, example_sphere_colors):
        pb.make_sphere(point, color=colour)
    pb.write_ply("test.ply", binary=False)