#!/usr/bin/env python3
"""
Render an isometric view of the gencer_quadruped robot directly from
model.sdf link poses + the real STL mesh files (no .step files exist in
the project, so this uses the actual simulation-ground-truth data instead).
"""
import xml.etree.ElementTree as ET
import numpy as np
from stl import mesh as stlmesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os

BASE = "/mnt/user-data/uploads/gz-ws/models/gencer_quadruped"
SDF_PATH = os.path.join(BASE, "model.sdf")

def rot_matrix(roll, pitch, yaw):
    # SDF/Gazebo extrinsic fixed-axis convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def parse_links(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find('model')
    links = []
    for l in model.findall('link'):
        name = l.get('name')
        pose_el = l.find('pose')
        pose_vals = [float(x) for x in pose_el.text.split()] if pose_el is not None else [0]*6
        vis = l.find('visual')
        mesh_uri = None
        scale = (1.0, 1.0, 1.0)
        color = (0.6, 0.6, 0.6)
        if vis is not None:
            geom = vis.find('geometry')
            if geom is not None:
                m = geom.find('mesh')
                if m is not None:
                    uri_el = m.find('uri')
                    mesh_uri = uri_el.text if uri_el is not None else None
                    sc_el = m.find('scale')
                    if sc_el is not None:
                        scale = tuple(float(x) for x in sc_el.text.split())
            mat = vis.find('material')
            if mat is not None:
                amb = mat.find('ambient')
                if amb is not None:
                    vals = [float(x) for x in amb.text.split()]
                    color = tuple(vals[:3])
        links.append({
            'name': name,
            'pose': pose_vals,
            'mesh_uri': mesh_uri,
            'scale': scale,
            'color': color,
        })
    return links

def load_transformed_mesh(link, base_dir):
    mesh_path = os.path.join(base_dir, link['mesh_uri'])
    m = stlmesh.Mesh.from_file(mesh_path)
    verts = m.vectors.reshape(-1, 3).copy()  # (N*3, 3)

    sx, sy, sz = link['scale']
    verts[:, 0] *= sx
    verts[:, 1] *= sy
    verts[:, 2] *= sz

    x, y, z, roll, pitch, yaw = link['pose']
    R = rot_matrix(roll, pitch, yaw)
    verts = verts @ R.T
    verts += np.array([x, y, z])

    return verts.reshape(-1, 3, 3)  # back to triangles

def main():
    links = parse_links(SDF_PATH)

    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection='3d')

    light_dir = np.array([0.4, -0.5, 0.8])
    light_dir = light_dir / np.linalg.norm(light_dir)

    all_pts = []
    for link in links:
        if link['mesh_uri'] is None:
            continue
        tris = load_transformed_mesh(link, BASE)
        color = np.array(link['color'])

        # per-face normal-based shading for a solid CAD-like look
        v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
        normals = np.cross(v1 - v0, v2 - v0)
        norm_lens = np.linalg.norm(normals, axis=1, keepdims=True)
        norm_lens[norm_lens == 0] = 1.0
        normals = normals / norm_lens
        brightness = 0.45 + 0.55 * np.clip(normals @ light_dir, 0, 1)
        face_colors = np.clip(color[None, :] * brightness[:, None], 0, 1)

        pc = Poly3DCollection(tris, facecolors=face_colors, edgecolor=None, linewidths=0.0, alpha=1.0)
        ax.add_collection3d(pc)
        all_pts.append(tris.reshape(-1, 3))

    all_pts = np.vstack(all_pts)
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    center = (mins + maxs) / 2
    max_range = (maxs - mins).max() / 2 * 1.15

    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)

    # isometric-ish view
    ax.view_init(elev=25, azim=-55)
    ax.set_box_aspect([1, 1, 1])
    ax.set_axis_off()
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    plt.tight_layout()
    out_path = "/mnt/user-data/outputs/quadruped_isometric.png"
    plt.savefig(out_path, dpi=300, bbox_inches='tight', facecolor='white')
    print("saved:", out_path)

if __name__ == "__main__":
    main()
