import numpy as np
import open3d as o3d
import imageio.v2 as imageio
import os

def ply_to_rgb_image(
    ply_path: str,
    out_rgb_path: str,
    fx: float, fy: float, cx: float, cy: float,
    W: int, H: int
):
    pcd = o3d.io.read_point_cloud(ply_path)
    if not pcd.has_colors():
        raise ValueError("PLY has no per-point colors.")

    pts = np.asarray(pcd.points)           # (N,3) in meters (usually)
    cols = np.asarray(pcd.colors)          # (N,3) floats in [0,1] (Open3D convention)

    X, Y, Z = pts[:, 0], pts[:, 1], pts[:, 2]
    valid = Z > 1e-6
    X, Y, Z = X[valid], Y[valid], Z[valid]
    cols = cols[valid]
    idx = np.nonzero(valid)[0]

    # Project
    u = (fx * (X / Z) + cx).astype(np.int32)
    v = (fy * (Y / Z) + cy).astype(np.int32)

    in_bounds = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, Z, cols, idx = u[in_bounds], v[in_bounds], Z[in_bounds], cols[in_bounds], idx[in_bounds]

    # Rasterize with z-buffer (closest point wins)
    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    zbuf = np.full((H, W), np.inf, dtype=np.float32)
    pix2pt = np.full((H, W), -1, dtype=np.int64)  # store original point index for each pixel

    cols_u8 = np.clip(cols * 255.0, 0, 255).astype(np.uint8)

    for k in range(len(Z)):
        uu, vv, zz = u[k], v[k], Z[k]
        if zz < zbuf[vv, uu]:
            zbuf[vv, uu] = zz
            rgb[vv, uu, :] = cols_u8[k]
            pix2pt[vv, uu] = idx[k]

    # save projected RGB
    imageio.imwrite(out_rgb_path, rgb)

    # Optional: save mapping + depth for later 2D->3D back-projection
    np.save(out_rgb_path.replace(".png", "_pix2pt.npy"), pix2pt)
    np.save(out_rgb_path.replace(".png", "_depth.npy"), zbuf)

    return rgb, pix2pt, zbuf

if __name__ == '__main__':
    # Example usage (replace with your intrinsics and resolution):
    W, H = 640, 480
    fx = fy = 600.0
    cx, cy = W/2.0, H/2.0

    _dir_path = os.path.dirname(os.path.realpath(__file__))
    scans_path = os.path.join(_dir_path, "..", "..", "Scans", "Zivid")

    ply_name = "owl0"

    ply_dir = os.path.join(scans_path, ply_name) + ".ply"
    rgb_dir = os.path.join(scans_path, ply_name) + ".png"
    rgb, pix2pt, depth = ply_to_rgb_image(
        ply_dir, rgb_dir,
        fx, fy, cx, cy, W, H
    )
