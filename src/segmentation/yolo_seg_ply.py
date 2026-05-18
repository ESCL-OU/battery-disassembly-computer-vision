import numpy as np
import open3d as o3d
from ultralytics import YOLO
import os

def segment_ply_with_yolo(
    ply_path,
    out_ply_path,
    fx, fy, cx, cy, W, H,
    yolo_model="yolov8n-seg.pt",
    target_class_names=None,
    conf=0.25,
    out_rgb_path="render_rgb.png",
    out_overlay_path="yolo_overlay.png",
):
    """
    - Projects colored PLY -> RGB image (H,W,3)
    - Runs YOLO-seg on that image
    - Exports:
        * out_ply_path: segmented point cloud (masked points)
        * out_rgb_path: the projected RGB image
        * out_overlay_path: RGB image with YOLO masks/boxes drawn on top

    Notes:
    - Requires PLY with per-point colors.
    - Intrinsics (fx,fy,cx,cy) and image size (W,H) should correspond to the capture geometry.
      If unknown, use arbitrary values (e.g., W=640,H=480, fx=fy=600, cx=320, cy=240).
    """

    # -------- 1) Load point cloud --------
    pcd = o3d.io.read_point_cloud(ply_path)
    pts = np.asarray(pcd.points)  # (N,3)
    cols = np.asarray(pcd.colors) if pcd.has_colors() else None

    if pts.size == 0:
        raise ValueError("Point cloud is empty.")
    if cols is None:
        raise ValueError("PLY has no RGB colors (pcd.has_colors() == False).")

    # -------- 2) Project to pixels --------
    X, Y, Z = pts[:, 0], pts[:, 1], pts[:, 2]
    valid = Z > 1e-6
    X, Y, Z = X[valid], Y[valid], Z[valid]
    cols_valid = cols[valid]
    idx_valid = np.nonzero(valid)[0]

    u = (fx * (X / Z) + cx).astype(np.int32)
    v = (fy * (Y / Z) + cy).astype(np.int32)

    in_bounds = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, Z = u[in_bounds], v[in_bounds], Z[in_bounds]
    cols_valid = cols_valid[in_bounds]
    idx_valid = idx_valid[in_bounds]

    # -------- 3) Rasterize with z-buffer (closest point wins per pixel) --------
    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    zbuf = np.full((H, W), np.inf, dtype=np.float32)
    pix2pt = np.full((H, W), -1, dtype=np.int64)

    cols_u8 = np.clip(cols_valid * 255.0, 0, 255).astype(np.uint8)

    for k in range(len(Z)):
        uu, vv, zz = u[k], v[k], Z[k]
        if zz < zbuf[vv, uu]:
            zbuf[vv, uu] = zz
            rgb[vv, uu, :] = cols_u8[k]
            pix2pt[vv, uu] = idx_valid[k]

    # Save projected RGB
    try:
        import imageio.v2 as imageio
        imageio.imwrite(out_rgb_path, rgb)
    except Exception:
        # fallback to OpenCV if imageio isn't installed
        import cv2
        cv2.imwrite(out_rgb_path, rgb[:, :, ::-1])  # RGB->BGR

    # -------- 4) Run YOLO segmentation --------
    model = YOLO(yolo_model)
    results = model.predict(source=rgb, conf=conf, verbose=False)
    r = results[0]

    if r.masks is None or r.masks.data is None or len(r.masks.data) == 0:
        raise RuntimeError(
            "YOLO produced no masks. Pretrained model likely doesn't recognize your object."
        )

    masks = r.masks.data.cpu().numpy()  # (M,H,W) float/bool-ish
    scores = r.boxes.conf.cpu().numpy() if r.boxes is not None else np.ones((masks.shape[0],))
    cls_ids = r.boxes.cls.cpu().numpy().astype(int) if r.boxes is not None else np.zeros((masks.shape[0],), dtype=int)
    names = r.names  # dict: class_id -> class_name

    # -------- 5) Choose which instance mask to use for 3D export --------
    keep = np.ones((masks.shape[0],), dtype=bool)
    if target_class_names is not None:
        keep = np.array([names[c] in target_class_names for c in cls_ids], dtype=bool)

    if not np.any(keep):
        raise RuntimeError(
            "No masks matched target_class_names. "
            "Try target_class_names=None or print r.names to see available classes."
        )

    best_i = int(np.argmax(scores * keep.astype(scores.dtype)))
    best_mask = masks[best_i] > 0.5  # (H,W) boolean

    # -------- 6) Map mask pixels -> point indices -> export segmented PLY --------
    chosen_pt_idx = pix2pt[best_mask]
    chosen_pt_idx = chosen_pt_idx[chosen_pt_idx >= 0]
    chosen_pt_idx = np.unique(chosen_pt_idx)

    if chosen_pt_idx.size == 0:
        raise RuntimeError(
            "Mask mapped to zero points. Likely projection/intrinsics mismatch or empty rasterization."
        )

    seg_pcd = pcd.select_by_index(chosen_pt_idx.tolist())
    o3d.io.write_point_cloud(out_ply_path, seg_pcd)

    # -------- 7) Export YOLO visual overlay on the RGB image --------
    # Ultralytics can render masks/boxes; save that rendered image.
    # r.plot() returns a BGR uint8 image (OpenCV-style) in most Ultralytics versions.
    overlay_bgr = r.plot()
    saved = False

    # Try OpenCV first (expects BGR)
    try:
        import cv2
        cv2.imwrite(out_overlay_path, overlay_bgr)
        saved = True
    except Exception:
        saved = False

    # Fallback: convert to RGB and use imageio/PIL
    if not saved:
        overlay_rgb = overlay_bgr[:, :, ::-1]
        try:
            import imageio.v2 as imageio
            imageio.imwrite(out_overlay_path, overlay_rgb)
            saved = True
        except Exception:
            from PIL import Image
            Image.fromarray(overlay_rgb).save(out_overlay_path)
            saved = True

    meta = {
        "out_ply_path": out_ply_path,
        "out_rgb_path": out_rgb_path,
        "out_overlay_path": out_overlay_path,
        "num_mask_instances": int(masks.shape[0]),
        "best_instance_index": best_i,
        "best_instance_score": float(scores[best_i]),
        "best_instance_class_id": int(cls_ids[best_i]) if len(cls_ids) > best_i else None,
        "best_instance_class_name": names[int(cls_ids[best_i])] if len(cls_ids) > best_i else None,
        "segmented_point_count": int(chosen_pt_idx.size),
    }
    return meta

if __name__ == '__main__':
    # Example usage (replace with your intrinsics and resolution):
    W, H = 640, 480
    fx = fy = 600.0
    cx, cy = W/2.0, H/2.0

    _dir_path = os.path.dirname(os.path.realpath(__file__))
    model_dir = os.path.join(_dir_path, "..", "..", "models", "YOLO")
    scans_path = os.path.join(_dir_path, "..", "..", "Scans", "Zivid")

    ply_name = "battery_pack"
    model_name = "yolov8n-seg.pt"

    ply_dir = os.path.join(scans_path, ply_name) + ".ply"
    rgb_dir = os.path.join(scans_path, ply_name) + ".png"
    rgb_overlay_dir = os.path.join(scans_path, ply_name) + "_overlay.png"
    seg_ply_dir = os.path.join(scans_path, ply_name) + "_seg.ply"

    model = os.path.join(model_dir, model_name)

    meta = segment_ply_with_yolo(
        ply_dir, seg_ply_dir,
        fx, fy, cx, cy, W, H,
        yolo_model=model,
        target_class_names=None,
        out_rgb_path= rgb_dir,
        out_overlay_path = rgb_overlay_dir,
    )
