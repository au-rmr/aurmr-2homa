"""Camera extrinsic calibration, eye-to-hand: AprilTags rigid on arm links, camera fixed.

    python3 tests/perception/handeye_solve.py <dir> --tags 1 36 39 --tag-size 0.036 [--family 36h11] [--out result.json]

<dir> holds, per pose, <name>_tf.json (tests/arm/handeye_sample.py: base_link -> each arm link) and <name>.npz
(tests/perception/capture_frames.py: colour image + K + D). Each tag may sit on a different link; which one is
found from the data. Steps:
 1. Per pose and tag: detect (OpenCV AprilTag dictionary, sub-pixel corners) and solvePnP with the camera's own
    intrinsics and distortion (rational polynomial).
 2. Per tag and candidate link: closed-form hand-eye (cv2.calibrateHandEye, eye-to-hand form, 5 methods), then
    refine camera + tag-on-link by the corners' reprojection error. The link with the lowest error is the tag's.
    Single-tag PnP of a small tag is ambiguous (a few poses off by degrees), hence the refinement.
 3. All tags together: one camera pose, one pose per tag on its link, refined over every corner of every pose.
Prints per-tag fits, the final reprojection rms and the camera pose (base_link -> the colour camera's frame,
rgb_camera_link: z forward, x right, y down). --tag-size is the black square's edge in m.
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

MIN_POSES = 4


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [float(x), float(y), float(z), float(w)]


def T(R, t):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, np.ravel(t)
    return m


def angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def tag_points(size):
    h = size / 2   # OpenCV corner order: top-left, top-right, bottom-right, bottom-left; tag z out of the tag
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def detect(img, family, tag_ids, size, K, D):
    """{tag_id: (T_cam_tag, reprojection px, side px, corners 4x2)} for the wanted tags seen in img."""
    gray = cv2.cvtColor(img[..., :3], cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_APRILTAG_{family}"))
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    found = {}
    if ids is None:
        return found
    obj = tag_points(size)
    for c, i in zip(corners, ids.ravel()):
        if int(i) not in tag_ids:
            continue
        c = cv2.cornerSubPix(gray, c.reshape(-1, 1, 2).astype(np.float32), (5, 5), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)).reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(obj, c.astype(np.float64), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - c) ** 2, axis=1))))
        side = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]))
        found[int(i)] = (T(cv2.Rodrigues(rvec)[0], tvec), err, side, c.astype(np.float64))
    return found


def to_x(Ts):
    return np.concatenate([np.concatenate([cv2.Rodrigues(m[:3, :3])[0].ravel(), m[:3, 3]]) for m in Ts])


def from_x(x):
    return [T(cv2.Rodrigues(x[6 * k:6 * k + 3])[0], x[6 * k + 3:6 * k + 6]) for k in range(len(x) // 6)]


def refine(obs, T_base_cam, T_links_tag, size, free=()):
    """obs: list of (tag index, T_base_link, K, D, corners). Refines [camera, tag 0, tag 1, ...] together.
    Tag indices in `free` also get their width and height fitted (a tag wrapped on a tube: its corners stay
    coplanar, but the corner-to-corner width is a chord, shorter than the printed size). Returns the poses, rms,
    per-corner errors and {tag index: (width, height)} for the free tags."""
    from scipy.optimize import least_squares
    n = len(T_links_tag)
    free = sorted(free)

    def split(x):
        Ts = from_x(x[:6 * (n + 1)])
        wh = {k: x[6 * (n + 1) + 2 * j: 6 * (n + 1) + 2 * j + 2] for j, k in enumerate(free)}
        return Ts, wh

    def residuals(x):
        Ts, wh = split(x)
        Tcb = np.linalg.inv(Ts[0])
        res = []
        for k, Tbl, K, D, corners in obs:
            w, h = wh.get(k, (size, size))
            obj = np.array([[-w / 2, h / 2, 0, 1], [w / 2, h / 2, 0, 1], [w / 2, -h / 2, 0, 1], [-w / 2, -h / 2, 0, 1]]).T
            pc = (Tcb @ Tbl @ Ts[1 + k] @ obj)[:3].T
            proj, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, D)
            res.append((proj.reshape(4, 2) - corners).ravel())
        return np.concatenate(res)

    x0 = np.concatenate([to_x([T_base_cam] + list(T_links_tag))] + [np.array([size, size])] * len(free))
    r = least_squares(residuals, x0, loss="soft_l1", f_scale=1.0)
    res = residuals(r.x).reshape(-1, 2)
    per_corner = np.linalg.norm(res, axis=1)
    Ts, wh = split(r.x)
    return Ts, float(np.sqrt(np.mean(per_corner ** 2))), per_corner, {k: tuple(float(v) for v in wh[k]) for k in free}


def closed_form(samples):
    """samples: list of (T_base_link, T_cam_tag). Best of the 5 OpenCV methods by tag-on-link consistency."""
    R_g, t_g, R_t, t_t = [], [], [], []
    for Tbl, Tct in samples:
        Tlb = np.linalg.inv(Tbl)   # eye-to-hand: feed link<-base as "gripper2base"; the result is camera -> base
        R_g.append(Tlb[:3, :3]); t_g.append(Tlb[:3, 3]); R_t.append(Tct[:3, :3]); t_t.append(Tct[:3, 3])
    best = None
    for method in ("TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"):
        try:
            R, t = cv2.calibrateHandEye(R_g, t_g, R_t, t_t, method=getattr(cv2, f"CALIB_HAND_EYE_{method}"))
        except cv2.error:
            continue
        Tbc = T(R, t)
        tags = [np.linalg.inv(Tbl) @ Tbc @ Tct for Tbl, Tct in samples]
        spread = float(np.mean([np.linalg.norm(m[:3, 3] - np.mean([n[:3, 3] for n in tags], axis=0)) for m in tags]))
        if np.all(np.isfinite(Tbc)) and (best is None or spread < best[1]):
            best = (Tbc, spread, tags[0], method)
    return best


def main():
    p = argparse.ArgumentParser()
    p.add_argument("dir")
    p.add_argument("--tags", type=int, nargs="+", required=True)
    p.add_argument("--tag-size", type=float, required=True)
    p.add_argument("--family", default="36h11")
    p.add_argument("--curved", type=int, nargs="*", default=[],
                   help="tags wrapped on a curved surface: their width and height are fitted too")
    p.add_argument("--out")
    a = p.parse_args()
    poses = []   # (name, {link: T_base_link}, K, D, {tag: detection})
    for tf_path in sorted(glob.glob(os.path.join(a.dir, "*_tf.json"))):
        name = os.path.basename(tf_path)[:-len("_tf.json")]
        img_path = os.path.join(a.dir, f"{name}.npz")
        if not os.path.exists(img_path):
            print(f"{name}: no image, skipped")
            continue
        tf = json.load(open(tf_path))
        links = {l: T(quat_to_R(v["quat_xyzw"]), v["xyz"]) for l, v in tf["frames"].items()}
        f = np.load(img_path)
        det = detect(f["rgb"], a.family, a.tags, a.tag_size, f["K"], f["D"])
        poses.append((name, links, f["K"], f["D"], det))
        seen = ", ".join(f"{t}: {d[2]:.0f}px/{d[1]:.2f}px" for t, d in sorted(det.items())) or "none"
        print(f"{name}: tags {seen}; link spread {tf['spread_mm']:.2f} mm")
    chosen = {}   # tag -> (link, T_link_tag, rms)
    for tag in a.tags:
        have = [q for q in poses if tag in q[4]]
        if len(have) < MIN_POSES:
            print(f"tag {tag}: seen in {len(have)} poses (< {MIN_POSES}), not used")
            continue
        fits = []
        for link in have[0][1]:
            cf = closed_form([(q[1][link], q[4][tag][0]) for q in have])
            if cf is None:
                continue
            obs = [(0, q[1][link], q[2], q[3], q[4][tag][3]) for q in have]
            Ts, rms, _, _ = refine(obs, cf[0], [np.linalg.inv(have[0][1][link]) @ cf[0] @ have[0][4][tag][0]], a.tag_size,
                                   free=[0] if tag in a.curved else [])
            fits.append((rms, link, Ts[1], Ts[0]))
        fits.sort(key=lambda f: f[0])
        print(f"tag {tag}: {len(have)} poses; reprojection rms by link: "
              + ", ".join(f"{l} {r:.2f}px" for r, l, *_ in fits))
        chosen[tag] = fits[0]
    if not chosen:
        sys.exit("no tag seen in enough poses")
    tags = sorted(chosen)
    obs = [(k, q[1][chosen[t][1]], q[2], q[3], q[4][t][3]) for k, t in enumerate(tags) for q in poses if t in q[4]]
    # Start the joint fit from each tag's own camera solution and keep the best result. A tag that only ever turns
    # about one axis (e.g. on the shoulder: base rotation only) fits well alone but leaves the camera undetermined,
    # so its own solution is not a safe start.
    best = None
    for t0 in tags:
        T_links = [np.linalg.inv(chosen[t0][3]) @ chosen[t0][3] @ chosen[t][2] if t == t0 else None for t in tags]
        # tag offsets consistent with this start camera: average over that tag's views
        T_links = []
        for t in tags:
            views = [q for q in poses if t in q[4]]
            Tlt = [np.linalg.inv(q[1][chosen[t][1]]) @ chosen[t0][3] @ q[4][t][0] for q in views]
            T_links.append(Tlt[len(Tlt) // 2])
        res = refine(obs, chosen[t0][3], T_links, a.tag_size, free=[k for k, t in enumerate(tags) if t in a.curved])
        print(f"joint fit started from tag {t0}'s camera: rms {res[1]:.2f} px")
        if best is None or res[1] < best[1]:
            best = res
    Ts, rms, per_corner, wh = best
    for k, (w, h) in wh.items():
        print(f"tag {tags[k]} (curved): fitted width {1000 * w:.1f} mm, height {1000 * h:.1f} mm "
              f"(printed {1000 * a.tag_size:.1f})")
    T_base_cam = Ts[0]
    singles = {t: chosen[t][3] for t in tags}
    agree = {t: (round(1000 * float(np.linalg.norm(singles[t][:3, 3] - T_base_cam[:3, 3])), 1),
                 round(angle_deg(singles[t][:3, :3].T @ T_base_cam[:3, :3]), 2)) for t in tags}
    print(f"ALL TAGS  camera at {np.round(T_base_cam[:3, 3], 4)} m; reprojection rms {rms:.2f} px, "
          f"worst corner {per_corner.max():.1f} px, {len(obs)} tag views")
    print("single-tag solutions vs joint (mm, deg): " + ", ".join(f"{t}: {v}" for t, v in agree.items()))
    out = {"parent": "base_link", "child": "rgb_camera_link", "xyz": [float(v) for v in T_base_cam[:3, 3]],
           "quat_xyzw": R_to_quat(T_base_cam[:3, :3]), "reprojection_rms_px": round(rms, 3),
           "worst_corner_px": round(float(per_corner.max()), 2), "views": len(obs), "poses": len(poses),
           "tags": {str(t): {"link": chosen[t][1], "xyz": [float(v) for v in Ts[1 + k][:3, 3]],
                             "quat_xyzw": R_to_quat(Ts[1 + k][:3, :3]), "single_tag_rms_px": round(chosen[t][0], 3),
                             "single_vs_joint_mm_deg": agree[t]} for k, t in enumerate(tags)},
           "family": a.family, "tag_size_m": a.tag_size,
           "curved_tags_fitted_size_m": {str(tags[k]): [round(w, 5), round(h, 5)] for k, (w, h) in wh.items()}}
    print(json.dumps(out, indent=1))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
