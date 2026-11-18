# Standalone packages: keep boundaries so code can leave the repo

This repo holds everything needed to run this robot, versioned together. Code with an audience of its own (a paper's
model, an algorithm another lab could use, a device driver) is a dependency: it lives in its own repo and is pinned
by commit: ROS source packages in `third_party.repos`, Python packages as `pip install git+<repo>@<full sha>` in
the image's Dockerfile (e.g. `docker/perception/Dockerfile`). Example: OptiGrasp inference lives in
`au-rmr/optigrasp`; the ROS 2 node that calls it and the robot parts (camera calibration, bin bounds, pixel to 3D)
stay here.

## What may be a dependency
- Repos the project controls (au-rmr/*, made for the new stack) and large upstream projects (e.g. facebookresearch,
  Universal Robots), pinned by commit.
- Not personal repos, and not the ROS 1-era au-rmr repos: they go private, move or vanish. Code from them is copied in
  (here or into a project repo) with its history noted and its authors credited. Code found on the lab machines was
  contributed to this project by its authors and may be used this way; it keeps its licence, unlabelled lab code is MIT.

## Writing code that may leave later
Start it here, as its own folder, if nobody outside needs it yet. Keep it easy to move:
- No imports from robot code (`robot/`, `skills/`, `demos/`, `sites/`), no lab paths, hostnames or calibration.
  It gets everything through function arguments or its own config.
- Plain Python (or plain C++) at the core; ROS goes in a thin wrapper next to it, not inside it.
- Its own tests that run without the robot stack, and its own licence header if it is not MIT.
- Its own dependencies listed in its own folder (e.g. `pyproject.toml`), not only in our Docker images.

## Breaking it out
Do it when someone outside actually needs it (a paper release, another lab), not before: each separate repo is
another pin to keep updated.
1. Extract with history: `git filter-repo --subdirectory-filter <dir>` on a fresh clone. Authors and commits stay.
2. Add README credits, `LICENSE`, `CITATION.cff` if it belongs to a paper.
3. Here: delete the folder and pin the repo by commit (as above), in the same commit.
4. From then on the external repo is the source of truth: changes go there first, then bump the pin here.
   Never edit two copies.
