# Dashboard image: base:dev plus pyqtgraph for capcup_driver's cap_dashboard (S6, 2026-11-12).
# Separate so base:dev isn't rebuilt mid-session (apt-get update could move other packages away
# from the control host's copy). Fold python3-pyqtgraph into docker/base/apt-packages.txt at the
# next planned base rebuild, then delete this.
FROM ghcr.io/au-rmr/aurmr-2homa/base:dev
RUN apt-get update && apt-get install -y --no-install-recommends python3-pyqtgraph \
    && rm -rf /var/lib/apt/lists/*
