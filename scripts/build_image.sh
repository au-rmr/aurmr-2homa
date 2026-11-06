#!/usr/bin/env bash
# Builds the base image. Only needs Docker; changes nothing on the host outside Docker.
set -eu
cd "$(git rev-parse --show-toplevel)"
docker build -f docker/base/Dockerfile -t ghcr.io/au-rmr/aurmr-2homa/base:dev "$@" .
