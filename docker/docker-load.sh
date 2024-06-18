#!/bin/bash
set -euo pipefail
IFS=$'\n\t'
docker load < robotem_rovne_docker_latest.tar.gz
