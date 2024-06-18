#!/bin/bash
set -euo pipefail
IFS=$'\n\t'
docker save robotem_rovne_docker:latest | gzip > robotem_rovne_docker_latest.tar.gz
