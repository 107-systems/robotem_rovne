#!/bin/bash
set -euo pipefail
IFS=$'\n\t'
docker buildx build --platform linux/arm64 \
  --tag robotem_rovne_docker \
  --output type=docker \
  .
