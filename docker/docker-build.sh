#!/bin/bash
set -euo pipefail
IFS=$'\n\t'
docker build --pull --tag robotem_rovne_docker .
