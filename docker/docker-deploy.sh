#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ "$#" -eq 0 ]; then
  echo "Usage: docker-deploy.sh 192.168.8.1"
fi

scp robotem_rovne_docker_latest.tar.gz fio@"$1":/home/fio
