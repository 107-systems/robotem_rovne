#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ "$#" -eq 0 ]; then
  echo "Usage: fetch-map.sh 192.168.8.1"
fi

scp -r -C fio@"$1":/tmp/2024* /tmp
