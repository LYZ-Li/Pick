#!/bin/bash
set -e

if docker compose version >/dev/null 2>&1; then
  COMPOSE_CMD=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
  COMPOSE_CMD=(docker-compose)
else
  echo "Docker Compose is not installed." >&2
  exit 1
fi

"${COMPOSE_CMD[@]}" -f docker/docker-compose.yml up -d --build
docker exec -it ros2_ur5e_dev bash
