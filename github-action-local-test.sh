#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

IMAGE="${IMAGE:-my_steel/dev-test}"
DOCKERFILE="${DOCKERFILE:-.devcontainer/Dockerfile}"
PLATFORM="${PLATFORM:-linux/amd64}"
SKIP_BUILD="${SKIP_BUILD:-0}"  # Set to 1 to reuse an already-built IMAGE
CONTAINER_NAME="${CONTAINER_NAME:-my-steel-devcontainer}"

DOCKERFILE_PATH="${REPO_ROOT}/${DOCKERFILE}"

if [[ ! -f "${DOCKERFILE_PATH}" ]]; then
  echo "ERROR: Dockerfile not found at ${DOCKERFILE_PATH}" >&2
  exit 1
fi

if [[ "${SKIP_BUILD}" == "1" ]]; then
  echo "==> Skipping image build, reusing existing image ${IMAGE}"
else
  echo "==> Building image ${IMAGE} (Dockerfile: ${DOCKERFILE_PATH})"
  docker buildx build --load --platform "${PLATFORM}" -t "${IMAGE}" -f "${DOCKERFILE_PATH}" "${REPO_ROOT}"
fi

if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "==> Reusing existing container ${CONTAINER_NAME}"
  docker start "${CONTAINER_NAME}" >/dev/null || true
else
  echo "==> Creating persistent container ${CONTAINER_NAME}"
  docker run \
    --detach \
    --platform "${PLATFORM}" \
    --name "${CONTAINER_NAME}" \
    "${IMAGE}" \
    tail -f /dev/null >/dev/null
fi

echo "==> Syncing workspace into container ${CONTAINER_NAME}"
docker exec -u root "${CONTAINER_NAME}" bash -lc "rm -rf /workspace && mkdir -p /workspace"
docker cp "${REPO_ROOT}/." "${CONTAINER_NAME}:/workspace"
docker exec -u root "${CONTAINER_NAME}" chown -R ros:ros /workspace >/dev/null 2>&1 || true

docker exec "${CONTAINER_NAME}" bash -lc "rm -rf /workspace/build /workspace/install /workspace/log"

echo "==> Running .github/actions/test/run.sh inside ${CONTAINER_NAME}"
docker exec "${CONTAINER_NAME}" bash -lc "cd /workspace && .github/actions/test/run.sh"
EXIT_CODE=$?

echo "==> Stopping container ${CONTAINER_NAME} (container preserved for reuse)"
# docker stop "${CONTAINER_NAME}" >/dev/null || true
if [ "${EXIT_CODE}" -eq 0 ]; then
  echo "==> SUCCESS: Scripts completed inside container"
else
  echo "==> FAILURE: Scripts exited with code ${EXIT_CODE}"
fi


exit "${EXIT_CODE}"
