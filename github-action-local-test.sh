#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-my_steel/pi-test}"
DOCKERFILE="${DOCKERFILE:-.devcontainer/Dockerfile.pi-test}"
CONTAINER="${CONTAINER:-pi-test}"
HOST_WORKDIR="${HOST_WORKDIR:-$PWD}"
CT_WORKDIR="${CT_WORKDIR:-/workspace}"
PLATFORM="${PLATFORM:-linux/arm64}"
SKIP_BUILD="${SKIP_BUILD:-0}"  # Set to 1 to reuse an already-built IMAGE

if [[ "${DOCKERFILE}" != /* ]]; then
  DOCKERFILE_PATH="${HOST_WORKDIR}/${DOCKERFILE}"
else
  DOCKERFILE_PATH="${DOCKERFILE}"
fi

BUILD_CONTEXT="${BUILD_CONTEXT:-${HOST_WORKDIR}}"

# shellcheck disable=SC2329
cleanup() {
  rc=$?
  if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    docker stop "${CONTAINER}" >/dev/null 2>&1 || true
    docker rm "${CONTAINER}" >/dev/null 2>&1 || true
  fi
  exit "${rc}"
}
trap cleanup INT TERM EXIT

if [[ "${SKIP_BUILD}" == "1" ]]; then
  echo "==> Skipping image build, reusing existing image ${IMAGE}"
else
  echo "==> Building image ${IMAGE} (Dockerfile: ${DOCKERFILE_PATH})"
  docker buildx build --load --platform "${PLATFORM}" -t "${IMAGE}" -f "${DOCKERFILE_PATH}" "${BUILD_CONTEXT}"
fi

echo "==> Starting container ${CONTAINER}"
docker run -d --platform "${PLATFORM}" --name "${CONTAINER}" "${IMAGE}" sleep infinity

echo "==> Copying workspace into container (no bind-mount; host bleibt unverändert)"
# docker cp "${HOST_WORKDIR}/." "${CONTAINER}:${CT_WORKDIR}"

echo "==> Fixing ownership & permissions inside container"
docker exec -u root "${CONTAINER}" chown -R ros:ros "${CT_WORKDIR}" || true
docker exec -u root "${CONTAINER}" find "${CT_WORKDIR}" -type f -name '*.sh' -exec chmod +x {} \; || true

echo "==> Executing setup, build and tests inside container as user 'ros'"
docker exec --user ros -it "${CONTAINER}" bash -lc "cd ${CT_WORKDIR} && ./setup.sh && ./build.sh && ./test.sh"
EXIT_CODE=$?

if [ "${EXIT_CODE}" -eq 0 ]; then
  echo "==> SUCCESS: Scripts completed inside container"
else
  echo "==> FAILURE: Scripts exited with code ${EXIT_CODE}"
fi

# cleanup via trap
exit "${EXIT_CODE}"
