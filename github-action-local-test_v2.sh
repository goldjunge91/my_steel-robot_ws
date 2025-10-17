#!/usr/bin/env bash
set -euo pipefail

# Local harness that mirrors the GitHub Action by using the devcontainer CLI.

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEVCONTAINER_CONFIG="${DEVCONTAINER_CONFIG:-.devcontainer/devcontainer.json}"
DEVCONTAINER_CLI="${DEVCONTAINER_CLI:-devcontainer}"
RUN_COMMAND="${RUN_COMMAND:-./setup.sh && ./build.sh && ./test.sh}"
CLEANUP="${CLEANUP:-0}"

if command -v "${DEVCONTAINER_CLI}" >/dev/null 2>&1; then
  if [[ "${DEVCONTAINER_CONFIG}" != /* ]]; then
    DEVCONTAINER_PATH="${REPO_ROOT}/${DEVCONTAINER_CONFIG}"
  else
    DEVCONTAINER_PATH="${DEVCONTAINER_CONFIG}"
  fi

  if [[ ! -f "${DEVCONTAINER_PATH}" ]]; then
    echo "ERROR: devcontainer configuration not found at ${DEVCONTAINER_PATH}" >&2
    exit 1
  fi

  cleanup() {
    local rc=$?
    if [[ "${CLEANUP}" == "1" ]]; then
      "${DEVCONTAINER_CLI}" rm --workspace-folder "${REPO_ROOT}" --config "${DEVCONTAINER_PATH}" >/dev/null 2>&1 || true
    fi
    exit "${rc}"
  }
  trap cleanup INT TERM EXIT

  echo "==> Bringing up devcontainer (${DEVCONTAINER_PATH})"
  "${DEVCONTAINER_CLI}" up --workspace-folder "${REPO_ROOT}" --config "${DEVCONTAINER_PATH}" >/dev/null

  echo "==> Running scripts inside devcontainer"
  set +e
  "${DEVCONTAINER_CLI}" exec --workspace-folder "${REPO_ROOT}" --config "${DEVCONTAINER_PATH}" -- bash -lc "${RUN_COMMAND}"
  EXIT_CODE=$?
  set -e
else
  echo "WARNING: '${DEVCONTAINER_CLI}' CLI not found. Falling back to plain Docker run." >&2
  IMAGE="${IMAGE:-devcontainer-local}"
  DOCKERFILE="${DOCKERFILE:-.devcontainer/Dockerfile}"
  PLATFORM="${PLATFORM:-linux/amd64}"
  CONTAINER_NAME="${CONTAINER_NAME:-devcontainer-local-fallback}"

  if [[ "${DOCKERFILE}" != /* ]]; then
    DOCKERFILE_PATH="${REPO_ROOT}/${DOCKERFILE}"
  else
    DOCKERFILE_PATH="${DOCKERFILE}"
  fi

  if [[ ! -f "${DOCKERFILE_PATH}" ]]; then
    echo "ERROR: Dockerfile not found at ${DOCKERFILE_PATH}" >&2
    exit 1
  fi

  echo "==> Building image ${IMAGE} from ${DOCKERFILE_PATH}"
  docker buildx build --load --platform "${PLATFORM}" -t "${IMAGE}" -f "${DOCKERFILE_PATH}" "${REPO_ROOT}"

  if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    echo "==> Reusing existing container ${CONTAINER_NAME}"
    docker start "${CONTAINER_NAME}" >/dev/null || true
  else
    echo "==> Creating persistent container ${CONTAINER_NAME}"
    docker run --detach --platform "${PLATFORM}" --name "${CONTAINER_NAME}" "${IMAGE}" tail -f /dev/null >/dev/null
  fi

  echo "==> Running scripts inside Docker container"
  set +e
  docker exec -u root "${CONTAINER_NAME}" bash -lc "rm -rf /workspace && mkdir -p /workspace"
  docker cp "${REPO_ROOT}/." "${CONTAINER_NAME}:/workspace"
  docker exec -u root "${CONTAINER_NAME}" chown -R ros:ros /workspace >/dev/null 2>&1 || true
  docker exec "${CONTAINER_NAME}" bash -lc "rm -rf /workspace/build /workspace/install /workspace/log"
  docker exec "${CONTAINER_NAME}" bash -lc "cd /workspace && ${RUN_COMMAND}"
  EXIT_CODE=$?
  set -e
  docker stop "${CONTAINER_NAME}" >/dev/null || true
fi

if [ "${EXIT_CODE}" -eq 0 ]; then
  echo "==> SUCCESS: Scripts completed inside container environment"
else
  echo "==> FAILURE: Scripts exited with code ${EXIT_CODE}"
fi

exit "${EXIT_CODE}"
