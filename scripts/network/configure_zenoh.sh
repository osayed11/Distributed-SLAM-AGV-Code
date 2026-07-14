#!/usr/bin/env bash
# Install the official Eclipse Zenoh ROS 2 DDS bridge for ORKAR ground truth.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ZENOH_VERSION="1.9.0"
RELEASE_BASE="https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}"

usage() {
    cat <<'EOF'
Usage:
  bash scripts/network/configure_zenoh.sh router-install
  bash scripts/network/configure_zenoh.sh router-run
  bash scripts/network/configure_zenoh.sh robot <router-address> [port]
  bash scripts/network/configure_zenoh.sh mocap-source <natnet-server> <local-ip> NAME=/TOPIC [...]
  bash scripts/network/configure_zenoh.sh status

The router runs on the MoCap-side laptop. Each robot runs one client bridge.
Only /optitrack/rigid_bodies/orkar_agv* crosses Wi-Fi; robot sensor DDS stays
on loopback. Exactly one source host converts selected NatNet rigid bodies to
ROS 2 PoseStamped topics for the router.
EOF
}

sudo_run() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    elif sudo -n true 2>/dev/null; then
        sudo "$@"
    elif [ -n "${SUDO_PASSWORD:-}" ]; then
        printf '%s\n' "${SUDO_PASSWORD}" | sudo -S -p "" "$@"
    else
        sudo "$@"
    fi
}

release_asset() {
    case "$(uname -s)/$(uname -m)" in
        Darwin/arm64)
            echo "zenoh-plugin-ros2dds-${ZENOH_VERSION}-aarch64-apple-darwin-standalone.zip 997415721cfbb74b209b9968e7a7e4f6bed94e6afa4559ddb02ee1b2edccc899"
            ;;
        Darwin/x86_64)
            echo "zenoh-plugin-ros2dds-${ZENOH_VERSION}-x86_64-apple-darwin-standalone.zip 803a1f47bac6cc9dd13ec49e57f2b7e868f20a651fe72183b78d82a96222e949"
            ;;
        Linux/aarch64|Linux/arm64)
            echo "zenoh-plugin-ros2dds-${ZENOH_VERSION}-aarch64-unknown-linux-gnu-standalone.zip e3eb1fd4459e4b877653419b1c25eaf92418d70fe53ee767eca005f1a19443dc"
            ;;
        Linux/x86_64)
            echo "zenoh-plugin-ros2dds-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-standalone.zip 91aa0d569fffd57e7ebb1a591b97789891c543b1ff0a1658413ce6cbbba34a9e"
            ;;
        *)
            echo "ERROR: unsupported platform: $(uname -s)/$(uname -m)" >&2
            return 1
            ;;
    esac
}

sha256_file() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | awk '{print $1}'
    else
        shasum -a 256 "$1" | awk '{print $1}'
    fi
}

install_bridge_binary() {
    local destination="$1"
    local asset checksum actual archive extract_dir candidate tmpdir

    if [ -x "${destination}" ] && "${destination}" --version 2>&1 | grep -q "${ZENOH_VERSION}"; then
        echo "Zenoh bridge ${ZENOH_VERSION} already installed: ${destination}"
        return
    fi

    read -r asset checksum < <(release_asset)
    tmpdir="$(mktemp -d)"
    archive="${tmpdir}/${asset}"
    extract_dir="${tmpdir}/extract"
    echo "Downloading official zenoh-bridge-ros2dds ${ZENOH_VERSION}..."
    curl -fL --retry 3 -o "${archive}" "${RELEASE_BASE}/${asset}"
    actual="$(sha256_file "${archive}")"
    if [ "${actual}" != "${checksum}" ]; then
        echo "ERROR: Zenoh archive SHA256 mismatch." >&2
        echo "expected: ${checksum}" >&2
        echo "actual:   ${actual}" >&2
        exit 1
    fi

    mkdir -p "${extract_dir}"
    python3 - "${archive}" "${extract_dir}" <<'PY'
import sys
import zipfile

with zipfile.ZipFile(sys.argv[1]) as archive:
    archive.extractall(sys.argv[2])
PY
    candidate="$(find "${extract_dir}" -type f -name zenoh-bridge-ros2dds -print -quit)"
    if [ -z "${candidate}" ]; then
        echo "ERROR: official archive does not contain zenoh-bridge-ros2dds." >&2
        exit 1
    fi

    if [[ "${destination}" = /usr/* ]]; then
        sudo_run install -D -m 0755 "${candidate}" "${destination}"
    else
        mkdir -p "$(dirname "${destination}")"
        install -m 0755 "${candidate}" "${destination}"
    fi
    rm -rf "${tmpdir}"
    "${destination}" --version
}

install_router() {
    install_bridge_binary "${HOME}/.local/bin/zenoh-bridge-ros2dds"
    echo "Router installed. Start it with:"
    echo "  bash scripts/network/configure_zenoh.sh router-run"
}

run_router() {
    install_bridge_binary "${HOME}/.local/bin/zenoh-bridge-ros2dds"
    echo "Starting MoCap router on tcp/0.0.0.0:7447 (Ctrl+C to stop)."
    export ROS_DISTRO=humble
    export RUST_LOG="${ZENOH_RUST_LOG:-zenoh_plugin_ros2dds=info}"
    exec "${HOME}/.local/bin/zenoh-bridge-ros2dds" \
        -c "${ROOT}/configs/zenoh/mocap_router.json5"
}

install_robot() {
    local router="$1"
    local port="${2:-7447}"
    local robot_id="${ROBOT_ID:-$(hostname -s)}"
    local config_tmp env_tmp profile_tmp

    [[ "${router}" =~ ^[A-Za-z0-9._:-]+$ ]] || {
        echo "ERROR: invalid router address: ${router}" >&2
        exit 2
    }
    [[ "${port}" =~ ^[0-9]+$ ]] && [ "${port}" -ge 1 ] && [ "${port}" -le 65535 ] || {
        echo "ERROR: invalid router port: ${port}" >&2
        exit 2
    }
    [[ "${robot_id}" =~ ^agv[0-9]+$ ]] || {
        echo "ERROR: robot hostname must be agv<number>, got: ${robot_id}" >&2
        echo "       Set ROBOT_ID=agv<number> to override it." >&2
        exit 2
    }

    install_bridge_binary /usr/local/bin/zenoh-bridge-ros2dds
    config_tmp="$(mktemp)"
    sed "s/__ROBOT_ID__/${robot_id}/g" \
        "${ROOT}/configs/zenoh/robot_gt.json5" >"${config_tmp}"
    sudo_run install -D -m 0644 "${config_tmp}" /etc/orkar/zenoh_robot_gt.json5
    rm -f "${config_tmp}"
    sudo_run install -D -m 0644 "${ROOT}/configs/systemd/orkar-zenoh-gt.service" /etc/systemd/system/orkar-zenoh-gt.service

    env_tmp="$(mktemp)"
    cat >"${env_tmp}" <<EOF
ZENOH_ROUTER_ENDPOINT=tcp/${router}:${port}
EOF
    sudo_run install -D -m 0644 "${env_tmp}" /etc/orkar/zenoh.env
    cat >"${env_tmp}" <<'EOF'
ORKAR_ROS_TRANSPORT=zenoh-bridge-ros2dds
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=1
EOF
    sudo_run install -D -m 0644 "${env_tmp}" /etc/orkar/ros_transport.env
    rm -f "${env_tmp}"

    profile_tmp="$(mktemp)"
    cat >"${profile_tmp}" <<'EOF'
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
if [ -r /etc/orkar/ros_transport.env ]; then
    set -a
    . /etc/orkar/ros_transport.env
    set +a
fi
EOF
    sudo_run install -m 0644 "${profile_tmp}" /etc/profile.d/orkar-ros-transport.sh
    rm -f "${profile_tmp}"

    if [ -f /etc/profile.d/orkar-fastdds.sh ]; then
        sudo_run mv /etc/profile.d/orkar-fastdds.sh /etc/profile.d/orkar-fastdds.sh.disabled
    fi
    sudo_run systemctl disable --now orkar-fastdds-discovery.service 2>/dev/null || true
    sudo_run systemctl disable --now orkar-mocap-bridge.service 2>/dev/null || true
    sudo_run systemctl daemon-reload
    sudo_run systemctl enable orkar-zenoh-gt.service >/dev/null
    sudo_run systemctl restart orkar-zenoh-gt.service
    sudo_run systemctl is-active --quiet orkar-zenoh-gt.service

    local connected=false
    local attempt
    for attempt in $(seq 1 15); do
        if ss -Htnp state established 2>/dev/null | \
            grep -F ":${port}" | grep -q zenoh-bridge-ro; then
            connected=true
            break
        fi
        sleep 1
    done
    if [ "${connected}" != true ]; then
        echo "ERROR: Zenoh service is active but did not connect to tcp/${router}:${port}." >&2
        echo "       Start the router and verify the robot can reach its address." >&2
        sudo_run journalctl -u orkar-zenoh-gt.service -n 20 --no-pager >&2 || true
        exit 1
    fi

    echo "Zenoh robot bridge active -> tcp/${router}:${port}"
    echo "Open a new shell, or source scripts/network/load_ros_transport_env.sh."
}

install_mocap_source() {
    local server="$1"
    local local_ip="$2"
    shift 2
    local mapping mappings="" first_name="" first_topic="" env_tmp attempt

    [[ "${server}" =~ ^[A-Za-z0-9._:-]+$ ]] || {
        echo "ERROR: invalid NatNet server address: ${server}" >&2
        exit 2
    }
    [[ "${local_ip}" =~ ^[0-9a-fA-F:.]+$ ]] || {
        echo "ERROR: invalid local interface address: ${local_ip}" >&2
        exit 2
    }
    [ "$#" -gt 0 ] || {
        echo "ERROR: provide at least one NAME=/optitrack/rigid_bodies/NAME mapping." >&2
        exit 2
    }
    for mapping in "$@"; do
        [[ "${mapping}" =~ ^[A-Za-z0-9_.-]+=/optitrack/rigid_bodies/[A-Za-z0-9_.-]+$ ]] || {
            echo "ERROR: invalid rigid-body mapping: ${mapping}" >&2
            exit 2
        }
        mappings+="${mappings:+ }${mapping}"
        if [ -z "${first_name}" ]; then
            first_name="${mapping%%=*}"
            first_topic="${mapping#*=}"
        fi
    done

    if [ ! -f /opt/orkar/natnet-sdk-4.4/PythonClient/NatNetClient.py ]; then
        echo "ERROR: OptiTrack NatNet SDK PythonClient 4.4 is missing." >&2
        echo "       Expected /opt/orkar/natnet-sdk-4.4/PythonClient/NatNetClient.py" >&2
        exit 1
    fi

    echo "Probing a real tracked NatNet pose for ${first_name}..."
    if ! timeout 12 python3 "${ROOT}/scripts/mocap/natnet_watch.py" \
        --server "${server}" --local "${local_ip}" --name "${first_name}" --once; then
        echo "ERROR: NatNet did not deliver a tracked pose for ${first_name}." >&2
        exit 1
    fi

    env_tmp="$(mktemp)"
    cat >"${env_tmp}" <<EOF
NATNET_SERVER=${server}
NATNET_LOCAL_IP=${local_ip}
MOCAP_FRAME_ID=world
MOCAP_FRAME_TIMEOUT_SEC=5
MOCAP_RIGID_BODIES="${mappings}"
EOF
    sudo_run install -D -m 0644 "${env_tmp}" /etc/orkar/natnet_pose_source.env
    rm -f "${env_tmp}"
    sudo_run install -D -m 0644 \
        "${ROOT}/configs/systemd/orkar-natnet-pose-source.service" \
        /etc/systemd/system/orkar-natnet-pose-source.service
    sudo_run systemctl disable --now orkar-mocap-bridge.service 2>/dev/null || true
    sudo_run systemctl daemon-reload
    sudo_run systemctl enable orkar-natnet-pose-source.service >/dev/null
    sudo_run systemctl restart orkar-natnet-pose-source.service
    sudo_run systemctl is-active --quiet orkar-natnet-pose-source.service

    echo "Proving the supervised ROS 2 pose output on ${first_topic}..."
    for attempt in $(seq 1 3); do
        if (
            set +u
            # shellcheck disable=SC1091
            source /opt/ros/humble/setup.bash
            # shellcheck disable=SC1091
            source "${ROOT}/agv2_ws/install/setup.bash"
            set -u
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            export ROS_DOMAIN_ID=0
            export ROS_LOCALHOST_ONLY=0
            unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE ORKAR_ROS_TRANSPORT
            timeout 12 ros2 topic echo "${first_topic}" \
                --once --no-daemon --spin-time 5 >/dev/null
        ); then
            echo "NatNet pose source active: ${mappings}"
            return
        fi
        sleep 2
    done

    echo "ERROR: source service is active but ${first_topic} produced no sample." >&2
    sudo_run journalctl -u orkar-natnet-pose-source.service -n 40 --no-pager >&2 || true
    exit 1
}

show_status() {
    if [ -r /etc/orkar/ros_transport.env ]; then
        cat /etc/orkar/ros_transport.env
        cat /etc/orkar/zenoh.env
        systemctl --no-pager --full status orkar-zenoh-gt.service || true
    else
        "${HOME}/.local/bin/zenoh-bridge-ros2dds" --version 2>/dev/null || echo "Router binary is not installed."
        curl -fsS http://127.0.0.1:8000/@/local 2>/dev/null || true
    fi
    if [ -r /etc/orkar/natnet_pose_source.env ]; then
        echo "--- NatNet pose source ---"
        cat /etc/orkar/natnet_pose_source.env
        systemctl --no-pager --full status orkar-natnet-pose-source.service || true
    fi
}

case "${1:-}" in
    router-install)
        [ "$#" -eq 1 ] || { usage >&2; exit 2; }
        install_router
        ;;
    router-run)
        [ "$#" -eq 1 ] || { usage >&2; exit 2; }
        run_router
        ;;
    robot)
        [ "$#" -ge 2 ] && [ "$#" -le 3 ] || { usage >&2; exit 2; }
        install_robot "$2" "${3:-7447}"
        ;;
    mocap-source)
        [ "$#" -ge 4 ] || { usage >&2; exit 2; }
        install_mocap_source "${@:2}"
        ;;
    status)
        [ "$#" -eq 1 ] || { usage >&2; exit 2; }
        show_status
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac
