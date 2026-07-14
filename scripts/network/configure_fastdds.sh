#!/usr/bin/env bash
# Configure deterministic Fast DDS discovery and the direct NatNet bridge.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SERVER_ID=0

usage() {
    cat <<'EOF'
Usage:
  bash scripts/network/configure_fastdds.sh client <server-address> [port]
  bash scripts/network/configure_fastdds.sh server <advertised-address> [port]
  bash scripts/network/configure_fastdds.sh bridge <natnet-server> NAME=/gt/ROBOT/pose [...]
  bash scripts/network/configure_fastdds.sh status

Examples:
  bash scripts/network/configure_fastdds.sh server SERVER_IP 11811
  bash scripts/network/configure_fastdds.sh client SERVER_IP 11811
  bash scripts/network/configure_fastdds.sh bridge MOTIVE_IP \
    RIGID_BODY=/gt/ROBOT/pose
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

install_text() {
    local path="$1"
    local mode="$2"
    local content="$3"
    local tmp
    tmp="$(mktemp)"
    printf '%s\n' "${content}" > "${tmp}"
    sudo_run install -D -m "${mode}" "${tmp}" "${path}"
    rm -f "${tmp}"
}

validate_host() {
    [[ "$1" =~ ^[A-Za-z0-9._:-]+$ ]] || {
        echo "ERROR: invalid host/address: $1" >&2
        exit 2
    }
}

validate_port() {
    [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -ge 1 ] && [ "$1" -le 65535 ] || {
        echo "ERROR: invalid UDP port: $1" >&2
        exit 2
    }
}

configure_client() {
    local server="$1"
    local port="${2:-11811}"
    validate_host "${server}"
    validate_port "${port}"

    install_text /etc/orkar/fastdds.env 0644 \
"RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DISCOVERY_SERVER=${server}:${port}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
ROS_LOCALHOST_ONLY=0"

    install_text /etc/profile.d/orkar-fastdds.sh 0644 \
'if [ -r /etc/orkar/fastdds.env ]; then
    set -a
    . /etc/orkar/fastdds.env
    set +a
fi'

    install_text /etc/orkar/fastdds_super_client.xml 0644 \
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<dds>
  <profiles xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\">
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>orkar_udp</transport_id>
        <type>UDPv4</type>
        <sendBufferSize>1048576</sendBufferSize>
        <receiveBufferSize>1048576</receiveBufferSize>
      </transport_descriptor>
      <transport_descriptor>
        <transport_id>orkar_large_shm</transport_id>
        <type>SHM</type>
        <maxMessageSize>4194304</maxMessageSize>
        <segment_size>16777216</segment_size>
        <port_queue_capacity>1024</port_queue_capacity>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name=\"super_client_profile\" is_default_profile=\"true\">
      <rtps>
        <userTransports>
          <transport_id>orkar_udp</transport_id>
          <transport_id>orkar_large_shm</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
        <builtin>
          <discovery_config>
            <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
            <discoveryServersList>
              <RemoteServer prefix=\"44.53.00.5f.45.50.52.4f.53.49.4d.41\">
                <metatrafficUnicastLocatorList>
                  <locator><udpv4><address>${server}</address><port>${port}</port></udpv4></locator>
                </metatrafficUnicastLocatorList>
              </RemoteServer>
            </discoveryServersList>
          </discovery_config>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>"

    if command -v ros2 >/dev/null 2>&1; then
        ros2 daemon stop >/dev/null 2>&1 || true
    fi
    echo "Fast DDS client -> ${server}:${port}"
    echo "Open a new shell, or source /etc/orkar/fastdds.env with set -a/set +a."
}

configure_server() {
    local advertised="$1"
    local port="${2:-11811}"
    configure_client "${advertised}" "${port}"
    [ -x "/opt/ros/${ROS_DISTRO}/bin/fastdds" ] || {
        echo "ERROR: /opt/ros/${ROS_DISTRO}/bin/fastdds is missing." >&2
        exit 1
    }
    install_text /etc/systemd/system/orkar-fastdds-discovery.service 0644 \
"[Unit]
Description=ORKAR Fast DDS Discovery Server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash && /bin/bash /opt/ros/${ROS_DISTRO}/bin/fastdds discovery -i ${SERVER_ID} -p ${port}'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target"
    sudo_run systemctl daemon-reload
    sudo_run systemctl enable --now orkar-fastdds-discovery.service
    echo "Fast DDS discovery server listening on UDP ${port}; clients use ${advertised}:${port}."
}

configure_bridge() {
    local natnet_server="$1"
    shift
    validate_host "${natnet_server}"
    [ "$#" -gt 0 ] || {
        echo "ERROR: provide at least one NAME=/gt/ROBOT/pose mapping." >&2
        exit 2
    }
    local mapping mappings="" names=""
    for mapping in "$@"; do
        [[ "${mapping}" =~ ^[A-Za-z0-9_.-]+=/gt/[A-Za-z0-9_.-]+/pose$ ]] || {
            echo "ERROR: invalid rigid-body mapping: ${mapping}" >&2
            exit 2
        }
        names+="${names:+ }${mapping%%=*}"
        mappings+="${mappings:+ }${mapping}"
    done
    python3 -m pip install --user --disable-pip-version-check "natnet==0.2.0"
    local multicast=""
    case "${NATNET_MODE:-auto}" in
        auto)
            local probe_name found=false
            echo "Probing NatNet unicast for active configured bodies..."
            for probe_name in ${names}; do
                if timeout 6 python3 "${ROOT}/scripts/mocap/natnet_watch.py" \
                    --server "${natnet_server}" --name "${probe_name}" --once >/dev/null 2>&1; then
                    multicast=false
                    found=true
                    echo "NatNet unicast frame received for ${probe_name}."
                    break
                fi
            done
            if [ "${found}" != true ]; then
                echo "Unicast frames absent; probing NatNet multicast..."
                for probe_name in ${names}; do
                    if timeout 6 python3 "${ROOT}/scripts/mocap/natnet_watch.py" \
                        --server "${natnet_server}" --name "${probe_name}" --multicast --once >/dev/null 2>&1; then
                        multicast=true
                        found=true
                        echo "NatNet multicast frame received for ${probe_name}."
                        break
                    fi
                done
            fi
            if [ "${found}" != true ]; then
                echo "ERROR: neither NatNet unicast nor multicast produced an active configured body (${names})." >&2
                exit 1
            fi
            ;;
        unicast) multicast=false ;;
        multicast) multicast=true ;;
        *)
            echo "ERROR: NATNET_MODE must be auto, unicast, or multicast." >&2
            exit 2
            ;;
    esac
    install_text /etc/orkar/mocap_bridge.env 0644 \
"NATNET_SERVER=${natnet_server}
NATNET_MULTICAST=${multicast}
MOCAP_FRAME_ID=world
MOCAP_FRAME_TIMEOUT_SEC=5
MOCAP_RIGID_BODIES=\"${mappings}\""
    install_text /etc/systemd/system/orkar-mocap-bridge.service 0644 \
"[Unit]
Description=ORKAR NatNet to ROS 2 MoCap Bridge
After=network-online.target orkar-fastdds-discovery.service
Wants=network-online.target

[Service]
Type=simple
User=${USER}
WorkingDirectory=${ROOT}
Environment=HOME=${HOME}
EnvironmentFile=/etc/orkar/fastdds.env
EnvironmentFile=/etc/orkar/mocap_bridge.env
ExecStart=/usr/bin/bash ${ROOT}/scripts/network/run_mocap_bridge.sh
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target"
    sudo_run systemctl daemon-reload
    sudo_run systemctl enable --now orkar-mocap-bridge.service
    local mode_name=unicast
    [ "${multicast}" = true ] && mode_name=multicast
    echo "MoCap bridge configured (${mode_name}): ${mappings}"
}

show_status() {
    echo "--- client configuration ---"
    if [ -r /etc/orkar/fastdds.env ]; then
        sed -n '1,20p' /etc/orkar/fastdds.env
    else
        echo "not configured"
    fi
    echo "--- services ---"
    systemctl --no-pager --full status orkar-fastdds-discovery.service 2>/dev/null | sed -n '1,18p' || true
    systemctl --no-pager --full status orkar-mocap-bridge.service 2>/dev/null | sed -n '1,24p' || true
}

mode="${1:-}"
case "${mode}" in
    client)
        [ "$#" -ge 2 ] && [ "$#" -le 3 ] || { usage >&2; exit 2; }
        configure_client "$2" "${3:-11811}"
        ;;
    server)
        [ "$#" -ge 2 ] && [ "$#" -le 3 ] || { usage >&2; exit 2; }
        configure_server "$2" "${3:-11811}"
        ;;
    bridge)
        [ "$#" -ge 3 ] || { usage >&2; exit 2; }
        shift
        configure_bridge "$@"
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
