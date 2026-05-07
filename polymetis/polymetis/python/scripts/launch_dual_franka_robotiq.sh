#!/usr/bin/env bash

set -euo pipefail

# Launch two Franka arms and two Robotiq grippers.
#
# Defaults:
#   left Franka   172.16.0.2 -> Polymetis port 50051
#   left Robotiq  /dev/ttyUSB0 -> Polymetis port 50052
#   right Franka  172.16.0.3 -> Polymetis port 50053
#   right Robotiq /dev/ttyUSB1 -> Polymetis port 50054
#   dual ZeroRPC interface -> port 4243

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if [[ "${EUID}" -eq 0 ]]; then
    echo "Error: do not run this launcher with sudo."
    echo "Run it as your normal desktop user so new terminals can attach to your DBus/X session:"
    echo "  bash ${BASH_SOURCE[0]}"
    echo
    echo "If gripper serial permissions fail, fix them once outside this launcher:"
    echo "  sudo usermod -a -G dialout \$USER"
    echo "Then log out and back in."
    exit 1
fi

CONDA_ENV="${CONDA_ENV:-polymetis-local}"
LEFT_FRANKA_IP="${LEFT_FRANKA_IP:-172.16.0.2}"
RIGHT_FRANKA_IP="${RIGHT_FRANKA_IP:-172.16.0.3}"
LEFT_ARM_PORT="${LEFT_ARM_PORT:-50051}"
LEFT_GRIPPER_PORT="${LEFT_GRIPPER_PORT:-50052}"
RIGHT_ARM_PORT="${RIGHT_ARM_PORT:-50053}"
RIGHT_GRIPPER_PORT="${RIGHT_GRIPPER_PORT:-50054}"
LEFT_GRIPPER_DEV="${LEFT_GRIPPER_DEV:-/dev/ttyUSB0}"
RIGHT_GRIPPER_DEV="${RIGHT_GRIPPER_DEV:-/dev/ttyUSB1}"
DUAL_INTERFACE_PORT="${DUAL_INTERFACE_PORT:-4243}"
START_DUAL_INTERFACE="${START_DUAL_INTERFACE:-1}"
USE_REAL_TIME="${USE_REAL_TIME:-1}"
LOG_DIR="${LOG_DIR:-${SCRIPT_DIR}/logs/dual_franka_robotiq}"
REQUIRED_RT_PRIO="${REQUIRED_RT_PRIO:-80}"
ROBOT_READY_TIMEOUT="${ROBOT_READY_TIMEOUT:-60}"
CLEANUP_OLD_PROCESSES="${CLEANUP_OLD_PROCESSES:-1}"

echo "========================================"
echo "Launching dual Franka + dual Robotiq"
echo "========================================"
echo "Script dir: ${SCRIPT_DIR}"
echo "Conda env: ${CONDA_ENV}"
echo "Left arm: ${LEFT_FRANKA_IP} -> ${LEFT_ARM_PORT}"
echo "Left gripper: ${LEFT_GRIPPER_DEV} -> ${LEFT_GRIPPER_PORT}"
echo "Right arm: ${RIGHT_FRANKA_IP} -> ${RIGHT_ARM_PORT}"
echo "Right gripper: ${RIGHT_GRIPPER_DEV} -> ${RIGHT_GRIPPER_PORT}"
echo "Dual interface enabled: ${START_DUAL_INTERFACE}, port ${DUAL_INTERFACE_PORT}"
echo "Franka realtime sudo path enabled: ${USE_REAL_TIME}"
echo "Polymetis realtime without sudo: 1"
echo "Cleanup old processes: ${CLEANUP_OLD_PROCESSES}"
echo "Logs: ${LOG_DIR}"

for required_file in launch_robot.py launch_gripper.py launch_dual_franka_interface_server.py; do
    if [[ ! -f "${required_file}" ]]; then
        echo "Error: ${required_file} not found in ${SCRIPT_DIR}"
        exit 1
    fi
done

mkdir -p "${LOG_DIR}"
if [[ ! -w "${LOG_DIR}" ]]; then
    ORIGINAL_LOG_DIR="${LOG_DIR}"
    LOG_DIR="/tmp/dual_franka_robotiq_${USER}"
    mkdir -p "${LOG_DIR}"
    echo "Warning: log directory is not writable: ${ORIGINAL_LOG_DIR}"
    echo "This usually happens after running this launcher with sudo."
    echo "Using fallback log directory instead: ${LOG_DIR}"
    echo "To fix the repository log directory once:"
    echo "  sudo chown -R \$USER:\$USER ${SCRIPT_DIR}/logs"
    if [[ ! -w "${LOG_DIR}" ]]; then
        echo "Error: fallback log directory is also not writable: ${LOG_DIR}"
        exit 1
    fi
fi

if [[ "${USE_REAL_TIME}" != "0" && "${USE_REAL_TIME}" != "1" && "${USE_REAL_TIME}" != "true" && "${USE_REAL_TIME}" != "false" ]]; then
    echo "Error: USE_REAL_TIME must be 0/1 or true/false, got '${USE_REAL_TIME}'"
    exit 1
fi

if [[ "${USE_REAL_TIME}" == "1" ]]; then
    USE_REAL_TIME="true"
elif [[ "${USE_REAL_TIME}" == "0" ]]; then
    USE_REAL_TIME="false"
fi

if [[ "${CLEANUP_OLD_PROCESSES}" != "0" && "${CLEANUP_OLD_PROCESSES}" != "1" ]]; then
    echo "Error: CLEANUP_OLD_PROCESSES must be 0 or 1, got '${CLEANUP_OLD_PROCESSES}'"
    exit 1
fi

check_realtime_permissions() {
    local rt_prio_limit
    rt_prio_limit="$(ulimit -r)"

    if [[ "${rt_prio_limit}" == "unlimited" ]]; then
        return
    fi

    if [[ "${rt_prio_limit}" =~ ^[0-9]+$ && "${rt_prio_limit}" -ge "${REQUIRED_RT_PRIO}" ]]; then
        return
    fi

    echo "Error: current shell realtime priority limit is ${rt_prio_limit}, but libfranka needs at least ${REQUIRED_RT_PRIO}."
    echo "Do not fix this by running the launcher with sudo; configure realtime permissions for your normal user instead."
    echo
    echo "Run these once in a normal terminal:"
    echo "  sudo groupadd -f realtime"
    echo "  sudo usermod -a -G realtime,dialout \$USER"
    echo "  printf '@realtime - rtprio 99\\n@realtime - memlock unlimited\\n' | sudo tee /etc/security/limits.d/99-realtime.conf"
    echo
    echo "Then log out and log back in, and verify:"
    echo "  groups"
    echo "  ulimit -r"
    echo
    echo "Expected: groups contains realtime and dialout; ulimit -r is 99 or unlimited."
    exit 1
}

check_realtime_permissions

cleanup_old_processes() {
    if [[ "${CLEANUP_OLD_PROCESSES}" != "1" ]]; then
        return
    fi

    echo "Cleaning up stale user processes from previous launches..."
    local uid
    uid="$(id -u)"
    local patterns=(
        "launch_dual_franka_interface_server.py"
        "launch_robot.py"
        "launch_gripper.py"
        "franka_panda_client"
        "run_server"
    )

    for pattern in "${patterns[@]}"; do
        pkill -u "${uid}" -f "${pattern}" 2>/dev/null || true
    done
    sleep 1
}

cleanup_old_processes

check_port_free() {
    local port="$1"
    local label="$2"

    if command -v ss >/dev/null 2>&1; then
        if ss -ltn "( sport = :${port} )" | tail -n +2 | grep -q .; then
            echo "Error: port ${port} for ${label} is already in use."
            echo "Stop the old process first, for example: pkill -f run_server"
            exit 1
        fi
    elif command -v lsof >/dev/null 2>&1; then
        if lsof -iTCP:"${port}" -sTCP:LISTEN >/dev/null 2>&1; then
            echo "Error: port ${port} for ${label} is already in use."
            echo "Stop the old process first, for example: pkill -f run_server"
            exit 1
        fi
    else
        echo "Warning: neither ss nor lsof found; skipping port check for ${label}:${port}."
    fi
}

check_port_free "${LEFT_ARM_PORT}" "left Franka arm"
check_port_free "${LEFT_GRIPPER_PORT}" "left Robotiq gripper"
check_port_free "${RIGHT_ARM_PORT}" "right Franka arm"
check_port_free "${RIGHT_GRIPPER_PORT}" "right Robotiq gripper"
if [[ "${START_DUAL_INTERFACE}" == "1" ]]; then
    check_port_free "${DUAL_INTERFACE_PORT}" "dual Franka interface"
fi

for dev in "${LEFT_GRIPPER_DEV}" "${RIGHT_GRIPPER_DEV}"; do
    if [[ -e "${dev}" ]]; then
        if [[ ! -r "${dev}" || ! -w "${dev}" ]]; then
            echo "Error: current user cannot read/write ${dev}."
            echo "This script no longer runs sudo chmod during launch, because that can hide unsafe startup failures."
            echo "Fix it once outside this launcher, for example:"
            echo "  sudo usermod -a -G dialout \$USER"
            echo "Then log out and back in. For a temporary one-shot fix:"
            echo "  sudo chmod 666 ${dev}"
            exit 1
        fi
    else
        echo "Warning: ${dev} does not exist. Check USB device names before using the gripper."
    fi
done

child_pids=()
started_titles=()

run_python_in_env() {
    local python_code="$1"
    shift

    (
        cd "${SCRIPT_DIR}"
        set +u
        source ~/.bashrc
        if command -v conda >/dev/null 2>&1; then
            eval "$(conda shell.bash hook)"
            conda activate "${CONDA_ENV}"
        fi
        set -u
        unset LD_LIBRARY_PATH
        unset PYTHONPATH
        export PYTHONNOUSERSITE=1
        export POLYMETIS_REALTIME_NO_SUDO=1
        python -c "${python_code}" "$@"
    )
}

wait_for_robot_metadata() {
    local label="$1"
    local port="$2"

    echo "Waiting for ${label} robot metadata on localhost:${port}..."
    run_python_in_env '
import sys
import time
import grpc
from polymetis_pb2 import Empty
from polymetis_pb2_grpc import PolymetisControllerServerStub

port = int(sys.argv[1])
timeout = float(sys.argv[2])
deadline = time.time() + timeout
last_error = None

while time.time() < deadline:
    try:
        channel = grpc.insecure_channel(f"localhost:{port}")
        grpc.channel_ready_future(channel).result(timeout=1.0)
        stub = PolymetisControllerServerStub(channel)
        stub.GetRobotClientMetadata(Empty(), timeout=1.0)
        channel.close()
        sys.exit(0)
    except Exception as exc:
        last_error = exc
        time.sleep(1.0)

print(f"Timed out waiting for robot metadata on localhost:{port}: {last_error}", file=sys.stderr)
sys.exit(1)
' "${port}" "${ROBOT_READY_TIMEOUT}" || {
        echo "Error: ${label} robot server opened port ${port}, but its robot client is not ready."
        echo "Check this log before starting the dual interface:"
        echo "  ${LOG_DIR}/${label}-franka-arm.log"
        exit 1
    }
}

run_process() {
    local title="$1"
    local command="$2"
    local log_file="${LOG_DIR}/${title}.log"

    if ! touch "${log_file}" 2>/dev/null; then
        echo "Error: cannot write log file: ${log_file}"
        echo "This usually happens after running this launcher with sudo."
        echo "Fix ownership once:"
        echo "  sudo chown -R \$USER:\$USER ${SCRIPT_DIR}/logs"
        exit 1
    fi

    local shell_command="
        cd '${SCRIPT_DIR}'
        set +u
        source ~/.bashrc
        if command -v conda >/dev/null 2>&1; then
            eval \"\$(conda shell.bash hook)\"
            conda activate '${CONDA_ENV}'
        fi
        set -u
        unset LD_LIBRARY_PATH
        unset PYTHONPATH
        export PYTHONNOUSERSITE=1
        export POLYMETIS_REALTIME_NO_SUDO=1
        export DUAL_INTERFACE_PORT='${DUAL_INTERFACE_PORT}'
        echo '[${title}] starting at '\$(date -Is) | tee -a '${log_file}'
        ${command} 2>&1 | tee -a '${log_file}'
        status=\${PIPESTATUS[0]}
        echo '[${title}] exited with status '\${status}' at '\$(date -Is) | tee -a '${log_file}'
        exec bash
    "

    echo "----------------------------------------"
    echo "Starting ${title}"
    echo "Log: ${log_file}"

    local launched=0
    if command -v gnome-terminal >/dev/null 2>&1; then
        if gnome-terminal --title="${title}" -- bash -lc "${shell_command}"; then
            launched=1
        else
            echo "Warning: gnome-terminal failed for ${title}; falling back if possible."
        fi
    fi

    if [[ "${launched}" -eq 0 ]]; then
        if command -v xterm >/dev/null 2>&1; then
            xterm -T "${title}" -e bash -lc "${shell_command}" &
            child_pids+=("$!")
        else
            echo "No gnome-terminal/xterm found. Running ${title} in background in this terminal."
            bash -lc "${shell_command}" &
            child_pids+=("$!")
        fi
    fi

    started_titles+=("${title}")
}

run_process \
    "left-franka-arm" \
    "python launch_robot.py robot_client=franka_hardware port=${LEFT_ARM_PORT} use_real_time=${USE_REAL_TIME} robot_client.executable_cfg.robot_ip=${LEFT_FRANKA_IP}"

sleep 2

run_process \
    "right-franka-arm" \
    "python launch_robot.py robot_client=franka_hardware port=${RIGHT_ARM_PORT} use_real_time=${USE_REAL_TIME} robot_client.executable_cfg.robot_ip=${RIGHT_FRANKA_IP}"

sleep 2

run_process \
    "left-robotiq-gripper" \
    "python launch_gripper.py gripper=robotiq_2f port=${LEFT_GRIPPER_PORT} gripper.comport=${LEFT_GRIPPER_DEV}"

sleep 2

run_process \
    "right-robotiq-gripper" \
    "python launch_gripper.py gripper=robotiq_2f port=${RIGHT_GRIPPER_PORT} gripper.comport=${RIGHT_GRIPPER_DEV}"

sleep 2

if [[ "${START_DUAL_INTERFACE}" == "1" ]]; then
    wait_for_robot_metadata "left" "${LEFT_ARM_PORT}"
    wait_for_robot_metadata "right" "${RIGHT_ARM_PORT}"

    run_process \
        "dual-franka-interface" \
        "python launch_dual_franka_interface_server.py"
fi

echo "========================================"
echo "Launch commands have been issued."
echo "Started: ${started_titles[*]}"
echo "Use Ctrl+C in each terminal to stop."
echo "If stale servers remain, run without sudo first:"
echo "  pkill -f run_server"
echo "========================================"

if [[ ${#child_pids[@]} -gt 0 ]]; then
    echo "Background PIDs: ${child_pids[*]}"
fi
