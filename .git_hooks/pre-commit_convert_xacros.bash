#!/usr/bin/env bash
# Pre-commit hook that convert xacros to URDF/SDF/SRDF. It makes sure that edits in xacros propagate to these files.

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPO_DIR="$(dirname "${SCRIPT_DIR}")"

# Convert xacros to URDF, SDF and SRDF
"${REPO_DIR}/panda_description/scripts/xacro2urdf.bash"
"${REPO_DIR}/panda_description/scripts/xacro2sdf.bash"
"${REPO_DIR}/panda_moveit_config/scripts/xacro2srdf.bash"
