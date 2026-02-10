#! /usr/bin/env bash

ask() {
    local reply

    echo -e -n "[INPUT] $1 [y/n] >> "
    read -r reply </dev/tty
    if [[ -z $reply ]]; then
        return 1;
    elif [ "$reply" == "y" ] || [ "$reply" == "Y" ]; then
        return 0;
    else
        return 1;
    fi
}

DEBUG=$2
dbg() {
    if [[ -n "${DEBUG}" ]]; then
        echo "[DBG] $1"
    fi
}

SCRIPT_NAME="$(readlink -f "$0")"
PKG_PATH="$(dirname "${SCRIPT_NAME}")"

dbg "Package path: ${PKG_PATH}"

cd ${PKG_PATH}
dbg "Current (PKG) directory: $(pwd)"

if ! $(git status >/dev/null 2>&1); then
    echo "[ERROR] Not a git repository..."
    exit 255
fi

GIT_REMOTE_URL=$(git config --get remote.origin.url)
if [[ "${GIT_REMOTE_URL}" == "https://github.com/tom-howard/ros2_pkg_template.git" ]]; then
    dbg "this is a clone of the package template"
    if [[ -z "$1" ]]; then
        dbg "No package name provided."
        echo -n "[INPUT] Please enter a name for your package >> "
        read -r PKG_NAME </dev/tty
        dbg "Using provided package name (after input): ${PKG_NAME}"
    else
        PKG_NAME="$1"
        dbg "Using provided package name: ${PKG_NAME}"
    fi
    FROM_TEMPLATE=True
else
    dbg "this is not the package template, preserve git, get repo name (or an alternative that is provided)."
    if [[ -n "$1" ]]; then
        PKG_NAME="$1"
        dbg "Using provided package name: ${PKG_NAME}"
    else
        dbg "Determining the package name from the git remote URL."
        PKG_NAME=$(basename "${GIT_REMOTE_URL}" ".git")
    fi
    FROM_TEMPLATE=False
fi

dbg "Package name: ${PKG_NAME}"

if [[ ! "${PKG_NAME}" =~ ^[a-z][a-z0-9_]*$ ]]; then
    echo "[ERROR] Invalid package name: '${PKG_NAME}'."
    echo "Package names must start with a lowercase letter, and can only contain lowercase letters, numbers, and underscores."
    exit 255
elif [[ "${PKG_NAME}" == "ros2_pkg_template" ]]; then
    echo "[ERROR] Package name cannot be 'ros2_pkg_template'. Please choose something different."
    exit 255
else
    dbg "Package name OK"
fi

PKG_IS_IN_A_WS=False
if [[ -z "${COLCON_PREFIX_PATH}" ]]; then
    dbg "No ROS 2 Workspace(s) detected."
else
    dbg "ROS 2 Workspace(s) detected: ${COLCON_PREFIX_PATH}"
    IFS=':' read -r -a COLCON_WS <<< "$COLCON_PREFIX_PATH"
    for ws in "${COLCON_WS[@]}"; do
        if [[ "${PKG_PATH}" == "$(dirname "${ws}")/src"* ]]; then
            PKG_IS_IN_A_WS=True
            dbg "PKG_PATH is inside workspace: $(dirname "${ws}")/src"
            break
        fi
    done
fi

if [[ "${PKG_IS_IN_A_WS}" == "True" ]]; then
    dbg "PKG_PATH is inside a ROS 2 workspace."
else
    if ! ask "This package doesn't appear to be inside a ROS 2 workspace. Are you sure you want to continue?"; then
        dbg "Not in a WS, don't continue."
        exit 255
    fi
    dbg "PKG_PATH is NOT inside any ROS 2 workspace, proceeding anyway."
fi

if ! ask "Initialise this ROS package with the name '${PKG_NAME}'?"; then
    echo "Exiting."
    exit 255
fi

echo "Initialising the '${PKG_NAME}' package..."

if [[ "${FROM_TEMPLATE}" == "True" ]]; then
    NEW_PKG_PATH="$(dirname "${PKG_PATH}")/${PKG_NAME}"
    dbg "Renaming package directory from '${PKG_PATH}'"
    dbg "New package location: ${NEW_PKG_PATH}"
    if [ -d "${NEW_PKG_PATH}" ]; then
        echo "[WARNING] The '${PKG_NAME}' ROS package (or a directory of the same name) already exists at '${NEW_PKG_PATH}'!"
        if ask "Do you want to overwrite it?"; then
            dbg "Removing ${NEW_PKG_PATH}..."
            rm -rf "${NEW_PKG_PATH}"
        else
            echo "Exiting."
            exit 255
        fi
    fi
fi

rm -f init_pkg.sh

TEMPLATE_NAME=ros2_pkg_template
mv ${TEMPLATE_NAME}_modules/ ${PKG_NAME}_modules/
sed -i '/<name>/s/'${TEMPLATE_NAME}'/'${PKG_NAME}'/' package.xml
sed -i '2 s/'${TEMPLATE_NAME}'/'${PKG_NAME}'/' CMakeLists.txt

if [[ "${FROM_TEMPLATE}" == "True" ]]; then
    rm -rf .git*
    cd $(dirname "${PKG_PATH}")
    mv "${PKG_PATH}" "${NEW_PKG_PATH}"
fi 

echo "Done."
