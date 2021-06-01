#! /bin/bash


# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

set -e
set -x

rsync -a --exclude='temp' --delete Plugins/AirSim ../../Plugins/
rsync -a --exclude='*/src*' --delete Plugins/AirSim/Source/AirLib ../../../
rsync -a --exclude='*/src*' --delete Plugins/AirSim/Source/CustomizedInterpolation ../../../

popd >/dev/null
set +x
