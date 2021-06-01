#! /bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

set -e
set -x

./clean.sh

rsync -a --exclude 'temp' --delete ../../Plugins/AirSim Plugins/
rsync -a --exclude='*/src' --delete ../../../AirLib Plugins/AirSim/Source/
rsync -a --exclude='*/src' --delete ../../../CustomizedInterpolation Plugins/AirSim/Source/

popd >/dev/null