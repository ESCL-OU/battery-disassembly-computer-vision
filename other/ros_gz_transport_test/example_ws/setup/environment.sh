SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ASSETS_PATH="../src/worlds/assets"

FULL_PATH=$SCRIPT_DIR/$ASSETS_PATH

export GZ_SIM_RESOURCE_PATH=$FULL_PATH

echo "GZ_SIM_RESOURCE_PATH set to: $FULL_PATH"
