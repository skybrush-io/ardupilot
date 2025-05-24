#!/bin/bash
#
# Helper script to build ArduCopter for multiple FMUs
#
# Officially supported configurations:
#
# sitl -- Software-in-the-loop simulator
# Pixhawk1 -- general Pixhawk1-based drones
# cmcopter -- for our own drones, which need a hack in the parsing of the PPMSum signals
# CubeBlack -- for Sparkl One
# CubeOrange
# CubeOrangePlus
# fmuv3, fmuv5 -- generic builds for Pixhawk FMU designs
# Pixhawk4 -- for the Holybro Pixhawk 4 (fmuv5 with a few tweaks)
# PH4-mini -- for the PixHawk 4 Mini
# Durandal -- for the Holybro Durandal
# lightdynamix-pixel -- for the Lightdynamix Pixel drone
# luminuousbee5 -- for LuminousBee5 outdoor
# QioTekZealotH743 -- donated to the Skybrush project
# speedybeef4v3 -- with disabled OSD support to make the firmware fit
# MatekH743, MatekH743-bdshot -- Matek H743 variants, without and with bidirectional DShot
#
# These variants are built but they are not officially supported:
#
# Pixhawk6C -- for the Holybro Pixhawk 6C and 6C Mini
# speedybeef4v4 -- lack of feedback from the community yet but probably works
#
# You may define the BOARDS= variable in the environment to override which
# boards to build for.

BOARDS=${BOARDS:-"sitl Pixhawk1 cmcopter CubeBlack CubeOrange CubeOrangePlus fmuv3 fmuv5 Pixhawk4 Pixhawk6C PH4-mini Durandal luminousbee5 2 QioTekZealotH743 MatekH743 MatekH743-bdshot lightdynamix-pixel speedybeef4v3 speedybeef4v4"}
ARM_TOOLCHAIN=${ARM_TOOLCHAIN:-"${HOME}/opt/toolchains/ardupilot"}

# Name of the folder to store compiled firmwares in
DIST_DIR=dist

set -e

# Declare that some of the boards are based on other boards so we can re-use
# their bootloaders. Note that we cannot re-use bootloaders in cases where
# the base bootloader was not built with flash-from-SD support if we enabled
# it by ourselves
BASE_BOARD_OF_cmcopter=Pixhawk1
BASE_BOARD_OF_lightdynamix_pixel=MatekH743-bdshot

cd "$(dirname $0)"

mkdir -p ${DIST_DIR}/

if [ "x$PYTHON" = x ]; then
    PYTHON=python3
fi

if [ ! -d .venv ]; then
    $PYTHON -m venv .venv
    .venv/bin/pip install -U pip wheel
    .venv/bin/pip install future "empy>=3,<4" intelhex pexpect
fi

export PATH=".venv/bin:$PATH"
if [ ! -d "${ARM_TOOLCHAIN}" ]; then
    echo "/!\\ ARM toolchain suggested by the ArduPilot developers is not installed."
    while true; do
        read -p "    Do you want to continue? [y/N] " yn
        case $yn in
        [Yy]*) break ;;
        [Nn]*) exit ;;
        *)
            echo ""
            echo "    Please respond with yes or no."
            ;;
        esac
    done
fi

if [ -d "${ARM_TOOLCHAIN}" ]; then
    export PATH="${ARM_TOOLCHAIN}/bin:$PATH"
fi

DATE=$(date +%Y%m%d)

source .venv/bin/activate

for BOARD in $BOARDS; do
    BOARD_LOWER=$(echo $BOARD | tr [[:upper:]] [[:lower:]])

    echo "Starting build for $BOARD..."
    echo ""

    rm -rf build/$BOARD

    # If the board is based on another board, copy the bootloader
    BASE_BOARD_VAR_NAME="BASE_BOARD_OF_$(echo "${BOARD}" | sed -e 's/-/_/g')"
    BASE_BOARD=${!BASE_BOARD_VAR_NAME}
    if [ x$BASE_BOARD != x ]; then
        rm -f Tools/bootloaders/${BOARD}_bl.*
        for fname in Tools/bootloaders/${BASE_BOARD}_bl.*; do
            EXT="${fname##*.}"
            cp $fname Tools/bootloaders/${BOARD}_bl.${EXT}
        done
    fi

    # If we have no bootloader yet, build from scratch
    if [ ! -f Tools/bootloaders/${BOARD}_bl.bin ]; then
        ./Tools/scripts/build_bootloaders.py ${BOARD}
    fi

    if [ x$BOARD = xcmcopter ]; then
        # The IOFMU code has to be rebuilt
        ./waf configure --board=$BOARD
        CXXFLAGS=-DCOLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE Tools/scripts/build_iofirmware.py
    else
        # Discard any changes made to the vendored IOFMU
        git restore Tools/IO_Firmware/*.bin
    fi

    ./waf configure --board=$BOARD
    ./waf copter
    if [ -f build/$BOARD/bin/arducopter.apj ]; then
        mkdir -p ${DIST_DIR}/${BOARD_LOWER}
        cp build/$BOARD/bin/arducopter.apj ${DIST_DIR}/${BOARD_LOWER}/arducopter-skybrush-${BOARD_LOWER}-$DATE.apj
    fi
    if [ -f build/$BOARD/bin/arducopter_with_bl.hex ]; then
        mkdir -p ${DIST_DIR}/${BOARD_LOWER}
        cp build/$BOARD/bin/arducopter_with_bl.hex ${DIST_DIR}/${BOARD_LOWER}/arducopter-skybrush-${BOARD_LOWER}-$DATE.hex
    fi
    if [ -f build/$BOARD/bin/arducopter.abin ]; then
        mkdir -p ${DIST_DIR}/${BOARD_LOWER}
        cp build/$BOARD/bin/arducopter.abin ${DIST_DIR}/${BOARD_LOWER}/arducopter-skybrush-${BOARD_LOWER}-$DATE.abin
    fi

    if [ x$BOARD = xcmcopter ]; then
        # Discard any changes made to the vendored IOFMU
        git restore Tools/IO_Firmware/*.bin
    fi

    # If the board is based on another board, remove the copied bootloader
    if [ x$BASE_BOARD != x ]; then
        rm -f Tools/bootloaders/${BOARD}_bl.*
    fi

    echo "Finished build for $BOARD."
    echo "====================================================================="
done

echo ""
echo "Compiled firmwares are in dist/"
