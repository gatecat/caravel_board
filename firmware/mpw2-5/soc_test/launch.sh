#!/usr/bin/env bash
set -ex

export FLASH=/run/media/gatecat/PYBFLASH
export DEV=`mpremote connect list | grep STLink | cut -f 1 -d ' '`

mpy-cross soc_driver.py
cp soc_driver.mpy ${FLASH}
sync
mpremote connect ${DEV} exec "import soc_driver; soc_driver.run()"
