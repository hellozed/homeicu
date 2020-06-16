#!/usr/bin/env python

# excute a tool to upload binary to ESP32 board through USBtoUART
# please modify the setting below

import os

print("Upload through USBtoUART without re-building")

tool = "/Users/a123/Documents/Arduino/hardware/espressif/esp32/tools/esptool/esptool "
para = "--chip esp32 --port /dev/cu.SLAB_USBtoUART --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect "
bin1 = "0xe000  /Users/a123/Documents/Arduino/hardware/espressif/esp32/tools/partitions/boot_app0.bin "
bin2 = "0x1000  /Users/a123/Documents/Arduino/hardware/espressif/esp32/tools/sdk/bin/bootloader_qio_80m.bin "
bin3 = "0x10000 /Users/a123/code/homeicu-build/firmware.ino.bin "
bin4 = "0x8000  /Users/a123/code/homeicu-build/firmware.ino.partitions.bin "

# the code below is for Arduino IDE built
# bin3 = "0x10000 /var/folders/zt/7cvp4v655ng7568vkml4bty00000gn/T/arduino_build_346603/firmware.ino.bin "
# bin4 = "0x8000 /var/folders/zt/7cvp4v655ng7568vkml4bty00000gn/T/arduino_build_346603/firmware.ino.partitions.bin "

shell_comand = tool+para+bin1+bin2+bin3+bin4
print(shell_comand)
os.system(shell_comand)
