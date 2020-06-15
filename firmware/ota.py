#!/usr/bin/env python

# excute a tool to upload binary to ESP32 board by OTA
# please modify the setting below

import os

ToolPath = "~/Documents/Arduino/hardware/espressif/esp32/tools/"
ESP32_IP = "192.168.1.23"
BinaryFile = "../../homeicu-build/firmware.ino.bin" 

os.system(ToolPath + "espota.py -r -d -i " + ESP32_IP + " -f " + BinaryFile)