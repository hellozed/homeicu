#!/usr/bin/env python

# execute a tool to upload the binary to ESP32 board by OTA
# please modify the setting below

import os

import socket

ToolPath = "~/Documents/Arduino/hardware/espressif/esp32/tools/"
esp32_ip = socket.gethostbyname("homeicu.local")
BinaryFile = "../../homeicu-build/firmware.ino.bin" 
shell_comand = ToolPath + "espota.py -r -d -t 3 -i " + esp32_ip + " -f " + BinaryFile
print(shell_comand)
os.system(shell_comand)