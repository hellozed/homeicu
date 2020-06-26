#!/usr/bin/env python

# author: ZWang
#
# execute a tool to upload the binary to ESP32 board by OTA
# please modify the setting below

import os
import sys
import socket

import stat
import time
import datetime


ToolPath = "~/Documents/Arduino/hardware/espressif/esp32/tools/"
esp32_ip = socket.gethostbyname("homeicu.local")
BinaryFile = "../../homeicu-build/firmware.ino.bin" 

Say = "say -v Fred "
os.system(Say + "hi") 

def upload():
    shell_comand = ToolPath + "espota.py -r -d -t 3 -i " + esp32_ip + " -f " + BinaryFile
    # print(shell_comand)
    
    try:    
        for number in range(1,3):
            print("\nuploading ... ... (CTRL+\ to stop)\n")
            r= os.system(shell_comand)
            if (r!=0):
                
                s = "upload failed! try again"
                print(s)
                os.system(Say + s)      #voice alert
                print(s)
                time.sleep(1)  
            else:
                print(time.asctime(time.localtime(time.time())))
                s = "congratulation, the uploading succeed!"
                print(s)
                os.system(Say + s)      #voice alert
                return 0
     
                s = "cancel uploading!"
                print(s)
                os.system(Say + s)      #voice alert

        exit()    # upload failed for 3 times
    except KeyboardInterrupt:
        exit()

def main():
    print('''
===========================================================
OTA upload to board and monitor new binary! 
    
Command Line:
"ota.py"     upload and monitor.
"ota.py new" only monitor the next binary.
"ota.py now" only upload the current binary.

1. Ctrl+C to exit
2. if upload failed, please check firewall and network
===========================================================''')

    # if command line arguments contains "next"
    # the program will skip upload current binary
    skip_current_version = False
    for arg in sys.argv[1:]:
        if arg == "next":
            skip_current_version = True
            print("\nwait the next version binary... \n")

        if arg == "now":
            print("\n only upload current binary, then exit. \n")
            upload()

    if (skip_current_version == False):
        r= upload()
        if (r!=0): exit()

    fileStatsObj = os.stat(BinaryFile)
    loadedVersionTime = time.ctime(fileStatsObj[stat.ST_MTIME])

    try:    
        while(True):
            time.sleep(1)   #sleep 1 sencond
            
            try:
                fileStatsObj = os.stat(BinaryFile)
                newVersionTime = time.ctime(fileStatsObj[stat.ST_MTIME])
            except FileNotFoundError:
                print("waiting building process to be completed.")
                pass    
            
            if (loadedVersionTime != newVersionTime):
                print("new binary found! upload again...")
        
                upload()
        
                loadedVersionTime = newVersionTime
                print("\n\n wait the next version binary!\n")
    except KeyboardInterrupt:
        exit()

if __name__ == "__main__":
    # execute only if run as a script
    main()

