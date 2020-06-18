#! /bin/bash

# clear the cache of Arduio IDE, VSCode, and Flutter

echo "delete all build out of : Arduio IDE, VSCode, and Flutter"

rm -rf /var/folders/zt/7cvp4v655ng7568vkml4bty00000gn/T/arduino_build_* 
rm -rf /Users/a123/code/homeicu-build 

echo "Done!"