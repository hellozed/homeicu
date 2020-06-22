#!/usr/bin/env python

# generate version from git, and store it in version.h
#
# author: ZWang

import os
import subprocess

print("\n\ngenerate version from the git, and store it in the version.txt")

print("git fetch ...")
os.system("git fetch")      # tag is added in the github side, use this command to get tag back

print("writing to version.h")
getVersion =  subprocess.Popen("git describe --tags", shell=True, stdout=subprocess.PIPE).stdout
getCommits =  subprocess.Popen("git rev-list HEAD | wc -l", shell=True, stdout=subprocess.PIPE).stdout

version =  getVersion.read()
commits =  getCommits.read()

version =  str(version.decode()).strip()
commits =  str(commits.decode()).strip()

print ("version  =" + version)
print ("commits  =" + commits)

# Opening a file 
file1 = open('version.h', 'w') 
  
# Writing a string to file 
version = "#define homeicu_version \"" + version + "\"\n"
commits = "#define homeicu_commits \"" + commits + "\"\n"

file1.write("// This version is generated from Git\n")
file1.write("// please run version.py again to get the correct version number \n")
file1.write("// before building the binary\n")
file1.write("\n\n")

file1.write(version)
file1.write(commits)
  
# Closing file 
file1.close() 

print("done!")