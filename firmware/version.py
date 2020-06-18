#!/usr/bin/env python

# generate version from git, and store it in version.h

import os
print("generate version from the git, and store it in the version.txt")

import subprocess

getVersion =  subprocess.Popen("git describe --tags", shell=True, stdout=subprocess.PIPE).stdout
getCommits =  subprocess.Popen("git rev-list HEAD | wc -l", shell=True, stdout=subprocess.PIPE).stdout

version =  getVersion.read()
commits =  getCommits.read()

# Opening a file 
file1 = open('version.h', 'w') 
  
# Writing a string to file 
version = "#define homeicu_version \"" + str(version.decode()).strip() + "\"\n"
commits = "#define homeicu_commits \"" + str(commits.decode()).strip() + "\"\n"

file1.write("// This version is generated from Git\n")
file1.write("// please run version.py again to get the correct version number \n")
file1.write("// before building the binary\n")
file1.write("\n\n")

file1.write(version)
file1.write(commits)
  
# Closing file 
file1.close() 