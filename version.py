#!/usr/bin/env python
# generate version from git, and store in version.h

import os
print("generate version from the git, and store it in the version.txt")

import subprocess

getVersion =  subprocess.Popen("git describe --tags", shell=True, stdout=subprocess.PIPE).stdout
getCommits =  subprocess.Popen("git rev-list HEAD | wc -l", shell=True, stdout=subprocess.PIPE).stdout

version =  getVersion.read()
commits =  getCommits.read()

# Opening a file 
file1 = open('version.txt', 'w') 
  
# Writing a string to file 
version = "#define homeicu_version = \"" + str(version.decode()).strip() + "\"\n"
commits = "#define homeicu_commits = \"" + str(commits.decode()).strip() + "\"\n"
file1.write(version)
file1.write(commits)
  
# Closing file 
file1.close() 