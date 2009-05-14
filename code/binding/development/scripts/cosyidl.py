#!/usr/bin/python

import sys, os, string

# Get a list of arguments.
argv = sys.argv

# Split arguments to omniidl arguments and -pn arguments.
omniidlArgs = ["omniidl"]
packageNames = []
headerSuffix = ".hh"
fileLocation = ""

for a in argv[1:]:
    if a.startswith("-pn"):
        packageNames.append(a[3:])
    else:
        if a.startswith("-Wbh="):
            headerSuffix = a[5:]
        elif a.startswith("-C"):
            fileLocation = a[2:]

        omniidlArgs.append(a)

# Run the IDL compiler.
omniStatus = os.system(string.join(omniidlArgs, ' '))

# Get the name of the input IDL file.
idlName = omniidlArgs[-1]
root, ext = os.path.splitext(os.path.basename(idlName))
headerName = fileLocation + "/" + root + headerSuffix

# If omniidl exited successfully, change all occurrences of old package names
# with new package names.
if omniStatus == 0:
    os.rename(headerName, headerName + ".old")
    outFile = open(headerName, 'w')
    for line in open(headerName + ".old"):
        for a in packageNames:
            oldName, newName = a.split('=')
            line = line.replace(oldName + headerSuffix, newName + headerSuffix)

        outFile.write(line)

    outFile.close()
    # Remove temporary file.
    os.remove(headerName + ".old")
