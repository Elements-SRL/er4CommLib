import os
import sys
import platform
from shutil import copyfile

# call: py copydlls.py release 32 <dll-path>

# build configuration
buildType = "Release" if sys.argv[1].lower() == "release" else "Debug"
archBits = int(sys.argv[2])
dllPath = sys.argv[3] + "/"

print("build: " + buildType + " " + str(archBits) + "bits")
lowerBuild = buildType.lower()
suffixBuild = "" if lowerBuild == "release" else "d"
archFolder = "x86/" if archBits == 32 else "x64/"

# OS configuration
configOS = "MAC" if platform.system() == 'Darwin' else ("WIN" if platform.system() == 'Windows' else "")
if configOS == "":
	exit(1)
osFolder = configOS.lower() + "/"
print("OS: " + configOS)

os.chdir("..")

headerFiles = ["er4commlib.h", "er4commlib_errorcodes.h", "er4commlib_global.h"]
appFolders = ["EDR4/device/commLib/"]

# copy dynamic libraries header files"
for headerFile in headerFiles:
	sourceFile = "er4commlib/" + headerFile
	print("\ncopying: " + sourceFile)
	for callerFolder in appFolders:
		destFile = callerFolder + headerFile
		copyfile(sourceFile, destFile)
		print("copied:  " + destFile)

# copy dynamic libraries"
file = "er4commlib" + suffixBuild
if configOS == "MAC":
	fullFiles = ["lib" + file + ".dylib", "lib" + file + ".1.dylib"]
	sourceFiles = []
	for fullFile in fullFiles:
		sourceFiles.append(dllPath + fullFile)
	
elif configOS == "WIN":
	if archBits == 32:
		fullFiles = [file + ".dll", "lib" + file + ".a"]
	else:
		fullFiles = [file + ".dll", file + ".lib"]
	sourceFiles = []
	for fullFile in fullFiles:
		sourceFiles.append(dllPath + lowerBuild + "/" + fullFile)
	
for idx, sourceFile in enumerate(sourceFiles):
	print("\ncopying: " + sourceFile)
	for callerFolder in appFolders:
		destFolder = callerFolder + osFolder + archFolder
		if not os.path.exists(destFolder):
			os.makedirs(destFolder)
		destFile = destFolder + fullFiles[idx]
		if os.path.isfile(sourceFile):
			copyfile(sourceFile, destFile)
			print("copied:  " + destFile)
		else:
			print(sourceFile + " does not exist")
				