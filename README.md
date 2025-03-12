# REQUIREMENTS
Currently supported platforms:
- Windows 10+
## Toolchain
Download Qt Creator with the Qt libraries 6.7.3 or newer and the compatibility with MSVC 2022 [Qt Community](https://www.qt.io/download-dev)
Download and install Visual Studio 2022 [Visual Studio Community](https://visualstudio.microsoft.com/it/vs/community/)
## Drivers
Install EDR4 to make sure all the needed drivers are correctly installed [elements-ic.com/downloads/](https://elements-ic.com/downloads/)
## Dependencies
The er4commlib requires the ftd2xx library and the libmpsse, both distributed by FTDI [www.ftdichip.com](https://www.ftdichip.com)
### ftd2xx
Download the archive from the following link: [ftd2xx static](https://www.dropbox.com/scl/fi/abiiaq99b0mb71map5uws/ftd2xx.zip?rlkey=jgul5rdh2twf5fq2wytuqs2yi&dl=0).
Extract it anywhere on the PC and save the location with the environment variable FTD2XX_PATH. FTD2XX_PATH will look something like C:\users\username\ftd2xx\
### libmpsse
Download the archive from the following link: [libmpsse static](https://www.dropbox.com/scl/fi/ydh3575915kba4bcsyqgl/libmpsse.zip?rlkey=qouhy0s6f3p1534io83w5jrax&dl=0).
Extract it anywhere on the PC and save the location with the environment variable LIBMPSSE_PATH. LIBMPSSE_PATH will look something like C:\users\username\libmpsse\
This version has been modified with respect to the library distributed by FTDI, since it loads the ftd2xx library statically, so the ftd2xx.dll file is not needed anymore
# BUILD
Open the project er4commLib.pro with Qt creator and click build
## Prebuilt
The prebuilt library is available on Elements website [elements-ic.com/edr4/#api](https://elements-ic.com/edr4/#api)
