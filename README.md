# REQUIREMENTS
Currently supported platforms:
- Windows 10+
- Linux Arm

# Windows
## Toolchain
Download Qt Creator with the Qt libraries 6.7.3 or newer and the compatibility with MSVC 2022 [Qt Community](https://www.qt.io/download-dev)
Download and install Visual Studio 2022 [Visual Studio Community](https://visualstudio.microsoft.com/it/vs/community/)
## Drivers
Install EDR4 to make sure all the needed drivers are correctly installed [elements-ic.com/downloads/](https://elements-ic.com/downloads/)
## Dependencies
The er4commlib requires the ftd2xx library and the libmpsse, both distributed by FTDI [www.ftdichip.com](https://www.ftdichip.com)
### ftd2xx
Download the archive from the following link: [ftd2xx static](https://www.dropbox.com/scl/fi/abiiaq99b0mb71map5uws/ftd2xx.zip?rlkey=jgul5rdh2twf5fq2wytuqs2yi&dl=0).
Extract it anywhere on the PC and save the location with the environment variable FTD2XX_PATH. FTD2XX_PATH will look something like ```C:\users\username\ftd2xx\```
### libmpsse
Download the archive from the following link: [libmpsse static](https://www.dropbox.com/scl/fi/ydh3575915kba4bcsyqgl/libmpsse.zip?rlkey=qouhy0s6f3p1534io83w5jrax&dl=0).
Extract it anywhere on the PC and save the location with the environment variable LIBMPSSE_PATH. LIBMPSSE_PATH will look something like ```C:\users\username\libmpsse\```
This version has been modified with respect to the library distributed by FTDI, since it loads the ftd2xx library statically, so the ftd2xx.dll file is not needed anymore

# Linux Arm
## Toolchain
Download Qt Creator with the Qt libraries 6.7.3 or newer [Qt Community](https://www.qt.io/download-dev)
## Drivers
## Dependencies
The er4commlib requires the ftd2xx library and the libmpsse, both distributed by FTDI [www.ftdichip.com](https://www.ftdichip.com)
### ftd2xx
Download the archive from the following link: [ftd2xx static](https://www.dropbox.com/scl/fi/ntb2198hfh7qtx2lcvngi/ftdi.zip?rlkey=f2ljzkbsfqrns5epjvyncgdta&st=at8wraxo&dl=0).
Extract it anywhere on the PC and save the location with the environment variable FTD2XX_PATH. FTD2XX_PATH will look something like ```/user/ftd2xx/```
### libmpsse
Download the archive from the following link: [libmpsse static](https://www.dropbox.com/scl/fi/3ng6cgsl177lr5pywrjzc/libMPSSE.zip?rlkey=g8jzv2lqe6ija5wcn28le9nc8&st=xhmt95sw&dl=0).
Extract it anywhere on the PC and save the location with the environment variable LIBMPSSE_PATH. LIBMPSSE_PATH will look something like ```/user/ftd2xx/```
This version has been modified with respect to the library distributed by FTDI, since it loads the ftd2xx library statically, so the ftd2xx.so file is not needed anymore

# BUILD
Open the project er4commLib.pro with Qt creator and click build
## Prebuilt
### Windows
The prebuilt library is available on Elements website [elements-ic.com/edr4/#api](https://elements-ic.com/edr4/#api)
### Linux Arm
The prebuilt library is available at the following link [elements-ic.com/edr4/#api](https://www.dropbox.com/scl/fi/vjm6y3hyal640p8gkanny/er4commLib.zip?rlkey=9qqkix7ifosbcf4y4a71nz78p&st=4ijbzgjb&dl=0)


