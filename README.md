# REQUIREMENTS
Compatibility has been verified for the following systems:
- Windows 10+
- Linux Arm

Note: Elements only provides free support for Windows 11+.

## C++
er4commlib requires c++17 or newer.

# BUILD
## Toolchain
Elements builds the er4commlib statically using Qt Creator with the Qt libraries 6.7.3 and MSVC 2022 [Qt Community](https://www.qt.io/download-dev)

Note: the Qt Libraries are not used. Qt Creator is used just to compile on different platforms seamlessly.

## Drivers
Install EDR4 to make sure all the needed drivers are correctly installed [elements-ic.com](https://elements-ic.com/downloads/)

## Dependencies
The er4commlib requires the ftdi_utils library developed by Elements [github.com](https://github.com/Elements-SRL/ftdi_utils/)

# PREBUILT BINARIES
Prebuilt versions of the library are available on Elements website [elements-ic.com](https://elements-ic.com/edr4/#api)
