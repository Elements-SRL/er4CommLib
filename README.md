# er4commLib
## Requirements
Currently supported platforms:
- Windows 10+
### Drivers
Install EDR4 to make sure all the needed drivers are correctly installed [elements-ic.com/downloads/](https://elements-ic.com/downloads/)

## Working with the Library
### Prebuilt
The prebuilt library is available on Elements website [elements-ic.com/emcr/#api](https://elements-ic.com/edr4/#api)
### Compilation
#### Requirements
Download the needed libraries and the corresponding header files from the following link [https://elements-ic.com/er4_distributables/](https://elements-ic.com/er4_distributables/)

**The archive contains everything you need to compile the project**, in particular there will also be third party dynamic libraries such as:
- ftd2xx.dll: developed and released by FTDI [www.ftdichip.com](https://www.ftdichip.com)
  
Make sure to add these libraries to the system path or the final application path.

#### Project configuration
To compile the library, the recommended settings can be found in the provided sln file.

For the provided sln to correctly work, set the following environmental variables to match the location on your computer of the downloaded libraries:
- FTD2XX_PATH
