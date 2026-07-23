#include "messagedispatcher_el03ce_char.h"

using namespace std;
#ifndef ER4COMMLIB_LABVIEW_WRAPPER
using namespace er4CommLib;
#endif

MessageDispatcher_EL03ce_Char_LegacyEdr3::MessageDispatcher_EL03ce_Char_LegacyEdr3(string id) :
    MessageDispatcherLegacyEdr3(id) {

    /************************\
     * Communication format *
    \************************/

    ftdiEepromId = FtdiEepromId56;
    rxChannel = 'A';
    txChannel = 'A';

    rxSyncWord = 0xFFFF0000;
    txSyncWord = 0x80;

    packetsPerFrame = 16;

    voltageChannelsNum = 1;
    currentChannelsNum = 1;
    totalChannelsNum = voltageChannelsNum+currentChannelsNum;

    readFrameLength = FTD_RX_SYNC_WORD_SIZE+(packetsPerFrame*(int)totalChannelsNum)*(int)FTD_RX_WORD_SIZE;

    infoStructSize = sizeof(InfoStruct_t);
    infoStructPtr = (uint8_t *)&infoStruct;

    maxOutputPacketsNum = ER4CL_DATA_ARRAY_SIZE/totalChannelsNum;

    txDataBytes = 22;

    /**********************\
     * Available settings *
    \**********************/

    /*! Current ranges */
    independentCurrentRangesFlag = false;
    currentRangesNum = CurrentRangesNum;
    currentRangesArray.resize(currentRangesNum);
    currentRangesArray[CurrentRange200pA].min = -200.0;
    currentRangesArray[CurrentRange200pA].max = 200.0;
    currentRangesArray[CurrentRange200pA].step = currentRangesArray[CurrentRange200pA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200pA].prefix = UnitPfxPico;
    currentRangesArray[CurrentRange200pA].unit = "A";
    currentRangesArray[CurrentRange2nA].min = -2.0;
    currentRangesArray[CurrentRange2nA].max = 2.0;
    currentRangesArray[CurrentRange2nA].step = currentRangesArray[CurrentRange2nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange2nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange2nA].unit = "A";
    currentRangesArray[CurrentRange20nA].min = -20.0;
    currentRangesArray[CurrentRange20nA].max = 20.0;
    currentRangesArray[CurrentRange20nA].step = currentRangesArray[CurrentRange20nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange20nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange20nA].unit = "A";
    currentRangesArray[CurrentRange200nA].min = -200.0;
    currentRangesArray[CurrentRange200nA].max = 200.0;
    currentRangesArray[CurrentRange200nA].step = currentRangesArray[CurrentRange200nA].max/SHORT_MAX;
    currentRangesArray[CurrentRange200nA].prefix = UnitPfxNano;
    currentRangesArray[CurrentRange200nA].unit = "A";
    defaultCurrentRangesIdx.resize(currentChannelsNum);
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        defaultCurrentRangesIdx[channelIdx] = CurrentRange200pA;
    }

    /*! Voltage ranges */
    voltageRangesNum = VoltageRangesNum;
    voltageRangesArray.resize(voltageRangesNum);
    voltageRangesArray[VoltageRange500mV].step = 1;
    voltageRangesArray[VoltageRange500mV].min = -512.0;
    voltageRangesArray[VoltageRange500mV].max = 511.0;
    voltageRangesArray[VoltageRange500mV].prefix = UnitPfxMilli;
    voltageRangesArray[VoltageRange500mV].unit = "V";
    defaultVoltageRangeIdx = VoltageRange500mV;
    rawVoltageZero = 512;

    /*! Sampling rates */
    samplingRatesNum = SamplingRatesNum;
    samplingRatesArray.resize(samplingRatesNum);
    samplingRatesArray[SamplingRate1_25kHz].value = 1.25;
    samplingRatesArray[SamplingRate1_25kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate1_25kHz].unit = "Hz";
    samplingRatesArray[SamplingRate5kHz].value = 5.0;
    samplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate5kHz].unit = "Hz";
    samplingRatesArray[SamplingRate10kHz].value = 10.0;
    samplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate10kHz].unit = "Hz";
    samplingRatesArray[SamplingRate20kHz].value = 20.0;
    samplingRatesArray[SamplingRate20kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate20kHz].unit = "Hz";
    samplingRatesArray[SamplingRate50kHz].value = 50.0;
    samplingRatesArray[SamplingRate50kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate50kHz].unit = "Hz";
    samplingRatesArray[SamplingRate100kHz].value = 100.0;
    samplingRatesArray[SamplingRate100kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate100kHz].unit = "Hz";
    samplingRatesArray[SamplingRate200kHz].value = 200.0;
    samplingRatesArray[SamplingRate200kHz].prefix = UnitPfxKilo;
    samplingRatesArray[SamplingRate200kHz].unit = "Hz";
    defaultSamplingRateIdx = SamplingRate1_25kHz;

    realSamplingRatesArray.resize(samplingRatesNum);
    realSamplingRatesArray[SamplingRate1_25kHz].value = 1.25e3/1024.0;
    realSamplingRatesArray[SamplingRate1_25kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate1_25kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate5kHz].value = 1.25e3/256.0;
    realSamplingRatesArray[SamplingRate5kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate5kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate10kHz].value = 1.25e3/128.0;
    realSamplingRatesArray[SamplingRate10kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate10kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate20kHz].value = 1.25e3/64.0;
    realSamplingRatesArray[SamplingRate20kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate20kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate50kHz].value = 50.0;
    realSamplingRatesArray[SamplingRate50kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate50kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate100kHz].value = 100.0;
    realSamplingRatesArray[SamplingRate100kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate100kHz].unit = "Hz";
    realSamplingRatesArray[SamplingRate200kHz].value = 200.0;
    realSamplingRatesArray[SamplingRate200kHz].prefix = UnitPfxKilo;
    realSamplingRatesArray[SamplingRate200kHz].unit = "Hz";

    integrationStepArray.resize(samplingRatesNum);
    integrationStepArray[SamplingRate1_25kHz].value = 1024.0/1.25;
    integrationStepArray[SamplingRate1_25kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate1_25kHz].unit = "s";
    integrationStepArray[SamplingRate5kHz].value = 256.0/1.25;
    integrationStepArray[SamplingRate5kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate5kHz].unit = "s";
    integrationStepArray[SamplingRate10kHz].value = 128.0/1.25;
    integrationStepArray[SamplingRate10kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate10kHz].unit = "s";
    integrationStepArray[SamplingRate20kHz].value = 64.0/1.25;
    integrationStepArray[SamplingRate20kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate20kHz].unit = "s";
    integrationStepArray[SamplingRate50kHz].value = 20.0;
    integrationStepArray[SamplingRate50kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate50kHz].unit = "s";
    integrationStepArray[SamplingRate100kHz].value = 10.0;
    integrationStepArray[SamplingRate100kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate100kHz].unit = "s";
    integrationStepArray[SamplingRate200kHz].value = 5.0;
    integrationStepArray[SamplingRate200kHz].prefix = UnitPfxMicro;
    integrationStepArray[SamplingRate200kHz].unit = "s";

    /*! Overampling ratios */
    oversamplingImplemented = false;
    oversamplingRatiosNum = OversamplingRatiosNum;
    oversamplingRatiosArray.resize(oversamplingRatiosNum);
    oversamplingRatiosArray[OversamplingRatioX1] = 1;

    /*! Voltage filters */
    dacIntFilterAvailable = true;
    voltageStimulusLpfOptionsNum = VoltageStimulusLpfsNum;
    voltageStimulusLpfOptions.resize(voltageStimulusLpfOptionsNum);
    voltageStimulusLpfOptions[VoltageStimulusLpf1kHz].value = 1.0;
    voltageStimulusLpfOptions[VoltageStimulusLpf1kHz].prefix = UnitPfxNone;
    voltageStimulusLpfOptions[VoltageStimulusLpf1kHz].unit = "kHz";
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].value = 10.0;
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].prefix = UnitPfxKilo;
    voltageStimulusLpfOptions[VoltageStimulusLpf10kHz].unit = "Hz";

    dacExtFilterAvailable = false;
    voltageReferenceLpfOptionsNum = VoltageReferenceLpfsNum;
    voltageReferenceLpfOptions.resize(voltageReferenceLpfOptionsNum);

    /*! Front end denoiser */
    ferdImplementedFlag = false;
    maxFerdSize = 512;

    /*! Default values */
    selectedVoltageRangeIdx = defaultVoltageRangeIdx;
    selectedCurrentRangesIdx = defaultCurrentRangesIdx;
    selectedSamplingRateIdx = defaultSamplingRateIdx;

    currentRanges.resize(currentChannelsNum);
    currentResolutions.resize(currentChannelsNum);
    for (uint16_t channelIdx = 0; channelIdx < currentChannelsNum; channelIdx++) {
        currentRanges[channelIdx] = currentRangesArray[selectedCurrentRangesIdx[channelIdx]];
        currentResolutions[channelIdx] = currentRangesArray[selectedCurrentRangesIdx[channelIdx]].step;
    }
    voltageRange = voltageRangesArray[selectedVoltageRangeIdx];
    voltageResolution = voltageRangesArray[selectedVoltageRangeIdx].step;
    baseSamplingRate = realSamplingRatesArray[selectedSamplingRateIdx];
    samplingRate = baseSamplingRate;
    integrationStep = integrationStepArray[selectedSamplingRateIdx];

    customOptionsNum = CustomOptionsNum;
    customOptionsNames.resize(customOptionsNum);
    customOptionsNames[CustomOptionCore] = "Read data from channel";
    customOptionsDescriptions.resize(customOptionsNum);
    customOptionsDescriptions[CustomOptionCore].resize(2);
    customOptionsDescriptions[CustomOptionCore][0] = "1";
    customOptionsDescriptions[CustomOptionCore][1] = "2";
    customOptionsDefault.resize(customOptionsNum);
    customOptionsDefault[CustomOptionCore] = 0;
    customOptionsNames[CustomOptionInputChannel] = "Close input switch on channel";
    customOptionsDescriptions[CustomOptionInputChannel].resize(2);
    customOptionsDescriptions[CustomOptionInputChannel][0] = "1";
    customOptionsDescriptions[CustomOptionInputChannel][1] = "2";
    customOptionsDefault[CustomOptionInputChannel] = 0;

    /*************\
     * Protocols *
    \*************/

    /*! Voltage ranges */
    protocolVoltageRangesArray.resize(ProtocolVoltageRangesNum);
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].min = -512.0;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].max = 511.0;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].step = 1.0;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].prefix = UnitPfxMilli;
    protocolVoltageRangesArray[ProtocolVoltageRange500mV].unit = "V";

    /*! Time ranges */
    protocolTimeRangesArray.resize(ProtocolTimeRangesNum);
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].max = 1000.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange2_10ms].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].min = 0.0;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].max = 200.0e6;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange0to2_28].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].max = 200.0e6;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1to2_28].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1orMore].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].max = (numeric_limits <double> ::max)();
    protocolTimeRangesArray[ProtocolTimeRange1orMore].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1orMore].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].min = -100.0e6;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].max = 100.0e6;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRangeSigned2_27].unit = "s";
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].min = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].max = 30.0e6;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].step = 1.0;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].prefix = UnitPfxMilli;
    protocolTimeRangesArray[ProtocolTimeRange1to2_25].unit = "s";

    /*! Protocol selection */
    protocolsNames.resize(ProtocolsNum);
    protocolsNames[ProtocolConstant] = "Constant";
    defaultProtocol = ProtocolConstant;
    selectedProtocol = defaultProtocol;
    triangularProtocolIdx = defaultProtocol;
    sealTestProtocolIdx = defaultProtocol;

    protocolsImages.resize(ProtocolsNum);
    protocolsImages[ProtocolConstant] = "holdingVoltage001";

    protocolsAvailableVoltages.resize(ProtocolsNum);
    protocolsAvailableTimes.resize(ProtocolsNum);
    protocolsAvailableSlopes.resize(ProtocolsNum);
    protocolsAvailableFrequencies.resize(ProtocolsNum);
    protocolsAvailableAdimensionals.resize(ProtocolsNum);

    protocolsAvailableVoltages[ProtocolConstant].push_back(ProtocolVHold);

    /*! Protocol voltages */
    protocolVoltagesNum = ProtocolVoltagesNum;
    protocolVoltageNames.resize(ProtocolVoltagesNum);
    protocolVoltageNames[ProtocolVHold] = "Vhold";

    protocolVoltageRanges.resize(ProtocolVoltagesNum);
    protocolVoltageRanges[ProtocolVHold].step = 1.0;
    protocolVoltageRanges[ProtocolVHold].min = voltageRangesArray[VoltageRange500mV].min;
    protocolVoltageRanges[ProtocolVHold].max = voltageRangesArray[VoltageRange500mV].max;
    protocolVoltageRanges[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageRanges[ProtocolVHold].unit = "V";

    protocolVoltageDefault.resize(ProtocolVoltagesNum);
    protocolVoltageDefault[ProtocolVHold].value = 0.0;
    protocolVoltageDefault[ProtocolVHold].prefix = UnitPfxMilli;
    protocolVoltageDefault[ProtocolVHold].unit = "V";
    selectedProtocolVoltage.resize(ProtocolVoltagesNum);
    for (unsigned int idx = 0; idx < ProtocolVoltagesNum; idx++) {
        selectedProtocolVoltage[idx] = protocolVoltageDefault[idx];
    }

    /*! Protocol times */
    protocolTimesNum = ProtocolTimesNum;
    protocolTimeNames.resize(ProtocolTimesNum);
    protocolTimeRanges.resize(ProtocolTimesNum);
    protocolTimeDefault.resize(ProtocolTimesNum);
    selectedProtocolTime.resize(ProtocolTimesNum);

    /*! Protocol adimensionals */
    protocolAdimensionalsNum = ProtocolAdimensionalsNum;
    protocolAdimensionalNames.resize(ProtocolAdimensionalsNum);
    protocolAdimensionalDefault.resize(ProtocolAdimensionalsNum);
    selectedProtocolAdimensional.resize(ProtocolAdimensionalsNum);

    voltageOffsetControlImplemented = false;
    selectedVoltageOffset.clear();

    /**************\
     * EDH format *
    \**************/

    edhFormat =
        "EDH Version: 2.0\n"
        "\n"
        "Elements EL03c characterizer\n"
        "Channels: 1\n"
        "\n"
        "Data header file\n"
        "\n"
        "Amplifier Setup\n"
        "Range: %currentRange%\n" // 200 pA
        "Sampling frequency (SR): %samplingRate%\n" // 1.25 kHz
        "Final Bandwidth: SR/2 (no filter)\n"
        "\n"
        "Acquisition start time: %dateHour%\n" // 04/11/2020 11:28:55.130
        "\n"
        "Active channels: %activeChannels%\n"; // 1

    /****************************\
     * Device specific controls *
    \****************************/

    /**********\
     * Coders *
    \**********/

    /*! Input controls */
    BoolCoder::CoderConfig_t boolConfig;
    DoubleCoder::CoderConfig_t doubleConfig;

    /*! Device reset */
    boolConfig.initialByte = 8;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    deviceResetCoder = new BoolArrayCoder(boolConfig);

    /*! Select stimulus channel */
    selectStimulusChannelFlag = false;
    singleChannelSSCFlag = false;

    /*! Digital offset compensations */
    digitalOffsetCompensationFlag = false;
    singleChannelDOCFlag = false;
    selectableDOCAutostopFlag = false;

    /*! Zap */
    zappableDeviceFlag = true;
    singleChannelZapFlag = true;

    boolConfig.initialByte = 7;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    zapCoder = new BoolArrayCoder(boolConfig);

    zapStates.resize(currentChannelsNum);
    for (unsigned int currentIdx = 0; currentIdx < currentChannelsNum; currentIdx++) {
        zapStates[currentIdx] = false;
    }

    /*! Channel off */
    channelOnFlag = false;
    singleChannelOnFlag = false;

    /*! Current range */
    boolConfig.initialByte = 8;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 2;
    BoolArrayCoder * range1Coder = new BoolArrayCoder(boolConfig);

    boolConfig.initialByte = 9;
    boolConfig.initialBit = 4;
    boolConfig.bitsNum = 1;
    BoolArrayCoder * range2Coder = new BoolArrayCoder(boolConfig);

    currentRangeCoders.resize(1);
    currentRangeCoders[0] = new EnsembleCoder();
    static_cast <EnsembleCoder *> (currentRangeCoders[0])->addCoder(range1Coder);
    static_cast <EnsembleCoder *> (currentRangeCoders[0])->addCoder(range2Coder);
    static_cast <EnsembleCoder *> (currentRangeCoders[0])->addMapItem(0); /*!< 200pA */
    static_cast <EnsembleCoder *> (currentRangeCoders[0])->addMapItem(1); /*!< 2nA   */
    static_cast <EnsembleCoder *> (currentRangeCoders[0])->addMapItem(3); /*!< 20nA  */
    static_cast <EnsembleCoder *> (currentRangeCoders[0])->addMapItem(7); /*!< 200nA */

    /*! Voltage range */
    boolConfig.initialByte = 0;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 1;
    voltageRangeCoder = new BoolArrayCoder(boolConfig);

    /*! Sampling rate */
    boolConfig.initialByte = 8;
    boolConfig.initialBit = 3;
    boolConfig.bitsNum = 4;
    samplingRateCoder = new BoolRandomArrayCoder(boolConfig);
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(0); /*!< 1.25kHz  -> 0b0000 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(1); /*!< 5kHz     -> 0b0001 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(2); /*!< 10kHz    -> 0b0010 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(3); /*!< 20kHz    -> 0b0011 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(8); /*!< 50kHz    -> 0b1000 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(9); /*!< 100kHz   -> 0b1001 */
    static_cast <BoolRandomArrayCoder *> (samplingRateCoder)->addMapItem(10); /*!< 200kHz  -> 0b1010 */

    /*! Protocol selection */
    boolConfig.initialByte = 10;
    boolConfig.initialBit = 0;
    boolConfig.bitsNum = 3;
    protocolsSelectCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol start */
    boolConfig.initialByte = 10;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    protocolStartCoder = new BoolArrayCoder(boolConfig);

    /*! Protocol voltages */
    protocolVoltageCoders.resize(ProtocolVoltagesNum);
    doubleConfig.initialByte = 1;
    doubleConfig.initialBit = 0;
    doubleConfig.bitsNum = 10;
    doubleConfig.minValue = protocolVoltageRanges[ProtocolVHold].min;
    doubleConfig.maxValue = protocolVoltageRanges[ProtocolVHold].max;
    doubleConfig.resolution = protocolVoltageRanges[ProtocolVHold].step;
    protocolVoltageCoders[ProtocolVHold] = new DoubleOffsetBinaryCoder(doubleConfig);

    /*! Protocol times */
    protocolTimeCoders.resize(ProtocolTimesNum);

    /*! Protocol Adimensionals */
    protocolAdimensionalCoders.resize(ProtocolAdimensionalsNum);

    /*! Internal DAC filter */
    boolConfig.initialByte = 9;
    boolConfig.initialBit = 2;
    boolConfig.bitsNum = 1;
    dacIntFilterCoder = new BoolNegatedArrayCoder(boolConfig);

    /*! Voltage offsets */
    voltageOffsetCoders.resize(currentChannelsNum);

    /*! Device specific controls */

    boolConfig.initialByte = 7;
    boolConfig.initialBit = 5;
    boolConfig.bitsNum = 1;
    customOptionsCoders.resize(customOptionsNum);
    customOptionsCoders[CustomOptionCore] = new BoolArrayCoder(boolConfig);

    boolConfig.initialByte = 7;
    boolConfig.initialBit = 1;
    boolConfig.bitsNum = 1;
    customOptionsCoders[CustomOptionInputChannel] = new BoolArrayCoder(boolConfig);

    /*******************\
     * Default status  *
    \*******************/

    txStatus.resize(txDataBytes);
    int txStatusIdx = 0;
    txStatus[txStatusIdx++] = txSyncWord; // HDR
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x04;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x40;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x0B;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
    txStatus[txStatusIdx++] = 0x00;
}

MessageDispatcher_EL03ce_Char_LegacyEdr3::~MessageDispatcher_EL03ce_Char_LegacyEdr3() {

}

void MessageDispatcher_EL03ce_Char_LegacyEdr3::initializeDevice() {
    this->setSamplingRate(defaultSamplingRateIdx, false);

    this->digitalOffsetCompensation(currentChannelsNum, false);

    MessageDispatcher::initializeDevice();
}

bool MessageDispatcher_EL03ce_Char_LegacyEdr3::checkProtocolValidity(string &message) {
    bool validFlag = true;
    message = "Valid protocol";
    switch (selectedProtocol) {
    case ProtocolConstant:
        if (!(protocolVoltageRangesArray[ProtocolVoltageRange500mV].includes(selectedProtocolVoltage[ProtocolVHold]))) {
            validFlag = false;
            message = "Vhold\nmust be within [-500,500]mV";

        } else {
            validFlag = true;
            message = "Valid protocol";
        }
        break;
    }
    return validFlag;
}
