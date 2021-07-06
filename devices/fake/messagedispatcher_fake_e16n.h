#ifndef MESSAGEDISPATCHER_FAKE_E16N_H
#define MESSAGEDISPATCHER_FAKE_E16N_H

#include "messagedispatcher_e16n.h"

/********************************************************************************************\
 *                                                                                          *
 *                                 MessageDispatcherFake                                    *
 *                                                                                          *
\********************************************************************************************/

class MessageDispatcher_fake_e16n : public MessageDispatcher_e16n {
public:
    /*****************\
     *  Ctor / Dtor  *
    \*****************/

    MessageDispatcher_fake_e16n(string di);
    ~MessageDispatcher_fake_e16n();

    /************************\
     *  Connection methods  *
    \************************/

    ErrorCodes_t connect(FtdiEeprom * ftdiEeprom) override;
    ErrorCodes_t disconnect() override;

    /******************************\
     *  Tx methods for generator  *
    \******************************/

    ErrorCodes_t setCurrentRange(uint16_t currentRangeIdx, uint16_t channelIdx, bool applyFlag) override;
    ErrorCodes_t setSamplingRate(uint16_t samplingRateIdx, bool applyFlag) override;

protected:
    enum SamplingRates {
        SamplingRate1_25kHz,
        SamplingRate5kHz,
        SamplingRate10kHz,
        SamplingRate20kHz,
        SamplingRatesNum
    };

    typedef enum {
        ProtocolItemVStepTStep,
        ProtocolItemVRamp,
        ProtocolItemVSin,
        ProtocolItemIStepTStep,
        ProtocolItemIRamp,
        ProtocolItemISin
    } ProtocolItemType_t;

    typedef struct {
        ProtocolItemType_t type;
        double a0;
        double da;
        double b0;
        double db;
        double c0;
        double dc;
        unsigned short currentItem;
        unsigned short nextItem;
        unsigned short repsNum;
        bool applySteps;
    } ProtocolItem_t;

    typedef struct {
        unsigned short protocolId;
        unsigned short itemsNum;
        unsigned short sweepsNum;
        vector <ProtocolItem_t> items;
    } ProtocolStruct_t;

    void initializeDevice() override {};

    /****************\
     *  Parameters  *
    \****************/

    /***************\
     *  Variables  *
    \***************/

    unsigned int ftdiRxMsgBufferReadLength = 0;

    unsigned int genRxMsgBufferLen = 0;

    /*! Variable used to access the rx msg buffer */
    uint32_t rxMsgBufferWriteOffset = 0;

    /*! Rx heartbeat variable */
    uint16_t rxHeartbeat = 0x0000;

    /*! Variables used to access the rx data buffer */
    uint32_t rxDataBufferWriteOffset = 0;

    bool ackFromGenAvailable = false;
    uint16_t lastAckHbFromGen;

    bool protocolAvailable = false;
    bool protocolOpened = false;

    ProtocolStruct_t builtProtocol;
    ProtocolStruct_t pushedProtocol;
    ProtocolStruct_t poppedProtocol;

    /***********************\
     *  Signals variables  *
    \***********************/

    double genResistance = 50.0e6;

    RangedMeasurement_t genVcVoltageResolution;
    double genVoltage = 0.0;
    double genVoltageStep = 0.0;
    double genVoltageAmp = 0.0;
    double genVoltageNorm;
    uint16_t genVoltageInt;

    RangedMeasurement_t genVcCurrentResolution;
    double genCurrent = 0.0;
    double genCurrentStep = 0.0;
    double genCurrentAmp = 0.0;
    double genCurrentNorm;
    uint16_t genCurrentInt;

    double wFHN = 0.0;
    double dwFHN = 0.0;
    double vFHN = 0.0;
    double dvFHN = 0.0;
    const double aFHN = 0.0007;
    const double bFHN = 0.8;
    const double tFHN = 5.0;

    Measurement_t genSamplingRate;
    double samplingTime = 0.0001;
    double integrationStep = 0.01;
    int integrationItemStepsNum;

    bool saturationFlag = false;

    /********************************************\
     *  Multi-thread synchronization variables  *
    \********************************************/

    mutable mutex genRxMutex;
    condition_variable genRxMsgBufferNotEmpty;
    condition_variable genRxMsgBufferNotFull;

    mutable mutex genTxMutex;
    condition_variable genTxProtocolAvailable;

    mutable mutex genParamMutex;

    thread gnThread;
    thread satThread;
};

#endif // MESSAGEDISPATCHER_FAKE_E16N_H
