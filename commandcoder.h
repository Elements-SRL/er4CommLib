#ifndef COMMANDCODER_H
#define COMMANDCODER_H

#include <stdint.h>
#include <vector>

#include <er4commlib_global.h>

using namespace std;

class CommandCoder {
public:
    CommandCoder(uint16_t initialByte, uint8_t initialBit, uint8_t bitsNum);
    ~CommandCoder();

protected:
    void encodeUint(uint32_t uintValue, vector <uint8_t> &encodingBytes);

    uint16_t initialByte;
    uint8_t bitsNum;
    uint8_t bytesNum;
    vector <uint8_t> bitOffsets;
    vector <uint8_t> bitMasks;
};

class BoolCoder : public CommandCoder {
public:
    typedef struct {
        uint16_t initialByte;
        uint8_t initialBit;
        uint8_t bitsNum;
    } CoderConfig_t;

    BoolCoder(CoderConfig_t config);

    virtual void encode(uint32_t value, vector <uint8_t> &encodingBytes) = 0;

protected:
    CoderConfig_t config;
};

class BoolArrayCoder : public BoolCoder {
public:
    BoolArrayCoder(CoderConfig_t config);

    void encode(uint32_t value, vector <uint8_t> &encodingBytes) override;
};

class BoolNegatedArrayCoder : public BoolArrayCoder {
public:
    BoolNegatedArrayCoder(CoderConfig_t config);

    void encode(uint32_t value, vector <uint8_t> &encodingBytes) override;
};

class BoolRandomArrayCoder : public BoolArrayCoder {
public:
    BoolRandomArrayCoder(CoderConfig_t config);

    void encode(uint32_t value, vector <uint8_t> &encodingBytes) override;
    void addMapItem(uint32_t to);

private:
    uint32_t map(uint32_t from);

    vector <uint32_t> tos;
    uint32_t toNum;
};

class BoolOneHotCoder : public BoolCoder {
public:
    BoolOneHotCoder(CoderConfig_t config);

    void encode(uint32_t value, vector <uint8_t> &encodingBytes) override;
};

class DoubleCoder : public CommandCoder {
public:
    typedef struct {
        uint16_t initialByte;
        uint8_t initialBit;
        uint8_t bitsNum;
        double resolution;
        double minValue;
        double maxValue;
        double offset;
    } CoderConfig_t;

    DoubleCoder(CoderConfig_t config);

    virtual void encode(double value, vector <uint8_t> &encodingBytes) = 0;

protected:
    double clip(double value);

    CoderConfig_t config;

    double resolution;
    double minValue;
    double maxValue;
    double offset;
};

class DoubleTwosCompCoder : public DoubleCoder {
public:
    DoubleTwosCompCoder(CoderConfig_t config);

    void encode(double value, vector <uint8_t> &encodingBytes) override;
};

class DoubleOffsetBinaryCoder : public DoubleCoder {
public:
    DoubleOffsetBinaryCoder(CoderConfig_t config);

    void encode(double value, vector <uint8_t> &encodingBytes) override;
};

class DoubleSignAbsCoder : public DoubleCoder {
public:
    DoubleSignAbsCoder(CoderConfig_t config);

    void encode(double value, vector <uint8_t> &encodingBytes) override;
};

#endif // COMMANDCODER_H
