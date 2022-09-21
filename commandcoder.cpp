//  Copyright (C) 2021 Filippo Cona
//
//  This file is part of EDR4.
//
//  EDR4 is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  EDR4 is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with EDR4.  If not, see <http://www.gnu.org/licenses/>.

#include "commandcoder.h"
#include <cmath>

CommandCoder::CommandCoder(uint16_t initialByte, uint8_t initialBit, uint8_t bitsNum):
    initialByte(initialByte),
    bitsNum(bitsNum) {

    /*! bitsNum+initialBit to find the offset of the last bit
     *  /7 because 7 is the number of usable bits per byte
     *  +6 is used to make a ceil roduning of the division result,
     *  even if we're left with a single bit we need a byte to handle it */
    bytesNum = (bitsNum+initialBit+6)/7;
    bitOffsets.resize(bytesNum);
    bitMasks.resize(bytesNum);
    uint8_t bitsInNthByte;
    for (uint8_t byteIdx = 0; byteIdx < bytesNum; byteIdx++) {
        bitOffsets[byteIdx] = initialBit;
        bitsInNthByte = (bitsNum+initialBit < 7 ? bitsNum+initialBit : 7)-initialBit;
        bitMasks[byteIdx] = ((U8_1 << bitsInNthByte)-U8_1) << initialBit;
        bitsNum -= bitsInNthByte;
        initialBit = 0;
    }
}

CommandCoder::~CommandCoder() {

}

void CommandCoder::encodeUint(uint32_t uintValue, vector <uint8_t> &encodingBytes) {
    for (uint8_t byteIdx = 0; byteIdx < bytesNum; byteIdx++) {
        encodingBytes[byteIdx+initialByte] &= ~bitMasks[byteIdx];
        encodingBytes[byteIdx+initialByte] |= (uintValue << bitOffsets[byteIdx]) & bitMasks[byteIdx];
        uintValue >>= (7-bitOffsets[byteIdx]);
    }
}

BoolCoder::BoolCoder(CoderConfig_t config) :
    CommandCoder(config.initialByte, config.initialBit, config.bitsNum),
    config(config) {

}

BoolArrayCoder::BoolArrayCoder(CoderConfig_t config) :
    BoolCoder(config) {

}

void BoolArrayCoder::encode(uint32_t value, vector <uint8_t> &encodingBytes) {
    this->encodeUint(value, encodingBytes);
}

BoolNegatedArrayCoder::BoolNegatedArrayCoder(CoderConfig_t config) :
    BoolArrayCoder(config) {

}

void BoolNegatedArrayCoder::encode(uint32_t value, vector <uint8_t> &encodingBytes) {
//    uint32_t mask = ((uint32_t)1 << (config.bitsNum-1))-(uint32_t)1;
    this->encodeUint((~value)/*&mask*/, encodingBytes);
}

BoolRandomArrayCoder::BoolRandomArrayCoder(CoderConfig_t config) :
    BoolArrayCoder(config) {

    tos.resize(0);
    toNum = 0;
}

void BoolRandomArrayCoder::encode(uint32_t value, vector <uint8_t> &encodingBytes) {
    this->encodeUint(this->map(value), encodingBytes);
}

void BoolRandomArrayCoder::addMapItem(uint32_t to) {
    tos.push_back(to);
    toNum++;
}

uint32_t BoolRandomArrayCoder::map(uint32_t from) {
    if (from >= toNum) {
        from = 0;
    }
    return tos[from];
}

BoolOneHotCoder::BoolOneHotCoder(CoderConfig_t config) :
    BoolCoder(config) {

}

void BoolOneHotCoder::encode(uint32_t value, vector <uint8_t> &encodingBytes) {
    this->encodeUint(1 << value, encodingBytes);
}

DoubleCoder::DoubleCoder(CoderConfig_t config) :
    CommandCoder(config.initialByte, config.initialBit, config.bitsNum),
    config(config),
    resolution(config.resolution),
    minValue(config.minValue),
    maxValue(config.maxValue) {

}

double DoubleCoder::clip(double value) {
    return (value > maxValue ? maxValue : (value < minValue ? minValue : value));
}

DoubleTwosCompCoder::DoubleTwosCompCoder(CoderConfig_t config) :
    DoubleCoder(config) {

}

void DoubleTwosCompCoder::encode(double value, vector <uint8_t> &encodingBytes) {
    value = this->clip(value);
    int32_t intValue = (int32_t)round(value/resolution);
    this->encodeUint((uint32_t)intValue, encodingBytes);
}

DoubleOffsetBinaryCoder::DoubleOffsetBinaryCoder(CoderConfig_t config) :
    DoubleCoder(config) {

}

void DoubleOffsetBinaryCoder::encode(double value, vector <uint8_t> &encodingBytes) {
    value = this->clip(value);
    uint32_t uintValue = (uint32_t)round((value-minValue)/resolution);
    this->encodeUint(uintValue, encodingBytes);
}

DoubleSignAbsCoder::DoubleSignAbsCoder(CoderConfig_t config) :
    DoubleCoder(config) {

}

void DoubleSignAbsCoder::encode(double value, vector <uint8_t> &encodingBytes) {
    value = this->clip(value);
    uint32_t uintValue = (uint32_t)round(fabs(value)/resolution);
    uintValue += (value < 0.0 ? 1 << (bitsNum-1) : 0);
    this->encodeUint(uintValue, encodingBytes);
}
