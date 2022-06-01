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

#ifndef FTDIEEPROMDEMO_H
#define FTDIEEPROMDEMO_H

#include "ftdieeprom.h"

class FtdiEepromDemo : public FtdiEeprom {
public:
    FtdiEepromDemo(string deviceId);
    ~FtdiEepromDemo();

    ErrorCodes_t openConnection(char channel = 'A') override;
    ErrorCodes_t closeConnection() override;
    ErrorCodes_t setVcOffset(uint16_t value) override;
protected:
    /*! FtdiEeprom interface */

    ErrorCodes_t loadData() override;
    ErrorCodes_t loadDeviceTuple() override;
    ErrorCodes_t loadVcOffset() override;
};

#endif // FTDIEEPROMDEMO_H
