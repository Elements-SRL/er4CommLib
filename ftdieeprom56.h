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

#ifndef FTDIEEPROM56_H
#define FTDIEEPROM56_H

#include "ftdieeprom.h"

#define E56_DEVICE_VER_ADDR 0x074
#define E56_FW_VER_ADDR 0x0075
#define E56_VC_OFFSET_ADDR 0x07B

class FtdiEeprom56 : public FtdiEeprom {
public:
    FtdiEeprom56(string deviceId);
    ~FtdiEeprom56();

    ErrorCodes_t setVcOffset(unsigned short value) override;

protected:
    /*! FtdiEeprom interface */
    ErrorCodes_t loadData() override;
    ErrorCodes_t loadDeviceTuple() override;
    ErrorCodes_t loadVcOffset() override;
};

#endif // FTDIEEPROM56_H
