#   Copyright (C) 2021 Filippo Cona
# 
#   This file is part of EDR4.
# 
#   EDR4 is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
# 
#   EDR4 is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
# 
#   You should have received a copy of the GNU Lesser General Public License
#   along with EDR4.  If not, see <http://www.gnu.org/licenses/>.

INCLUDEPATH += $$PWD/
DEPENDPATH += $$PWD/

mingw:win32:CONFIG(release, debug|release) {
    win32-g++:contains(QT_ARCH, i386): LIBS += -L$$PWD/win/x86/ -lftd2xx
    else:win32-g++:contains(QT_ARCH, x86_64): LIBS += -L$$PWD/win/x64/ -lftd2xx
}
else:mingw:win32:CONFIG(debug, debug|release) {
    win32-g++:contains(QT_ARCH, i386): LIBS += -L$$PWD/win/x86/ -lftd2xx
    else:win32-g++:contains(QT_ARCH, x86_64): LIBS += -L$$PWD/win/x64/ -lftd2xx
}
else:msvc:win32:CONFIG(release, debug|release): LIBS += -L$$PWD/win/x64/ -lftd2xx
else:msvc:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/win/x64/ -lftd2xx
else:unix: LIBS += -L$$PWD/mac/x64/ -lftd2xx
