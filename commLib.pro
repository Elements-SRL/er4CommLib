#-------------------------------------------------
#
# Project created by QtCreator 2018-07-11T14:37:06
#
#-------------------------------------------------

QT       -= core gui

include(./quietWarnings.pri)

CONFIG(debug, debug|release) {
    TARGET = er4commlibd
    DEFINES += DEBUGPRINT
}

CONFIG(release, debug|release) {
    TARGET = er4commlib
}

TEMPLATE = lib
CONFIG += c++11

# use as static library
DEFINES += ER4COMMLIB_STATIC
CONFIG += staticlib

# or create .dll
#DEFINES += ER4COMMLIB_LIBRARY

include(../version.pri)

DEFINES += "VERSION_MAJOR=$$VERSION_MAJOR"\
    "VERSION_MINOR=$$VERSION_MINOR"\
    "VERSION_BUILD=$$VERSION_BUILD"

VERSION_FULL = $${VERSION_MAJOR}.$${VERSION_MINOR}.$${VERSION_BUILD}

SOURCES += \
    er4commlib.cpp \
    ftdieeprom.cpp \
    ftdieeprom56.cpp \
    ftdieepromdemo.cpp \
    messagedispatcher.cpp \
    commandcoder.cpp \
    devices/e4/messagedispatcher_e4.cpp \
    devices/fake/messagedispatcher_fake_e16n.cpp \
    devices/e16/messagedispatcher_e16n.cpp

HEADERS += \
    er4commlib.h \
    er4commlib_errorcodes.h \
    er4commlib_global.h \
    ftdieeprom.h \
    ftdieeprom56.h \
    ftdieepromdemo.h \
    messagedispatcher.h \
    commandcoder.h \
    devices/e4/messagedispatcher_e4.h \
    devices/fake/messagedispatcher_fake_e16n.h \
    devices/e16/messagedispatcher_e16n.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += ./ \
    devices \
    devices/e4 \
    devices/e16 \
    devices/fake

DEPENDPATH += ./ \
    devices \
    devices/e4 \
    devices/e16 \
    devices/fake

macx: INCLUDEPATH += /usr/local/include
macx: DEPENDPATH += /usr/local/include

include(./ftd2xx/includeftd2xx.pri)

win32: QMAKE_LFLAGS+=-Wl,-Map=er4commlib_$${VERSION_FULL}.map
win32: QMAKE_CXXFLAGS_RELEASE+=-O0
