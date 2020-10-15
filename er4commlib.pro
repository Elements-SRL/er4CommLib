#-------------------------------------------------
#
# Project created by QtCreator 2018-07-11T14:37:06
#
#-------------------------------------------------

QT       -= core gui

include (./quietWarnings.pri)

CONFIG(debug, debug|release) {
    TARGET = er4commlibd
    DEFINES += DEBUGPRINT
}

CONFIG(release, debug|release) {
    TARGET = er4commlib
}

TEMPLATE = lib

DEFINES += ER4COMMLIB_LIBRARY

SOURCES += \
    devices/fake/messagedispatcher_fake_e16n.cpp \
    devices/nanopore/messagedispatcher_e16n.cpp \
    er4commlib.cpp \
    ftdieeprom.cpp \
    ftdieeprom56.cpp \
    ftdieepromdemo.cpp \
    messagedispatcher.cpp \
    commandcoder.cpp

HEADERS += \
    devices/fake/messagedispatcher_fake_e16n.h \
    devices/nanopore/messagedispatcher_e16n.h \
    er4commlib.h \
    er4commlib_errorcodes.h \
    er4commlib_global.h \
    ftdieeprom.h \
    ftdieeprom56.h \
    ftdieepromdemo.h \
    messagedispatcher.h \
    commandcoder.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += ./ \
    devices \
    devices/nanopore \
    devices/fake

DEPENDPATH += ./ \
    devices \
    devices/nanopore \
    devices/fake

macx: LIBS += -L/usr/local/lib/ -lftd2xx

macx: INCLUDEPATH += /usr/local/include
macx: DEPENDPATH += /usr/local/include

include (./ftd2xx/includeftd2xx.pri)

