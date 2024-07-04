QT       -= core gui

include(./quietWarnings.pri)

CONFIG(debug, debug|release) {
    TARGET = er4commlibd
    DEFINES += DEBUG_PRINT
}

CONFIG(release, debug|release) {
    TARGET = er4commlib
}

TEMPLATE = lib
CONFIG += c++14

#DEFINES += ER4COMMLIB_LABVIEW_WRAPPER
#DEFINES += ER4COMMLIB_PYTHON_WRAPPER

contains(DEFINES, ER4COMMLIB_LABVIEW_WRAPPER) {
    # create .dll
    TARGET = er4commlib_labview
    DEFINES += ER4COMMLIB_LIBRARY
    #DEFINES += OUTPUT_DATA_ONLY_FOR_ACTIVE_CHANNELS
    SOURCES += er4commlib_labview.cpp
    HEADERS += er4commlib_labview.h
    include($$(LABVIEW_TO_C_PATH)/includelabview.pri)
}

contains(DEFINES, ER4COMMLIB_PYTHON_WRAPPER) {
    # create .dll
    TARGET = er4CommLib_python
    DEFINES += ER4COMMLIB_LIBRARY
    #DEFINES += OUTPUT_DATA_ONLY_FOR_ACTIVE_CHANNELS
    CONFIG -= app_bundle

    SOURCES += er4commlib_python.cpp
    LIBS += -L"$$(LOCAL_PYTHON_3_10_7)\libs" -lpython310
    INCLUDEPATH += $$(LOCAL_PYBIND_11)\include \
            "$$(LOCAL_PYTHON_3_10_7)\include"
}

! contains(DEFINES, ER4COMMLIB_LIBRARY) {
    # build statically
    DEFINES += ER4COMMLIB_STATIC
    CONFIG += staticlib
}
include(./version.pri)

DEFINES += "VERSION_MAJOR=$$VERSION_MAJOR"\
    "VERSION_MINOR=$$VERSION_MINOR"\
    "VERSION_BUILD=$$VERSION_BUILD"

VERSION_FULL = $${VERSION_MAJOR}.$${VERSION_MINOR}.$${VERSION_BUILD}

SOURCES += \
    er4commlib.cpp \
    ftdieeprom.cpp \
    ftdieeprom56.cpp \
    ftdieepromdemo.cpp \
    calibrationeeprom.cpp \
    messagedispatcher.cpp \
    commandcoder.cpp \
    devices/e1/messagedispatcher_e1light.cpp \
    devices/e1/messagedispatcher_e1plus.cpp \
    devices/e1/messagedispatcher_e1hc.cpp \
    devices/e1/messagedispatcher_e1uln.cpp \
    devices/eNPR/messagedispatcher_enpr.cpp \
    devices/eNPR/messagedispatcher_enpr_hc.cpp \
    devices/e2/messagedispatcher_e2hc.cpp \
    devices/e4/messagedispatcher_e4n.cpp \
    devices/e4/messagedispatcher_e4e.cpp \
    devices/e16/messagedispatcher_e16e.cpp \
    devices/e16/messagedispatcher_e16n.cpp \
    devices/e16/messagedispatcher_e16fastpulses.cpp \
    devices/e16/messagedispatcher_e16hc.cpp \
    devices/e16/messagedispatcher_e16eth.cpp \
    devices/testboard/messagedispatcher_el06b.cpp \
    devices/testboard/messagedispatcher_el06c.cpp \
    devices/testboard/messagedispatcher_el06d_el06e.cpp \
    devices/testboard/messagedispatcher_el06f.cpp \
    devices/fake/messagedispatcher_fake_enpr.cpp \
    devices/fake/messagedispatcher_fake_enpr_hc.cpp \
    devices/fake/messagedispatcher_fake_e16n.cpp \
    devices/fake/messagedispatcher_fake_e16fastpulses.cpp

HEADERS += \
    er4commlib.h \
    er4commlib_errorcodes.h \
    er4commlib_global.h \
    er4commlib_global_addendum.h \
    ftdieeprom.h \
    ftdieeprom56.h \
    ftdieepromdemo.h \
    calibrationeeprom.h \
    messagedispatcher.h \
    commandcoder.h \
    devices/e1/messagedispatcher_e1light.h \
    devices/e1/messagedispatcher_e1plus.h \
    devices/e1/messagedispatcher_e1hc.h \
    devices/e1/messagedispatcher_e1uln.h \
    devices/eNPR/messagedispatcher_enpr.h \
    devices/eNPR/messagedispatcher_enpr_hc.h \
    devices/e2/messagedispatcher_e2hc.h \
    devices/e4/messagedispatcher_e4n.h \
    devices/e4/messagedispatcher_e4e.h \
    devices/e16/messagedispatcher_e16e.h \
    devices/e16/messagedispatcher_e16n.h \
    devices/e16/messagedispatcher_e16fastpulses.h \
    devices/e16/messagedispatcher_e16hc.h \
    devices/e16/messagedispatcher_e16eth.h \
    devices/testboard/messagedispatcher_el06b.h \
    devices/testboard/messagedispatcher_el06c.h \
    devices/testboard/messagedispatcher_el06d_el06e.h \
    devices/testboard/messagedispatcher_el06f.h \
    devices/fake/messagedispatcher_fake_enpr.h \
    devices/fake/messagedispatcher_fake_enpr_hc.h \
    devices/fake/messagedispatcher_fake_e16n.h \
    devices/fake/messagedispatcher_fake_e16fastpulses.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += ./ \
    devices \
    devices/e1 \
    devices/eNPR \
    devices/e2 \
    devices/e4 \
    devices/e16 \
    devices/fake \
    devices/testboard

DEPENDPATH += ./ \
    devices \
    devices/e1 \
    devices/eNPR \
    devices/e2 \
    devices/e4 \
    devices/e16 \
    devices/fake \
    devices/testboard

macx: INCLUDEPATH += /usr/local/include
macx: DEPENDPATH += /usr/local/include

include($$(FTD2XX_PATH)includeftd2xx.pri)

win32: QMAKE_LFLAGS+=-Wl,-Map=er4commlib_$${VERSION_FULL}.map
win32: QMAKE_CXXFLAGS_RELEASE+=-O0
