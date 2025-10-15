# This is used to compile the DLL used by liblablib and python wrapper (hopefully not for long 27/02/2025)
INCLUDEPATH += $$(ER4COMMLIB_PATH)include
DEPENDPATH += $$(ER4COMMLIB_PATH)include

win32:CONFIG(release, debug|release): LIBS += -L$$(ER4COMMLIB_PATH)lib/release/ -ler4commlibDLL
else:win32:CONFIG(debug, debug|release): LIBS += -L$$(ER4COMMLIB_PATH)lib/debug/ -ler4commlibDLLd
else:unix:CONFIG(release, debug|release): LIBS += -L$$(ER4COMMLIB_PATH)lib/ -ler4commlibDLL
else:unix:CONFIG(debug, debug|release): LIBS += -L$$(ER4COMMLIB_PATH)lib/ -ler4commlibDLLd

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/release/liber4commlibDLL.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/debug/liber4commlibDLLd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/release/er4commlibDLL.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/debug/er4commlibDLLd.lib
else:unix:CONFIG(release, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/liber4commlibDLL.a
else:unix:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/liber4commlibDLLd.a

# has to be called after
include($$(FTDI_UTILS_PATH)includeftdiutils.pri)
