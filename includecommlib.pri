DEFINES += ER4COMMLIB_STATIC

INCLUDEPATH += $$(ER4COMMLIB_PATH)include
DEPENDPATH += $$(ER4COMMLIB_PATH)include

win32:CONFIG(release, debug|release): LIBS += -L$$(ER4COMMLIB_PATH)lib/release/ -ler4commlib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$(ER4COMMLIB_PATH)lib/debug/ -ler4commlibd
else:unix:CONFIG(release, debug|release): LIBS += -L$$(ER4COMMLIB_PATH)lib/ -ler4commlib
else:unix:CONFIG(debug, debug|release): LIBS += -L$$(ER4COMMLIB_PATH)lib/ -ler4commlibd

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/release/liber4commlib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/debug/liber4commlibd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/release/er4commlib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/debug/er4commlibd.lib
else:unix:CONFIG(release, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/liber4commlib.a
else:unix:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$(ER4COMMLIB_PATH)lib/liber4commlibd.a

# has to be called after
include($$(FTD2XX_PATH)includeftd2xx.pri)
