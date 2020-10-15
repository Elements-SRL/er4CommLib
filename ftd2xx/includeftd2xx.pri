INCLUDEPATH += $$PWD/
DEPENDPATH += $$PWD/

mingw:win32:CONFIG(release, debug|release): LIBS += -L$$PWD/win/x86/ -lftd2xx
else:mingw:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/win/x86/ -lftd2xxd
else:msvc:win32:CONFIG(release, debug|release): LIBS += -L$$PWD/win/x64/ -lftd2xx
else:msvc:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/win/x64/ -lftd2xxd
else:unix: LIBS += -L$$PWD/mac/x64/ -lftd2xx
