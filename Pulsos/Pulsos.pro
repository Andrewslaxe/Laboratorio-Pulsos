QT       += core gui
QT       += serialport
QT       += widgets printsupport
greaterThan(QT_MAJOR_VERSION, 4):

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    qcustomplot.cpp \
    widget.cpp
    qcustomplot.cpp
HEADERS += \
    qcustomplot.h \
    widget.h
    qcustomplot.h
FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

