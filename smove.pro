#-------------------------------------------------
#
# Project created by QtCreator 2016-09-11T11:42:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = smove
TEMPLATE = app


SOURCES += Common/main.cpp \
    Views/MainWindowView.cpp \
    Views/CommandListView.cpp \
    Views/ProgramView.cpp \

HEADERS  += MainWindowView.h \
    Views/MainWindowView.h \
    Views/CommandListView.h \
    Views/ProgramView.h \
    Services/SerialService.h \
    Common/config.h

FORMS    += Views/MainWindowView.ui \
    Views/CommandListView.ui \
    Views/ProgramView.ui

RESOURCES += \
    Theme/style.qrc

CONFIG += c++11
