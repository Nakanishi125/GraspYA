#
# dhPlugins.pri
#

TEMPLATE = lib

#DESTDIR = C:/DhaibaWorksSDK2/bin/Packages/SamplePckg1/Plugins/
#DLLDESTDIR= C:/DhaibaWorksSDK2/bin/Packages/SamplePckg1/Plugins/
DESTDIR = C:/Users/nakanishi/kenkyu/GraspYA/Plugins
DLLDESTDIR= C:/Users/nakanishi/kenkyu/GraspYA/Plugins

#OBJECTS_DIR = c:/temp
#MOC_DIR = c:/temp

CONFIG += release
CONFIG += plugin
CONFIG += c++11
CONFIG += precompile_header

QT += core
QT += gui
QT += script
QT += opengl
QT += xml
QT += network
QT += widgets

#自分の環境にあわせて以下を編集
DW_SDK_DIR = C:/DhaibaWorksSDK2
INCLUDEPATH += .
INCLUDEPATH += ../Commons
INCLUDEPATH += $${DW_SDK_DIR}/SDK/src
INCLUDEPATH += $${DW_SDK_DIR}/SDK/src/dh
INCLUDEPATH += $${DW_SDK_DIR}/SDK/GeneratedFiles
INCLUDEPATH += $${DW_SDK_DIR}/etc/OpenGL/include

LIBS += $${DW_SDK_DIR}/SDK/lib/DhaibaWorksSDK2.lib
LIBS += $${DW_SDK_DIR}/etc/OpenGL/lib/glut64.lib

PRECOMPILED_HEADER = stdafx.h

#RESOURCES = ../dhPlugins.qrc
