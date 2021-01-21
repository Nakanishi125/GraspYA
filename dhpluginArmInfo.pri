# ----------------------------------------------------
# dhpluginArmInfo.pri
# ------------------------------------------------------

#INCLUDEPATH += C:/lib64/glpk-4.55/src
#INCLUDEPATH += C:/lib64/PQP_v1.3/include

INCLUDEPATH += C:/kenkyu/GSL/include                                         #gslのインクルードパス

INCLUDEPATH += "C:/Program Files/PCL 1.8.1/include/pcl-1.8"                          #以下，PCLと3rdパーティーのインクルードパス
INCLUDEPATH += "C:/Program Files/PCL 1.8.1/3rdParty/Boost/include/boost-1_64"
INCLUDEPATH += "C:/Program Files/PCL 1.8.1/3rdParty/Eigen/eigen3"
INCLUDEPATH += "C:/Program Files/PCL 1.8.1/3rdParty/FLANN/include"
INCLUDEPATH += "C:/Program Files/OpenNI2/Include"
INCLUDEPATH += "C:/Program Files/PCL 1.8.1/3rdParty/Qhull/include"
INCLUDEPATH += "C:/Program Files/PCL 1.8.1/3rdParty/VTK/include/vtk-8.0"

INCLUDEPATH += "C:/GnuWin32/include"

#LIBS += C:/lib64/glpk-4.55/w64/glpk_4_55.lib
#LIBS += C:/lib64/PQP_v1.3/lib/PQPvs2015.lib

LIBS += "C:/kenkyu/GSL/lib/gsl/gsl.lib"                                      #gslのLibファイルリンク
LIBS += "C:/kenkyu/GSL/lib/gsl/cblas.lib"


LIBS += "C:/Program Files/PCL 1.8.1/lib/pcl_common_release.lib"                      #以下PCLの必要なLibファイルリンク
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_features_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_filters_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_io_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_io_ply_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_kdtree_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_keypoints_release.lib"
LIBS += "C:/Program Files/PCL 1.8.1/lib/pcl_ml_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_octree_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_outofcore_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_people_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_recognition_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_registration_release.lib"
LIBS += "C:/Program Files/PCL 1.8.1/lib/pcl_sample_consensus_release.lib"
LIBS += "C:/Program Files/PCL 1.8.1/lib/pcl_search_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_segmentation_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_stereo_release.lib"
LIBS += "C:/Program Files/PCL 1.8.1/lib/pcl_surface_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_tracking_release.lib"
#LIBS += "C:/Program Files/PCL 1.8.0/lib/pcl_visualization_release.lib"


LIBS += "C:/Users/ynunakanishi/Desktop/glpk-5.0/w64/glpk.lib"
#LIBS += "C:/GnuWin32/lib/glpk-bcc.lib"




#コンパイルしたDLLを指定directoryにコピーしたい場合
#DLLDESTDIR="C:\work\project\DhaibaWorks\DHToolsM_V2_2019\DHToolsM\Plugins"

HEADERS += ./dhpluginArmInfo.h
HEADERS += ./dhcontact.h
HEADERS += ./dhpluginArmInfo_global.h
HEADERS += ./stdafx.h
HEADERS += ./csv.hpp
HEADERS += ./pgmlib.h
HEADERS += ./Vector2D.hpp
HEADERS += ./rom_eval.hpp
HEADERS += ./coordinate_eval.hpp
HEADERS += ./collision_eval.hpp
HEADERS += ./finalpos.h
HEADERS += ./force_closure.hpp


SOURCES += ./dhpluginArmInfo.cpp
SOURCES += ./dhcontact.cpp
SOURCES += ./stdafx.cpp
SOURCES += ./csv.cpp
SOURCES += ./pgmlib.cpp
SOURCES += ./rom_eval.cpp
SOURCES += ./coordinate_eval.cpp
SOURCES += ./collision_eval.cpp
SOURCES += ./finalpos.cpp
SOURCES += ./force_closure.cpp





#DEFINES += _SCL_SECURE_NO_WARNINGS
