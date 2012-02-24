#-------------------------------------------------
#
# Project created by QtCreator 2011-07-01T12:58:44
#
#-------------------------------------------------


TARGET = server_30015
TEMPLATE = app


SOURCES += LinServer.cpp\
    pb/messages_robocup_ssl_wrapper.pb.cc \
    pb/messages_robocup_ssl_detection.pb.cc \
    pb/messages_robocup_ssl_geometry.pb.cc \
	pb/netraw.cpp \
	pb/grSim_Replacement.pb.cc \
	pb/grSim_Commands.pb.cc \
	pb/grSim_Packet.pb.cc \
	Robot.cpp \
	Goalie.cpp

INCLUDEPATH += include
LIBS += -Llibs/ -lprotobuf-2.4.0\
    -lpthread
