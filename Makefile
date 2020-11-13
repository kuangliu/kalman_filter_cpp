CC = g++
CFLAGS = -Wall -std=c++11
SRCS = point_tracker.cc
PROG = point_tracker

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
