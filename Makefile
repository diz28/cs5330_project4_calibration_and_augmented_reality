
# Di Zhang
# Feb 10, 2023
# CS5330 - Computer Vision

CC = g++
PROJECT = main
SRC = main.cpp tasks.cpp
LIBS = `pkg-config --cflags --libs opencv4 yaml-cpp`
$(PROJECT) : $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(LIBS)
