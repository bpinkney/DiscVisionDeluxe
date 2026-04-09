CXX = g++

SOURCES = DfisX.cpp Daero.cpp Dgyro.cpp Dpropagate.cpp
LIB =
CXXFLAGS = -Wall -g -O4 -std=c++11
#LDFLAGS = -lpthread
#VPATH=src

#try loading in disc definitions for test purposes
CXXFLAGS += -I../common/inc
ifeq ($(OS),Windows_NT)     # is Windows_NT on XP, 2000, 7, Vista, 10...
    detected_OS := Windows
    # you also need http://www.gnu.org/software/make/ for windows...
    CXXFLAGS += -IC:/MinGW/include/eigen3/
else
    detected_OS := $(shell uname)  # same as "uname -s"
    #assume linux and default eigen3 install location
    CXXFLAGS += -I/usr/include/eigen3/
    CXXFLAGS += -I/usr/local/include/eigen3/
endif

TARGETS := main

.PHONY: all
all: main

main: main.o ${LIB}
		@echo "   [$@]"
		@$(CXX) -o dfisx $^ $(CXXFLAGS) $(SOURCES) $(LDFLAGS) 

.PHONY: clean
clean:
		@rm -rf *.o $(TARGETS)