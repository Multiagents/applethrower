CC = clang
CXX = clang++

# Compiler options, includes, library links
INCLUDE = -Isrc
FLAGS = -Wall -Wno-unused-result -O3 -ggdb -I. -lm
#FLAGS = -lrt -lpthread -openmp

# List all .c files to be compiled
SRC = $(shell find src/ -type f -name '*.cpp')

MAIN = main/main.cpp
EXEC = bin/prog

# Compile the main source code "MAIN" and output binary "EXEC"
default: $(MAIN) $(SRC)
	$(CXX) $(FLAGS) $(INCLUDE) $(MAIN) $(SRC) -o $(EXEC)

