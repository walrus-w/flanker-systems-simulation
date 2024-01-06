# Define the C++ compiler to use
CC = g++

# Define any directories containing header files
# Add the path to Eigen and to your source header files
INCLUDES = -I./walrus-new-nav-system -IC:/libraries/eigen-3.4.0
	-IC:/libraries/eigen-3.4.0

# Define the C++ source files
SRCS = $(wildcard ./walrus-new-nav-system/*.cpp)

# Define the C++ object files
OBJS = $(SRCS:.cpp=.o)

# Defines the output executable
MAIN = nav_system

# The main target
all: $(MAIN)

# Link the object files into a binary
$(MAIN): $(OBJS)
	$(CC) $(INCLUDES) -o $(MAIN) $(OBJS)

# Compile the source files into object files
.cpp.o:
	$(CC) $(INCLUDES) -c $<  -o $@

# Clean target
clean:
	$(RM) $(MAIN) $(OBJS)