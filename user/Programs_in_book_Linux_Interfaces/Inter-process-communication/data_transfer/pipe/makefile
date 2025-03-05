# List all C source files in the current directory
SRC := $(wildcard *.c)

# Define the corresponding executable files
EXE := $(SRC:.c=)

# Compiler
CC := gcc

# Compiler flags
CFLAGS := -Wall -Wextra -Werror

# Default target: build all executables
all: $(EXE)

# Rule to build each executable
%: %.c
	$(CC) $(CFLAGS) -o $@ $<

# Clean target to remove all executables
clean:
	rm -f $(EXE)

# Phony targets
.PHONY: all clean
