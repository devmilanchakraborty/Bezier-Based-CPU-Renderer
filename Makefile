CC      := gcc
TARGET  := renderer

# 1. Standardize Flags
# Added -Wall and -Wextra because high-level C devs expect to see them.
# Added -I./files/include to ensure headers are found.
CFLAGS  := -O3 -march=native -ffast-math -funroll-loops -flto -pthread \
           -Wall -Wextra -I./files/include -I./files/src

# 2. Linker Flags
LDFLAGS := -pthread -flto -lm

# 3. Path Management
VPATH   := files/src

# 4. Source & Object Setup
SRC     := main.c scene.c renderer.c geometry.c math_utils.c
OBJ     := $(SRC:.c=.o)

# Default rule
all: $(TARGET)

# Linking step
$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Compilation step
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 5. The "Professional" Cleanup
.PHONY: all clean

clean:
	rm -f *.o $(TARGET)
