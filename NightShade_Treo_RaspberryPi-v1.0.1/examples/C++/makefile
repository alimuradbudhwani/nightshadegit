CC = g++
RM = rm -f
CFLAGS = -Wall -g -lNightShade_Treo -lwiringPi

SRCS = $(wildcard *.cpp)
TARGETS = $(SRCS:.cpp=)

all: $(TARGETS)

%: %.cpp
	$(CC) $(CFLAGS) $< -o $@

.PHONY: clean

clean:
	$(RM) $(TARGETS)
