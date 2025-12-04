CXX       := g++
CXXFLAGS  := -std=c++17 -Wall -Wextra -O2
TARGET    := app

.PHONY: all run clean
all: $(TARGET)

$(TARGET): main.cpp Quadrotor.cpp controller.cpp
	@$(CXX) $(CXXFLAGS) main.cpp Quadrotor.cpp controller.cpp -o $@

run: all
	@./$(TARGET)

clean:
	@rm -f $(TARGET)
