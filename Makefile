CXX       := g++
CXXFLAGS  := -std=c++17 -Wall -Wextra -O2
TARGET    := app

.PHONY: all run clean
all: $(TARGET)

$(TARGET): main.cpp chiakhoa.cpp
	@$(CXX) $(CXXFLAGS) main.cpp chiakhoa.cpp -o $@

run: all
	@./$(TARGET)

clean:
	@rm -f $(TARGET)
