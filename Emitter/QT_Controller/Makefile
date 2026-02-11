# Nom du binaire
TARGET = drone_controller

# Compilateur
CXX = g++

# Flags Qt via pkg-config
CXXFLAGS = -Wall -Wextra -std=c++17 $(shell pkg-config --cflags Qt6Widgets Qt6SerialPort Qt6Network)
LDFLAGS  = $(shell pkg-config --libs Qt6Widgets Qt6SerialPort Qt6Network)

# Sources
SRC = $(wildcard *.cpp)
OBJ = $(SRC:.cpp=.o)

# Règle par défaut
all: $(TARGET)

# Compilation finale
$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)

# Compilation des .cpp en .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Nettoyage
clean:
	rm -f $(OBJ) $(TARGET)

run: clean all
	./$(TARGET)
