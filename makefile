CXX ?= g++

DEBUG := 0

ifeq ($(DEBUG),1)
  CXXFLAGS += -O0 -g
else
  CXXFLAGS += -O3
endif

CXXFLAGS += -c -Wall

SensorFusionObject := SensorFusion.o main.o

all: SensorFusion

# $< dependency object sets, $@ object sets
SensorFusion: $(SensorFusionObject)
	$(CXX) $(SensorFusionObject) -o $@ $(LDFLAGS)

%.o: %.cpp; $(CXX) $< -o $@ $(CXXFLAGS)

.PHONY : clean
clean: ; rm -f $(SensorFusionObject)
