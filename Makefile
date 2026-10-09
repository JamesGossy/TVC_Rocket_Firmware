CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Icommon -Iflight_software -Isim

SOURCES = $(wildcard common/*.cpp) $(wildcard flight_software/*.cpp) $(wildcard sim/*.cpp)
HEADERS = $(wildcard common/*.h) $(wildcard flight_software/*.h) $(wildcard sim/*.h)

tvc_sim: $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCES) -static -o tvc_sim

clean:
	rm -f tvc_sim tvc_sim.exe flight.csv montecarlo.csv
