main: main.o Map.o MapReader.o Path.o
	g++ -o main main.o Map.o MapReader.o Path.o

test: tests.o Map.o MapReader.o Path.o
	g++ -o test tests.o Map.o MapReader.o Path.o
	
tests.o: tests/tests.cpp
	g++ -c -std=c++2a tests/tests.cpp

Map.o: src/Map.cpp
	g++ -c -std=c++2a src/Map.cpp

MapReader.o: src/MapReader.cpp
	g++ -c -std=c++2a src/MapReader.cpp

Path.o: src/Path.cpp
	g++ -c -std=c++2a src/Path.cpp

main.o: src/main.cpp
	g++ -c -std=c++2a src/main.cpp

clean:
	rm *.o main