
CXXFLAGS = -Wall -g -I ../include/ 

all: EjemploRaton2 EjemploRaton

EjemploRaton: objs/EjemploRaton.o
	g++ $(CXXFLAGS) -o EjemploRaton objs/EjemploRaton.o `pkg-config opencv4 --libs`

EjemploRaton2: objs/EjemploRaton2.o
	g++ $(CXXFLAGS) -o EjemploRaton2 objs/EjemploRaton2.o `pkg-config opencv4 --libs`

objs/EjemploRaton2.o: EjemploRaton2.cpp
	g++ $(CXXFLAGS) -o objs/EjemploRaton2.o -c EjemploRaton2.cpp `pkg-config opencv4 --cflags`

objs/EjemploRaton.o: EjemploRaton.cpp
	g++ $(CXXFLAGS) -o objs/EjemploRaton.o -c EjemploRaton.cpp `pkg-config opencv4 --cflags`

clean: 
	rm objs/*.o EjemploRaton EjemploRaton2
