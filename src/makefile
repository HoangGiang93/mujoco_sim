#LINUX
COMMON=-O2 -I../include -L../bin -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS = -lmujoco200 -lGL -lm -lglew ../bin/libglfw.so.3
CC = g++

ROOT = mujoco_sim

all:
	$(CC) $(COMMON) main.cpp $(LIBS) -o ../bin/$(ROOT)

main.o:
	$(CC) $(COMMON) -c main.cpp

clean:
	rm *.o ../bin/$(ROOT)
