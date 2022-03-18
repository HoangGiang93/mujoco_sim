COMMON=-O2 -I../include -L../lib -std=c++11 -mavx -pthread -Wl,-rpath,'$$ORIGIN'

all:
	g++ $(COMMON) testxml.cc    -lmujoco_nogl                             			-o ../bin/testxml
	g++ $(COMMON) testspeed.cc  -lmujoco_nogl                             			-o ../bin/testspeed
	g++ $(COMMON) compile.cc    -lmujoco_nogl                             			-o ../bin/compile
	g++ $(COMMON) derivative.cc -lmujoco_nogl -fopenmp                    			-o ../bin/derivative
	g++ $(COMMON) basic.cc      -lmujoco -lGL -lGLEW ../lib/libglfw.so.3			-o ../bin/basic
	g++ $(COMMON) record.cc     -lmujoco -lGL -lGLEW ../lib/libglfw.so.3 			-o ../bin/record
	gcc -c -O2 -mavx -I../include ../sample/uitools.c
	g++ $(COMMON) uitools.o simulate.cc -lmujoco -lGL -lGLEW ../lib/libglfw.so.3 	-o ../bin/simulate
	rm *.o