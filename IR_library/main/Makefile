CC=g++
IDIR=../include/
IDIRS=-I../include/

LDIR=-L../lib/
GTESTDIR=-L../gtest/
CFLAGS=-std=c++14 -Wall -O2 $(IDIRS) $(LDIR) -fopenmp -I/usr/include/eigen3/
ODIR=../obj/
SRCDIR=

LIBS=-lIRLibrary
EXEC_PATH=../bin/

DEPS=../lib/libIRLibrary.a

all: $(EXEC_PATH)simple2R $(EXEC_PATH)mod2R $(EXEC_PATH)Spatial3Rtest $(EXEC_PATH)checkRigidBodyMotion $(EXEC_PATH)planar3r $(EXEC_PATH)spatial6r


$(EXEC_PATH)simple2R: $(SRCDIR)simple2R.cpp $(DEPS) | $(EXEC_PATH)
	$(CC) $(CFLAGS) $< $(LIBS) -o $@ 

$(EXEC_PATH)mod2R: $(SRCDIR)mod2R.cpp $(DEPS)
	$(CC) $(CFLAGS) $< $(LIBS) -o $@ 

$(EXEC_PATH)checkRigidBodyMotion: $(SRCDIR)checkRigidBodyMotion.cpp $(DEPS) | $(EXEC_PATH)
	$(CC) $(CFLAGS) $< $(LIBS) -o $@ 

$(EXEC_PATH)planar3r: $(SRCDIR)planar3r.cpp $(DEPS) | $(EXEC_PATH)
	$(CC) $(CFLAGS) $< $(LIBS) -o $@

$(EXEC_PATH)spatial6r: $(SRCDIR)spatial6r.cpp $(DEPS) | $(EXEC_PATH)
	$(CC) $(CFLAGS) $< $(LIBS) -o $@

gradeAll: $(SRCDIR)gradeAll.cpp $(DEPS) | $(EXEC_PATH)
	$(CC) $(CFLAGS) $(GTESTDIR) $< $(LIBS) -lgtest -lpthread -o $(EXEC_PATH)$@ 

$(EXEC_PATH):
	mkdir -p $(EXEC_PATH)


.PHONY: clean

clean:
	rm  $(EXEC_PATH)* 
