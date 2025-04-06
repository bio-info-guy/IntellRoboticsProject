CXX=g++
IDIR=include/
IDIRS=-Iinclude/
LDIR=-Llib/
CFLAGS=-std=c++14 -Wall -O2 $(IDIRS) $(LDIR) -fopenmp -fPIC
ODIR=obj/
SRCDIR=src/
MAINDIR=main/

LIBS=
EXEC_PATH=bin/
LIB_PATH=lib/

_DEPS=RigidBodyMotion.hpp TypeDefs.hpp ForwardKinematics.hpp Spatial6R.hpp MathUtils.hpp Jacobian.hpp InverseKinematics.hpp TrajectoryPlanning.hpp
DEPS=$(patsubst %,$(IDIR)%,$(_DEPS))

_OBJ_ALL=Planar2R.o Planar2Rmod.o Planar3R.o Spatial3R.o Spatial6R.o  MathUtils.o RigidBodyMotion.o ForwardKinematics.o Jacobian.o InverseKinematics.o TrajectoryPlanning.o
OBJ_ALL=$(patsubst %,$(ODIR)%,$(_OBJ_ALL))


all: $(LIB_PATH)libIRLibrary.a
	cd main && $(MAKE)

exec: $(LIB_PATH)libIRLibrary.a  
	cd main && $(MAKE)

$(ODIR):
	mkdir -p $(ODIR)

$(LIB_PATH):
	mkdir -p $(LIB_PATH)

$(EXEC_PATH):
	mkdir -p $(EXEC_PATH)

$(ODIR)%.o: $(SRCDIR)%.cpp $(IDIR)%.hpp $(DEPS) | $(ODIR) $(LIB_PATH)
	$(CXX) -c -o $@ $< $(CFLAGS)

gradeAll: $(MAINDIR)gradeAll.cpp $(DEPS) | $(EXEC_PATH)
	cd main && $(MAKE) gradeAll

$(LIB_PATH)libIRLibrary.a: $(OBJ_ALL)
	ar rcs $@ $^

.PHONY: clean

clean:
	rm -f $(ODIR)*.o *~ $(EXEC_PATH)* $(LIB_PATH)*
