CC = g++

CLASSDIR = /home/cristinel/noc/vnoc3
INCDIRS = $(CLASSDIR)/include

LIB_DIR = -L/usr/lib/X11
LIB = -lX11 -lm
X11_INCLUDE = -I/usr/X11R6/include
BDDDIR = /home/cristinel/noc/vnoc3/lib

WARN_FLAGS = -Wall -Wpointer-arith -Wcast-qual -Wstrict-prototypes -O -D__USE_FIXED_PROTOTYPES__ -ansi -pedantic -Wmissing-prototypes -Wshadow -Wcast-align -D_POSIX_SOURCE
DEBUG_FLAGS = -g
OPT_FLAGS = -O3

#FLAGS = $(WARN_FLAGS)
#FLAGS = $(DEBUG_FLAGS) 
FLAGS = $(OPT_FLAGS)
FLAGS += $(addprefix -I, $(INCDIRS))


EXE = sfra

OBJ = hmetisInterface.o fp_plan.o fp_btree.o fp_sa.o sfra.o sfra_gui.o sfra_hungarian.o vnoc_app.o vnoc_topology.o vnoc_utils.o vnoc_event.o vnoc.o sfra_main.o 

SRC = hmetisInterface.cpp fp_plan.cpp fp_btree.cpp fp_sa.cpp sfra.cpp sfra_gui.cpp sfra_hungarian.cpp vnoc_app.cpp vnoc_topology.cpp vnoc_utils.cpp vnoc_event.cpp vnoc.cpp sfra_main.cpp

H = include/hmetis.h include/hmetisInterface.h include/config.h include/fp_plan.h include/fp_btree.h include/fp_sa.h include/sfra.h include/sfra_gui.h include/sfra_hungarian.h include/vnoc_app.h include/vnoc_topology.h include/vnoc_utils.h include/vnoc_event.h include/vnoc.h

$(EXE): $(OBJ)
	$(CC) $(FLAGS) $(OBJ) -o $(EXE) $(LIB_DIR) $(LIB) $(BDDDIR)/libhmetis.a

hmetisInterface.o: hmetisInterface.cpp $(H)
	$(CC) -c $(FLAGS) hmetisInterface.cpp

fp_plan.o: fp_plan.cpp $(H)
	$(CC) -c $(FLAGS) fp_plan.cpp

fp_btree.o: fp_btree.cpp $(H)
	$(CC) -c $(FLAGS) fp_btree.cpp

fp_sa.o: fp_sa.cpp $(H)
	$(CC) -c $(FLAGS) fp_sa.cpp

sfra.o: sfra.cpp $(H)
	$(CC) -c $(FLAGS) sfra.cpp

sfra_gui.o: sfra_gui.cpp $(H)
	$(CC) -c $(FLAGS) $(X11_INCLUDE) sfra_gui.cpp

sfra_hungarian.o: sfra_hungarian.cpp $(H)
	$(CC) -c $(FLAGS) $(X11_INCLUDE) sfra_hungarian.cpp

vnoc_app.o: vnoc_app.cpp $(H)
	$(CC) -c $(FLAGS) vnoc_app.cpp

vnoc_topology.o: vnoc_topology.cpp $(H)
	$(CC) -c $(FLAGS) vnoc_topology.cpp

vnoc_utils.o: vnoc_utils.cpp $(H)
	$(CC) -c $(FLAGS) vnoc_utils.cpp

vnoc_event.o: vnoc_event.cpp $(H)
	$(CC) -c $(FLAGS) vnoc_event.cpp

vnoc.o: vnoc.cpp $(H)
	$(CC) -c $(FLAGS) vnoc.cpp

sfra_main.o: sfra_main.cpp $(H)
	$(CC) -c $(FLAGS) sfra_main.cpp


