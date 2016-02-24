OBJ		= main.o
PROGRAM	= out
.PHONY: out
$(PROGRAM) : $(OBJ)
	gcc -o $(PROGRAM) $(OBJ) -lwiringPi
