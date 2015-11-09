IDIR =./
CC=gcc
CFLAGS=-I$(IDIR)

ODIR=./
LDIR =../lib

_DEPS = spp_interface.h rs232.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = main.o spp_interface.o rs232.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

spptest: $(OBJ)
	gcc -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~
