CC=gcc
CFLAGS=-Wall -Wextra -I. -static -std=c99
LDFLAGS=
OPENMP=-fopenmp

.PHONY:= all clean

all: allowed-nodeset mpol-bind stream2

numa.o: numa.c
	$(CC) $(CFLAGS) $^ -c -o $@ $(LDFLAGS)

allowed-nodeset: allowed-nodeset.c numa.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

mpol-bind: mpol-bind.c numa.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

stream2: stream2.c numa.o
	$(CC) $(CFLAGS) -O3  $^ -o stream2.gnu.x $(OPENMP)
	$(CC) $(CFLAGS) -O3 -DHBM $^ -o stream2.gnu.HBM.x $(OPENMP)

clean:
	rm -f stream2.gnu.*x mpol-bind allowed-nodeset numa.o
