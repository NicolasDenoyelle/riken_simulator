LDFLAGS=
CFLAGS=-Wall -Wextra -Werror -rdynamic -fopenmp

SRC=numa.c
OBJ=$(patsubst %.c, %.o, $(SRC))
PROGS=allowed-nodeset stream stream-hbm

all: $(OBJ) $(PROGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(LDFLAGS)

stream-hbm: stream.c
	$(CC) $(CFLAGS) -O3 -DHBM $< -o $@ $(LDFLAGS)

stream: stream.c
	$(CC) $(CFLAGS) -O3 $< -o $@ $(LDFLAGS)

%: %.c $(OBJ)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -f $(OBJ) $(PROGS)
