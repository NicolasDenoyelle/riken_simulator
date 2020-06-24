#include <stdio.h>

#include "numa.h"

#include "numa.c"

#define MAXNODE (NUMA_MAXNODE / (8 * sizeof(unsigned long)))

int main() {
    static unsigned long nodeset[MAXNODE];

    for (unsigned i = 0; i<MAXNODE; i++) nodeset[i] = 0;

    unsigned long numnode = 0;
    int err;

    err = get_allowed_nodeset(nodeset, NUMA_MAXNODE, &numnode);
    if (err != 0) {
        perror("get_mempolicy: ");
    }

    for (int i = 0; i < NUMA_MAXNODE; i++)
        if (bitmask_isset(nodeset, i)) printf("%d ", i);
    printf("\n");
}
