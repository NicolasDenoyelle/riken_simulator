#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

#include "numa.h"

int
test_mbind(const size_t size)
{
    void* buf;
    int out = 0;

    // Setup
    bitmask_decl(nodeset, NUMA_MAXNODE);
    bitmask_decl(allowed_nodeset, NUMA_MAXNODE);
    get_allowed_nodeset(allowed_nodeset, NUMA_MAXNODE, NULL);
    buf = mmap(NULL, size, PROT_READ | PROT_WRITE,
               MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
    if (buf == NULL) { perror("mmap"); return -1; }

    // Bind for each node
    for (long i=0; i<NUMA_MAXNODE; i++) {
        if (bitmask_isset(allowed_nodeset, i)){
            bitmask_zero(nodeset, NUMA_MAXNODE);
            bitmask_set(nodeset, i);

            if (mbind(buf, size, MPOL_BIND, nodeset,
                      NUMA_MAXNODE, MPOL_MF_MOVE | MPOL_MF_STRICT) == -1)
            {
                perror("mbind");
                out = -i-1;
                goto exit;
            }

            if (check_binding(buf, size, nodeset, NUMA_MAXNODE,
                              MPOL_BIND) != 0)
            {
                out = -i-1;
                goto exit;
            }
        }
    }

    // Cleanup
exit:
    if (munmap(buf, size) != 0)
        perror("munmap");
    return out;
}

//------------------------------------------------------//
// Main.
//------------------------------------------------------//

int
main()
{
    const size_t numa_page_size = sysconf(_SC_PAGESIZE);
    const size_t size = numa_page_size * 32;

    int err = test_mbind(size);

    if (err < 0) {
        fprintf(stderr, "Test failed on node %d\n", -err-1);
        return -1;
    } else {
        fprintf(stderr, "Success.\n");
        return 0;
    }
}
