#include <stdio.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "numa.h"

#ifndef __NR_mbind
#ifdef __i386__
#define __NR_mbind 274
#elif defined(__x86_64__)
#define __NR_mbind 237
#elif defined(__ia64__)
#define __NR_mbind 1259
#elif defined(__powerpc__) || defined(__ppc__) || defined(__PPC__) || \
    defined(__powerpc64__) || defined(__ppc64__)
#define __NR_mbind 259
#elif defined(__sparc__)
#define __NR_mbind 353
#elif defined(__aarch64__)
#define __NR_mbind 235
#elif defined(__arm__)
#define __NR_mbind 319
#endif
#endif

#ifndef __NR_get_mempolicy
#ifdef __i386__
#define __NR_get_mempolicy 275
#elif defined(__x86_64__)
#define __NR_get_mempolicy 238
#elif defined(__ia64__)
#define __NR_get_mempolicy 1260
#elif defined(__powerpc__) || defined(__ppc__) || defined(__PPC__) || \
    defined(__powerpc64__) || defined(__ppc64__)
#define __NR_get_mempolicy 260
#elif defined(__sparc__)
#define __NR_get_mempolicy 304
#elif defined(__aarch64__)
#define __NR_get_mempolicy 236
#elif defined(__arm__)
#define __NR_get_mempolicy 320
#endif
#endif

// mbind syscall wrapper as in libnuma
long mbind(const void* addr, const size_t len, const int mode,
           const unsigned long* nodemask, const unsigned long maxnode,
           const int flags) {
    if (syscall(__NR_mbind, addr, len, mode, nodemask, maxnode, flags) != 0) {
        perror("mbind");
        return -1;
    }
    return 0;
}

// get_mempolicy syscall wrapper as in libnuma
long get_mempolicy(int* mode, unsigned long* nodemask, unsigned long maxnode,
                   void* addr, int flags) {
    if (syscall(__NR_get_mempolicy, mode, nodemask, maxnode, addr, flags) !=
        0) {
        perror("get_mempolicy");
        return -1;
    }
    return 0;
}

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

int get_allowed_nodeset(unsigned long* nodeset, unsigned long maxnode,
                        unsigned long* numnode) {
    unsigned long n = sizeof(unsigned long) * 8 * bitmask_nulongs(maxnode);
    int err = get_mempolicy(NULL, nodeset, maxnode, NULL, MPOL_F_MEMS_ALLOWED);
    if (err != 0) return err;
    while (n-- > 0 && !bitmask_isset(nodeset, n))
        ;
    *numnode = n + 1;
    return 0;
}

const char* policy_str(const int mode) {
    switch (mode) {
        case MPOL_DEFAULT:
            return "MPOL_DEFAULT";
        case MPOL_BIND:
            return "MPOL_BIND";
        case MPOL_INTERLEAVE:
            return "MPOL_INTERLEAVE";
        case MPOL_PREFERRED:
            return "MPOL_PREFERRED";
#ifdef MPOL_LOCAL
        case MPOL_LOCAL:
            return "MPOL_LOCAL";
#endif
        default:
            return NULL;
    }
}
