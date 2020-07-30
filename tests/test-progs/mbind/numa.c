#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
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
#elif defined(__powerpc__) || defined(__ppc__) || defined(__PPC__) ||   \
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
#elif defined(__powerpc__) || defined(__ppc__) || defined(__PPC__) ||   \
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
        return -1;
    }
    return 0;
}

// get_mempolicy syscall wrapper as in libnuma
long get_mempolicy(int* mode, unsigned long* nodemask, unsigned long maxnode,
                   void* addr, int flags) {
    if (syscall(__NR_get_mempolicy, mode, nodemask, maxnode, addr, flags) !=
        0) {
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
    while (n-- > 0 && !bitmask_isset(nodeset, n));
    if (numnode != NULL)
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

static size_t
numa_nodeset_snprintf(char *str, const size_t len,
                      const unsigned long *nodeset,
                      const unsigned long maxnode)
{
    unsigned i, n = 0;
    char* buf = str;

    memset(str, 0, len);
    for (i = 0; i < maxnode; i++) {
        if (bitmask_isset(nodeset, i)) {
            if (n == 0)
                buf += snprintf(buf, len + str - buf, "[%u", i);
            else {
                buf += snprintf(buf, len + str - buf, ",%u", i);
                n++;
            }
        }
    }
    buf += snprintf(buf, len + str - buf, "]");
    return buf - str;
}

int
check_binding(const void* addr, const size_t size,
              const unsigned long* nodemask,
              const unsigned long maxnode,
              const int mode)
{
    const size_t numa_page_size = sysconf(_SC_PAGESIZE);
    const size_t npages = (size / numa_page_size) +
        (size % numa_page_size ? 1 : 0);
    const size_t start = (size_t)addr - ((size_t)addr % numa_page_size);
    const size_t end = start + (npages * numa_page_size);

    size_t page;
    int err;
    int _mode;
    size_t nset = 0;
    bitmask_decl(_nodemask, maxnode);
    bitmask_decl(allowed_nodeset, NUMA_MAXNODE);

    // Retrieve allowed nodeset.
    if (get_allowed_nodeset(allowed_nodeset, NUMA_MAXNODE, &nset) != 0)
        goto allowed_nodeset_failure;

    // If one node set, all MPOL policies are equivalent.
    bitmap_nset(nodemask, maxnode, nset);

    for (page = start; page < end; page += numa_page_size) {
        err = get_mempolicy(&_mode, _nodemask, maxnode, (void*)page,
                            MPOL_F_NODE | MPOL_F_ADDR);
        if (err != 0)
            goto syscall_failure;

        // mode equal ?
        if (_mode != mode && nset > 1)
            goto mode_match_failure;

        // is included ?
        for (size_t i = 0; i < maxnode; i++) {
            if (bitmask_isset(_nodemask, i) && !bitmask_isset(nodemask, i))
                goto nodeset_match_failure;
        }
    }

    return 0;

allowed_nodeset_failure:
    perror("get_allowed_nodeset");
    return -1;

syscall_failure:
    fprintf(stderr, "get_mempolicy error: %s\n", strerror(err));
    return -1;

mode_match_failure:
    fprintf(stderr, "Page @%p mode %s did not match mode %s\n",
            (void*)page, policy_str(_mode), policy_str(mode));
    return -1;

nodeset_match_failure:;
    char _nodemask_str[256];
    char nodemask_str[256];
    numa_nodeset_snprintf(_nodemask_str, sizeof(_nodemask_str),
                          _nodemask, maxnode);
    numa_nodeset_snprintf(nodemask_str, sizeof(nodemask_str),
                          nodemask, maxnode);

    fprintf(stderr, "Page @%p nodemask %s did not match nodemask %s\n",
            (void*) page, _nodemask_str, nodemask_str);
    return -1;
}
