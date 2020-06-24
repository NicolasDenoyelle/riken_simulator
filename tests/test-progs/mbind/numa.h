#ifndef __NUMA_H__
#define __NUMA_H__

#include <stdlib.h>

/******************************************************************************
 * Bitmask
 *****************************************************************************/

// Number of unsigned long required to fit maxnode bits.
#define bitmask_nulongs(maxnode)                   \
    (((maxnode) / (8 * (sizeof(unsigned long)))) + \
     ((maxnode) % (8 * (sizeof(unsigned long))) ? 1 : 0))

// Unsigned long in ulong array that contain bit i.
#define bitmask_mask(nodeset, i) (nodeset[(i) / (8 * sizeof(*(nodeset)))])

// Bit i in mask obtained with bitmask_mask(nodeset, i)
#define bitmask_maskbit(nodeset, i) ((i) % (8 * sizeof(*(nodeset))))

// Whether bit i is set in nodeset.
#define bitmask_isset(nodeset, i) \
    (bitmask_mask(nodeset, i) & (1UL << bitmask_maskbit(nodeset, i)))

// Set bit i in set.
#define bitmask_set(nodeset, i) \
    (bitmask_mask(nodeset, i) |= (1UL << bitmask_maskbit(nodeset, i)))

// Clear bit i in set
#define bitmask_clear(nodeset, i) \
    (bitmask_mask(nodeset, i) &= ~(1UL << bitmask_maskbit(nodeset, i)))

// Empty set.
#define bitmask_zero(nodeset, maxnode)                          \
    do {                                                        \
        for (size_t i = 0; i < bitmask_nulongs(maxnode); i++) { \
            nodeset[i] = 0;                                     \
        }                                                       \
    } while (0)

// Empty set.
#define bitmask_fill(nodeset, maxnode)                                        \
    do {                                                                      \
        size_t i;                                                             \
        for (i = 0; i < bitmask_nulongs(maxnode) - 1; i++) nodeset[i] = ~0UL; \
        nodeset[i] = ~0UL >> ((sizeof(*(nodeset)) * 8) -                      \
                              (maxnode % (sizeof(*(nodeset)) * 8)));          \
    } while (0)

// Store in n the number of bits set.
#define bitmap_nset(nodeset, maxnode, n)                                    \
    do {                                                                    \
        unsigned long mask;                                                 \
        n = 0;                                                              \
        for (size_t i = 0; i < maxnode; i++) {                              \
            n += mask & 1UL ? 1 : 0;                                        \
            mask = i % (sizeof(*nodeset) * 8) ? mask >> 1                   \
                                              : i / (sizeof(*nodeset) * 8); \
        }                                                                   \
    } while (0)

// Declare an empty nodeset fitting maxnode.
#define bitmask_decl(name, maxnode)               \
    unsigned long name[bitmask_nulongs(maxnode)]; \
    bitmask_zero(name, maxnode)

/******************************************************************************
 * NUMA
 *****************************************************************************/

#define MPOL_DEFAULT 0
#define MPOL_PREFERRED 1
#define MPOL_BIND 2
#define MPOL_INTERLEAVE 3
#define MPOL_LOCAL 4
#define MPOL_F_ADDR (1 << 1)
#define MPOL_MF_STRICT (1 << 0)
#define MPOL_MF_MOVE (1 << 1)
#define MPOL_F_MEMS_ALLOWED (1 << 2)
#define MPOL_F_NODE (1 << 0)
#define MPOL_F_ADDR (1 << 1)
#define NUMA_MAXNODE 1024  // Maximum number of numanodes

long mbind(const void* addr, const size_t len, const int mode,
           const unsigned long* nodemask, const unsigned long maxnode,
           const int flags);

long get_mempolicy(int* mode, unsigned long* nodemask, unsigned long maxnode,
                   void* addr, int flags);

int get_allowed_nodeset(unsigned long* nodeset, unsigned long maxnode,
                        unsigned long* numnode);

const char* policy_str(const int mode);

#endif  //__NUMA_H__
