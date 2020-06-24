#ifndef __NUMA_HH__
#define __NUMA_HH__

#include "base/types.hh"

class System;

namespace NUMA {
/** The system maximum number of NUMANODES **/
static const long unsigned int MAXNUMANODES = 2048;

enum mode : uint32_t {
    // FOR MBIND/GET_MEMPOLICY
    // for MBIND since linux 2.6.7
    MBIND_MPOL_DEFAULT = 0,
    MBIND_MPOL_PREFERRED = 1,
    MBIND_MPOL_BIND = 2,
    MBIND_MPOL_INTERLEAVE = 3,
    MBIND_MPOL_LOCAL = 4,
};

enum flags : uint32_t {
    // since linux-2.6.16
    MBIND_MPOL_MF_STRICT = (1 << 0),
    MBIND_MPOL_MF_MOVE = (1 << 1),
    MBIND_MPOL_MF_MOVE_ALL = (1 << 2),
    // since linux-2.6.26
    MBIND_MPOL_F_STATIC_NODES,
    MBIND_MPOL_F_RELATIVE_NODES,
    // since linux 2.6.7
    GET_MEMPOLICY_MPOL_F_NODE = (1 << 0),
    GET_MEMPOLICY_MPOL_F_ADDR = (1 << 1),
    // since linux-2.6.24
    GET_MEMPOLICY_MPOL_F_MEMS_ALLOWED = (1 << 2),
};

class Bitmask {
   public:
    /** The maximum number of bits in bitmask **/
    static const long unsigned int MAXBITS = MAXNUMANODES;

   private:
    /** The number of basic type elements used to store bits **/
    static const long unsigned int NULONGS =
        MAXBITS / (8 * sizeof(long unsigned int));
    /** The bits **/
    long unsigned int mask[NULONGS];

   public:
    /** Empty (zeroed) bitmask **/
    Bitmask();

    /** Bitmask with bit n set. **/
    Bitmask(const size_t n);

    /** Bitmask with range [[i, ii]] set. **/
    Bitmask(const size_t i, const size_t ii);

    /** Copy constructor **/
    Bitmask(const Bitmask &src);

    /**
     * String conversion to bitmask.
     * @param str: A string value among the following:
     * - string output from to_string(),
     * - "all", "fill", "full" to set all bits,
     * - NULL, "none", "zero", "empty" to clear all bits,
     * - a string of ones and zeros,
     * - a comma separated list of bits index to set.
     **/
    Bitmask(const char *str);

    /** access elements in mask **/
    long unsigned int *get_mask() { return mask; }

    bool operator==(const Bitmask &rhs);
    bool operator<(const Bitmask &rhs);
    bool operator<=(const Bitmask &rhs);
    bool operator>(const Bitmask &rhs);
    bool operator>=(const Bitmask &rhs);
    Bitmask operator|(const Bitmask &rhs) const;
    Bitmask operator&(const Bitmask &rhs) const;
    Bitmask &operator|=(const Bitmask &rhs);
    Bitmask &operator&=(const Bitmask &rhs);
    Bitmask operator~() const;
    Bitmask operator!() const;
    operator bool() const;

    /** Copy Bitmask content. **/
    void copy_to(Bitmask &dst) const;
    void copy_from(const Bitmask &src);

    unsigned long *ulongs() { return mask; }
    unsigned long nulongs() { return NULONGS; }

    /** Empty a bitmask with all bits cleared. **/
    void zero();

    /** Set all bist in bitmask. **/
    void fill();

    /**
     * Bitwise comparison.
     * @param lhs: The left handside bitmask of comparison.
     * @param rhs: The right handside bitmask of comparison.
     * @return 1 if most significant bit of lhs is greater
     *than rhs MSB.
     * @return -1 if most significant bit of rhs is greater
     *than lhs MSB.
     * @return 0 If bitmask or equal bitwise.
     **/
    int bitwise_cmp(const Bitmask &rhs) const;

    /** Create a bitmask of ored bits **/
    Bitmask bitwise_or(const Bitmask &rhs) const;

    /** Create a bitmask of anded bits **/
    Bitmask bitwise_and(const Bitmask &rhs) const;

    /** Create a negated bitmask **/
    Bitmask bitwise_not() const;

    /**
     * Check whether a bit in bitmask is set.
     * @param i: the index of the bit to check.
     * @return -1 if i is greater than bitmask length.
     * @return 0 if bit is not set else a positive value.
     **/
    int isset(const long unsigned int i) const;

    /** Check whether all bits are zero. **/
    bool iszero() const;

    /** Check whether all bits in bitmask are set. **/
    bool isfull() const;

    /**
     * Set a bit in bitmask.
     * @param i: The bit to set in bitmask.
     * @return -1 if "i" is greater than bitmask length.
     * @return 0
     **/
    int set(const long unsigned int i);

    /**
     * Clear a bit in bitmask.
     * @param i: The bit to clear in bitmask.
     * @return -1 if i is greater than bitmask length.
     * @return 0
     **/
    int clear(const long unsigned int i);

    /**
     * Set a range [[i, ii]] of bits in bitmask.
     * @param i: The index of the first lower bit to set.
     * @param ii: The index of the last lower bit to set.
     * @return -1 if i or ii is greater than bitmask length.
     * @return 0
     **/
    int set_range(const long unsigned int i, const long unsigned int ii);

    /**
     * Clear a range [[i, ii]] of bits in bitmask.
     * @param i: The index of the first lower bit to clear.
     * @param ii: The index of the last lower bit to clear.
     * @return -1 if "i" or "ii" is greater than bitmask
     *length.
     * @return 0
     **/
    int clear_range(const long unsigned int i, const long unsigned int ii);

    /**
     * Count the number of bits set in bitmask.
     * @return The number of bits set in bitmask.
     **/
    long unsigned int nset() const;

    /**
     * Get index of the first bit set in bitmask.
     * @return -1 if no bit is set.
     * @return The index of last bit set in bitmask.
     **/
    long last() const;

    /**
     * Get index of the next bit set in bitmask.
     * Will loop if last bit or greater is provided.
     **/
    long next(long i) const;

    /**
     * Get index of the last bit set in bitmask.
     * @return -1 if no bit is set.
     * @return The index of first bit set in bitmask.
     **/
    long first() const;

    /** Bitmask conversion to string. Char * filled with '0'
     * and '1'. **/
    char *to_string() const;
};  // class Bitmask

/**
 * Memory policy for matching physical pages with system
 *memories.
 **/
class MemPolicy {
   private:
    Bitmask _nodeset;
    enum mode _mode;
    long unsigned int _flags;

   public:
    MemPolicy()
        : _nodeset(),
          _mode(NUMA::mode::MBIND_MPOL_DEFAULT),
          _flags(NUMA::flags::MBIND_MPOL_MF_MOVE) {}

    MemPolicy(const Bitmask &nodeset, const enum mode mode,
              const long unsigned int &flags)
        : _nodeset(nodeset), _mode(mode), _flags(flags) {}

    /** libnuma style arguments. **/
    MemPolicy(const long unsigned int *nodeset,
              const long unsigned int maxnode, const long unsigned int mode,
              const long unsigned int &flags)
        : _nodeset(), _mode(static_cast<enum mode>(mode)), _flags(flags) {
        unsigned long *mask = _nodeset.get_mask();
        const unsigned long nulongs =
            NUMA::Bitmask::MAXBITS / (8 * sizeof(*mask));
        const unsigned long maxlongs =
            maxnode / (8 * sizeof(*mask)) + maxnode % (8 * sizeof(*mask)) ? 1
                                                                          : 0;
        for (unsigned long i = 0; i < nulongs && i < maxlongs; i++)
            mask[i] = nodeset[i];
    }

    /** Copy constructor **/
    MemPolicy(const MemPolicy &p)
        : _nodeset(p._nodeset), _mode(p._mode), _flags(p._flags) {}

    const enum mode &mode() const { return _mode; }
    enum mode &mode() { return _mode; }
    long unsigned int &flags() { return _flags; }
    const long unsigned int &flags() const { return _flags; }

    Bitmask nodeset() const { return Bitmask(_nodeset); }
    Bitmask &nodeset() { return _nodeset; }
    unsigned long *mask() { return _nodeset.get_mask(); }

    void set(const MemPolicy &policy) {
        _nodeset.copy_from(policy.nodeset());
        _mode = policy.mode();
        _flags = policy.flags();
    }

    void set(const Bitmask &nodeset, const enum mode mode,
             const long unsigned int flags) {
        _nodeset.copy_from(nodeset);
        _mode = mode;
        _flags = flags;
    }
};  // class MemPolicy

size_t get_num_nodes(const System &sys);

};  // namespace NUMA

#endif  // __SIM_NUMA_HH__
