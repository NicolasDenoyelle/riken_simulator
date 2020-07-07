#ifndef __MEMPOOL_HH__
#define __MEMPOOL_HH__

#include <deque>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "arch/utility.hh"
#include "base/addr_range.hh"
#include "base/addr_range_map.hh"
#include "base/types.hh"
#ifndef TEST
#include "mem/physical.hh"
#include "sim/serialize.hh"
#endif

/**
 * Physical Memory Allocator Abstraction.
 * Input and Output Addr in Memory pool are page Addr >> TheISA::PageShift
 **/
class MemoryPool {
  public:
    /** Start address of the pool **/
    virtual Addr start() const = 0;

    /**
     * Address equal to end is included in the pool.
     **/
    virtual Addr end() const = 0;

    /** Total size of the pool **/
    virtual size_t size() const = 0;

    /**
     * Total not allocated number of pages in memory pool.
     * If memory is fragmented, it is different from the
     * allocable size.
     **/
    virtual size_t available() const = 0;

    /** Check if address is in the pool. (page Addr >> TheISA::PageShift) **/
    virtual bool contains(const Addr addr) const = 0;

    /**
     * Allocate one chunk of memory.
     * @param size: The minimum number of pages.
     * @param addr: Output address of allocation.
     * @return 0 on success.
     * @return ENOMEM if a single chunk cannot fit npages.
     **/
    virtual int allocate(const size_t npage, Addr &addr) = 0;

    /**
     * Free memory.
     * @param addr: (Addr >> TheISA::PageShift)
     * @return 0 on success;
     * @return EDOM if chunk is out of bounds.
     * @return EINVAL if chunk overlaps an area in the pool.
     **/
    virtual int free(const size_t npage, const Addr addr) = 0;
};

#ifndef TEST
/**
 * MemoryPool used in "sim/system.cc" to provide physical
 * memory.
 **/
class LegacyMemPool : public MemoryPool , public Serializable
{
  private:
    Addr pagePtr;
    PhysicalMemory* physmem;

  public:
    LegacyMemPool() : pagePtr(0), physmem(NULL) {}
    LegacyMemPool(PhysicalMemory* _physmem) : pagePtr(0), physmem(_physmem) {}

    void init(PhysicalMemory *_physmem) { physmem = _physmem; }

    Addr start() const { return 0; }

    Addr end() const { return physmem->totalSize(); }

    size_t size() const { return physmem->totalSize(); }

    bool contains(const Addr addr) const { return physmem->isMemAddr(addr); }

    size_t available() const
    {
        return (physmem->totalSize() >> TheISA::PageShift) - (pagePtr);
    }

    int allocate(const size_t npages, Addr& addr)
    {
        Addr return_addr = pagePtr << TheISA::PageShift;
        pagePtr += npages;

        Addr next_return_addr = pagePtr << TheISA::PageShift;

        AddrRange m5opRange(0xffff0000, 0x100000000);
        if (m5opRange.contains(next_return_addr)) {
            warn("Reached m5ops MMIO region\n");
            return_addr = 0xffffffff;
            pagePtr = 0xffffffff >> TheISA::PageShift;
        }

        if ((pagePtr << TheISA::PageShift) > physmem->totalSize()) {
            pagePtr -= npages;
            return ENOMEM;
        }

        addr = return_addr >> TheISA::PageShift;
        return 0;
    }

    int free(const size_t npages, const Addr addr)
    {
        if ((npages << TheISA::PageShift) != physmem->totalSize() ||
            addr != 0) {
            warn("Legacy memory can only free the whole "
                 "memory.");
            return EINVAL;
        }

        pagePtr = 0;
        return 0;
    }
    void serialize(CheckpointOut& cp) const { SERIALIZE_SCALAR(pagePtr); }
    void unserialize(CheckpointIn& cp) { UNSERIALIZE_SCALAR(pagePtr); }
};
#endif

/**
 * MemoryPool implementation for continuous and contiguous
 * memory address space. Will not perform well in presence of
 * lot of fragmentation. Hopefully, this should not happen in
 * physical memory.
 **/
class ContiguousMemPool : public MemoryPool
#ifndef TEST
                        , public Serializable
#endif
{
  private: // Attributes
    AddrRange _range;            // Memory pool range.
    std::deque<AddrRange> _pool; // Available memory
    bool _isDefragmented;        // Whether chunks are sorted and
                                 // defragmented.

  private: // Methods
    void defragment() { _isDefragmented = true; defragment(_pool); }

  public: // Methods
    ContiguousMemPool(AddrRange range);
    ContiguousMemPool(){}
    ~ContiguousMemPool(){}

    /**
     * Merge contiguous chunks and
     * sort them npages wise in descending order.
     **/
    static void defragment(std::deque<AddrRange> pool);

    void init(AddrRange range);
    AddrRange range() const;
    size_t size() const;
    Addr start() const;
    Addr end() const;
    bool contains(const Addr addr) const;
    size_t available() const;
    int allocate(const size_t npages, Addr &addr);
    int free(const size_t npages, const Addr addr);
#ifndef TEST
    void serialize(CheckpointOut& cp) const;
    void unserialize(CheckpointIn& cp);
#endif
};

/** MemoryPool implementation for a set of memories. **/
class NUMAMemPool : public MemoryPool
#ifndef TEST
                  , public Serializable
#endif
{
  private:
    // Allocator for each memory
    std::vector<ContiguousMemPool> _pool;

    // Map addr with their pool allocator
    AddrRangeMap<ContiguousMemPool*> _addrMap;

  public:
    // Construct allocator for a set of memories.
    NUMAMemPool(const AddrRangeList memories);

    // Construct empty allocator.
    NUMAMemPool() {}

    // Initialize NUMA memory pool from as set of addr_range.
    void init(const AddrRangeList memories);

    // Get AddrRange of a specific node
    AddrRange range(const size_t nid) const { return _pool.at(nid).range(); }

    // Get the number of NUMA Nodes (AddrRanges) in the pool.
    size_t get_num_nodes() const { return _pool.size(); }

    size_t size() const;
    Addr start() const;
    Addr end() const;
    bool contains(const Addr addr) const;
    // Get node id of addr or -1 if none of pools contain addr.
    int locate(const Addr addr) const;
    size_t available() const;
    size_t available(const size_t nid) const {
        return _pool.at(nid).available();
    }
    int free(const size_t npage, const Addr addr);
    int allocate(const size_t npage, Addr &addr);
    int allocate(const size_t npage, Addr &addr, const unsigned nid);
#ifndef TEST
    void serialize(CheckpointOut& cp) const;
    void unserialize(CheckpointIn& cp);
#endif
};

#endif // __MEMPOOL_HH__
