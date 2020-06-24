#include <gtest/gtest.h>

#include <deque>
#include <vector>

#include "base/addr_range.hh"
#include "sim/mem_pool.hh"

const Addr page_size = (1UL<<TheISA::PageShift);

TEST(MemoryPoolTest, ContiguousMemPool)
{
    AddrRange range = RangeSize(1UL << 10, 1UL << 18); // 64 pages
    ContiguousMemPool pool(range);

    const size_t size = pool.size();
    const size_t npages = size / page_size;
    const Addr start = pool.start();
    const Addr end = pool.end();
    Addr addr;

    // Allocate more than available fails ENOMEM
    EXPECT_EQ(pool.allocate(npages + 1, addr), ENOMEM);

    // Allocate one page ok
    EXPECT_EQ(pool.allocate(1, addr), 0);

    // Free out of bound fails EDOM
    EXPECT_EQ(pool.free(1, start - page_size), EDOM);

    // Free on non-allocated memory fails EINVAL
    EXPECT_EQ(pool.free(1, addr + page_size), EINVAL);

    // Free ok
    EXPECT_EQ(pool.free(1, addr), 0);

    // allocate everything ok
    EXPECT_EQ(pool.allocate(npages, addr), 0);

    // free odd pages ok
    for (Addr a = start; a < end; a += 2*page_size)
        EXPECT_EQ(pool.free(1, a), 0);

    // allocate more than one page fails ENOMEM,
    // because all odd pages are allocated, and allocation
    // of sparse area is not possible.
    EXPECT_EQ(pool.allocate(2, addr), ENOMEM);

    // free even pages ok
    for (Addr a = start + page_size; a < end; a += 2*page_size)
        EXPECT_EQ(pool.free(1, a), 0);
}

TEST(MemoryPoolTest, NUMAMemPoolGenericTests)
{
    AddrRangeList numa_nodes = {
        RangeSize(2*page_size, 1UL << 18),
        RangeSize(3*page_size + (1UL << 18), (1UL << 19))};

    NUMAMemPool pool(numa_nodes);

    std::vector<AddrRange> chunks;
    const size_t size = pool.size();
    const size_t npages = size / page_size;
    const Addr start = pool.start();
    Addr addr;

    // Allocate more than available fails ENOMEM
    EXPECT_EQ(pool.allocate(npages + 1, addr), ENOMEM);

    // Allocate one page ok
    EXPECT_EQ(pool.allocate(1, addr), 0);

    // Free out of bound fails EDOM
    EXPECT_EQ(pool.free(1, start - page_size), EDOM);

    // Free on non-allocated memory fails EINVAL
    EXPECT_EQ(pool.free(1, addr + page_size), EINVAL);

    // Free ok
    EXPECT_EQ(pool.free(1, addr), 0);

    // allocate everything ok
    for (auto c = numa_nodes.begin(); c != numa_nodes.end(); c++)
        EXPECT_EQ(pool.allocate(c->size()/page_size, addr), 0);

    // free odd pages ok
    for (auto c = numa_nodes.begin(); c != numa_nodes.end(); c++)
        for (Addr a = c->start(); a < c->end(); a += 2*page_size)
            EXPECT_EQ(pool.free(1, a), 0);

    // allocate more than one page fails ENOMEM,
    // because all odd pages are allocated, and allocation
    // of sparse area is not possible.
    EXPECT_EQ(pool.allocate(2, addr), ENOMEM);

    // free even pages ok
    for (auto c = numa_nodes.begin(); c != numa_nodes.end(); c++)
        for (Addr a = c->start() + page_size; a < c->end(); a += 2*page_size)
            EXPECT_EQ(pool.free(1, a), 0);
}
