#include "sim/mem_pool.hh"

#include <algorithm>
#include <cerrno>
#include <cstdio>

#include "base/logging.hh"

//------------------------------------------------------------------------
// ContiguousMemPool implementation
//------------------------------------------------------------------------

/**
 * Functor for sorting sizes in descending order.
 * If sizes are equal, put smallest address first.
 **/
static struct size_fnctr_cmp {
    bool operator()(const AddrRange& i, const AddrRange& j)
    {
        return i.size() == j.size() ? i.start() < j.start()
                                    : i.size() > j.size();
    }
} size_fnctr;

/** Functor for sorting addresses in ascending order. **/
static struct addr_fnctr_cmp {
    bool operator()(const AddrRange& i, const AddrRange& j)
    {
        return i.start() < j.start();
    }
} addr_fnctr;

void
ContiguousMemPool::defragment(std::deque<AddrRange> pool)
{
    AddrRange current, next;

    // If empty, or contains a single element,
    // then nothing to do.
    if (pool.size() <= 1)
        return;

    // Sort address wise for defragmenting
    std::sort(pool.begin(), pool.end(), addr_fnctr);

    // Get first chunk out.
    current = pool.front();
    pool.pop_front();

    // pop front chunks and merge them if they are
    // contiguous
    size_t n = pool.size();
    for (size_t i = 0; i < n; i++) {
        next = pool.front();
        pool.pop_front();

        if (!current.append(next)) {
            pool.push_back(current);
            current = next;
        }
    }

    // Push back last chunk
    pool.push_back(current);

    // Sort size wise to put large chunks front.
    std::sort(pool.begin(), pool.end(), size_fnctr);
}

void
ContiguousMemPool::init(AddrRange range)
{
    Addr start = range.start() >> TheISA::PageShift;
    Addr end = range.end() >> TheISA::PageShift;
    _range = AddrRange(range, start, end - start);
    _pool.clear();
    _pool.push_back(_range);
    _isDefragmented = true;
}

ContiguousMemPool::ContiguousMemPool(AddrRange r) { init(r); }

AddrRange
ContiguousMemPool::range() const
{
    Addr start = _range.start() << TheISA::PageShift;
    Addr end = _range.end() << TheISA::PageShift;
    return AddrRange(_range, start, end - start);
}

size_t
ContiguousMemPool::size() const
{
    return _range.size() << TheISA::PageShift;
}

Addr
ContiguousMemPool::start() const
{
    return _range.start() << TheISA::PageShift;
}

Addr
ContiguousMemPool::end() const
{
    return _range.end() << TheISA::PageShift;
}

bool
ContiguousMemPool::contains(const Addr addr) const
{
    return _range.contains(addr >> TheISA::PageShift);
}

size_t
ContiguousMemPool::available() const
{
    size_t tot = (size_t)std::accumulate(_pool.cbegin(), _pool.cend(), Addr(0),
        [](const Addr& a, const AddrRange& b) { return a + b.size(); });
    return tot << TheISA::PageShift;
}

int
ContiguousMemPool::free(const size_t npage, const Addr addr)
{
    AddrRange range = AddrRange(_range, addr >> TheISA::PageShift, npage);

    // Make sure chunk is not out of bounds
    if (!range.isSubset(_range))
        return EDOM;

    // Make sure chunk does not overlap available chunks
    for (auto ch = _pool.cbegin(); ch != _pool.cend(); ch++)
        if (range.intersects(*ch))
            return EINVAL;

    // Greedy optimization:
    // If the chunk appends to head then do it.
    // Else put the chunk at the back.
    // In the case of mbind, the syscall usually follow
    // mbind and calls free.
    if (range.append(_pool.front())) {
        _pool.pop_front();
        _pool.push_front(range);
    } else {
        _pool.push_back(range);
        _isDefragmented = false; // not defragmented anymore
    }

    return 0;
}

int
ContiguousMemPool::allocate(const size_t npage, Addr& addr)
{
    if (_pool.empty())
        return ENOMEM;

    // If we are lucky, front chunk is large enough.
    AddrRange c = _pool.front();
    _pool.pop_front();

    // Unlucky
    if (c.size() < npage) {
        _pool.push_back(c);    // put back in the pool.
        if (_isDefragmented) { // If the pool is
                               // defragmented, then out
                               // of memory.
            return ENOMEM;
        } else { // Lets defragment chunks and put big
                 // chunks front and try again.
            defragment();
            return this->allocate(npage, addr);
        }
    }

    // Lucky
    AddrRange lhs, rhs;
    c.split(npage, lhs, rhs);

    // push back remaining piece.
    if (rhs.size() > 0) {
        _pool.push_back(rhs);

        // Sort from largest size to smallest size.
        // Break ties ordering address in ascending order.
        if (_pool.size() > 1 && (++_pool.crbegin())->size() < rhs.size())
            std::sort(_pool.begin(), _pool.end(), size_fnctr);
    }

    // store chunk addr
    addr = lhs.start() << TheISA::PageShift;
    return 0;
}

void ContiguousMemPool::serialize(CheckpointOut& cp) const {
    paramOut(cp, "addr_start", _range.start());
    paramOut(cp, "addr_end", _range.end());
    paramOut(cp, "defragmented", _isDefragmented);
    paramOut(cp, "count", _pool.size());
    for (auto chunk : _pool) {
        paramOut(cp, "chunk.addr_start", chunk.start());
        paramOut(cp, "chunk.addr_end", chunk.end());
    }
}

void ContiguousMemPool::unserialize(CheckpointIn& cp){
    Addr start, end;
    size_t count;

    _pool.clear();
    paramIn(cp, "addr_start", start);
    paramIn(cp, "addr_end", end);
    _range = RangeEx(start, end);
    paramIn(cp, "defragmented", _isDefragmented);
    paramIn(cp, "count", count);

    for (size_t i = 0; i < count; i++) {
        paramIn(cp, "chunk.addr_start", start);
        paramIn(cp, "chunk.addr_end", end);
        _pool.push_back(RangeEx(start, end));
    }
}

//------------------------------------------------------------------------
// NUMAMemPool implementation
//------------------------------------------------------------------------

void
NUMAMemPool::init(const AddrRangeList memories)
{
    _pool.clear();
    _addrMap.clear();

    for (auto range = memories.begin(); range != memories.end(); range++) {
        for (auto pool = _pool.begin(); pool != _pool.end(); pool++)
            if (range->intersects(pool->range()))
                fatal("Cannot have overlapping physical address ranges in "
                      "memory pool.");
        _pool.push_back(ContiguousMemPool(*range));
    }

    auto pool = _pool.begin();
    auto range = memories.begin();
    for (; pool != _pool.end(); pool++) {
        _addrMap.insert(AddrRange(*range), &(*pool));
        range++;
    }
}

NUMAMemPool::NUMAMemPool(const AddrRangeList memories)
{
    init(memories);
}

size_t
NUMAMemPool::size() const
{
    return std::accumulate(_pool.cbegin(), _pool.cend(), size_t(0),
        [](const size_t& a, const ContiguousMemPool& b) {
            return a + b.size();
        });
}

Addr
NUMAMemPool::start() const
{
    return _addrMap.begin()->first.start();
}

Addr
NUMAMemPool::end() const
{
    auto last = _addrMap.end();
    last--;
    return last->first.end();
}

bool
NUMAMemPool::contains(const Addr addr) const
{
    return _addrMap.find(addr) != _addrMap.end();
}

int NUMAMemPool::locate(const Addr addr) const
{
    auto it = _addrMap.find(addr);
    if (it == _addrMap.end())
        return -1;
    return std::distance(&(*_pool.begin()),
                         static_cast<
                         const ContiguousMemPool*>(&(*it->second)));
}

size_t
NUMAMemPool::available() const
{
    return std::accumulate(_pool.cbegin(), _pool.cend(), size_t(0),
        [](const size_t& a, const ContiguousMemPool& b) {
            return a + b.available();
        });
}

int
NUMAMemPool::free(const size_t npage, const Addr addr)
{
    auto it = _addrMap.find(addr);
    return it != _addrMap.end() ? it->second->free(npage, addr) : EDOM;
}

int
NUMAMemPool::allocate(const size_t npage, Addr& addr)
{
    for (auto it = _pool.begin(); it != _pool.end(); it++) {
        if (it->allocate(npage, addr) == 0)
            return 0;
    }

    return ENOMEM;
}

int
NUMAMemPool::allocate(const size_t npage, Addr& addr, const unsigned nid)
{
    if (_pool.size() <= nid)
        return EDOM;
    return _pool.at(nid).allocate(npage, addr);
}

void NUMAMemPool::serialize(CheckpointOut& cp) const {
    size_t count = 0;

    paramOut(cp, "count", _pool.size());
    for (auto p : _pool) {
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", count++));
        p.serialize(cp);
    }
}

void NUMAMemPool::unserialize(CheckpointIn& cp){
    size_t count;

    _pool.clear();
    _addrMap.clear();
    paramIn(cp, "count", count);

    for (size_t i = 0; i < count; i++) {
        ScopedCheckpointSection sec(cp, csprintf("Entry%d", i));
        ContiguousMemPool p;

        p.unserialize(cp);
        _pool.push_back(p);
    }

    for (auto pool = _pool.begin(); pool != _pool.end(); pool++)
        _addrMap.insert(pool->range(), &(*pool));
}
