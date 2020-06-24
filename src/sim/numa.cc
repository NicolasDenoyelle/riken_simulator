#include "sim/numa.hh"

#include "sim/system.hh"

size_t
NUMA::get_num_nodes(const System& sys)
{
    return sys.getMemoryPool().get_num_nodes();
}
