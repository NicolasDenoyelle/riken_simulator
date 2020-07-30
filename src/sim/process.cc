/*
 * Copyright (c) 2014-2016 Advanced Micro Devices, Inc.
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Ali Saidi
 *          Brandon Potter
 */

#include "sim/process.hh"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <csignal>
#include <map>
#include <string>
#include <vector>

#include "base/intmath.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "mem/se_translating_port_proxy.hh"
#include "params/Process.hh"
#include "sim/emul_driver.hh"
#include "sim/fd_array.hh"
#include "sim/fd_entry.hh"
#include "sim/numa.hh"
#include "sim/syscall_desc.hh"
#include "sim/system.hh"

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/linux/process.hh"

#elif THE_ISA == SPARC_ISA
#include "arch/sparc/linux/process.hh"
#include "arch/sparc/solaris/process.hh"

#elif THE_ISA == MIPS_ISA
#include "arch/mips/linux/process.hh"

#elif THE_ISA == ARM_ISA
#include "arch/arm/freebsd/process.hh"
#include "arch/arm/linux/process.hh"

#elif THE_ISA == X86_ISA
#include "arch/x86/linux/process.hh"

#elif THE_ISA == POWER_ISA
#include "arch/power/linux/process.hh"

#elif THE_ISA == RISCV_ISA
#include "arch/riscv/linux/process.hh"

#else
#error "THE_ISA not set"
#endif


using namespace std;
using namespace TheISA;

Process::Process(ProcessParams *params, EmulationPageTable *pTable,
                 ObjectFile *obj_file)
    : SimObject(params),
      mempolicy(
          // Allow all numa nodes
          NUMA::Bitmask(0, NUMA::get_num_nodes(*(params->system))-1),
          // Allocate on first memory available.
          // On linux this policy is MPOL_LOCAL. See NUMA.hh
          NUMA::MPOL_PREFERRED,
          // Unused because this is used when moving pages.
          NUMA::MPOL_MF_STRICT),
      allowed_nodeset(0, NUMA::get_num_nodes(*(params->system))-1),
      interleave_node(0),
      system(params->system),
      useArchPT(params->useArchPT),
      kvmInSE(params->kvmInSE),
      pTable(pTable),
      initVirtMem(system->getSystemPort(), this,
                  SETranslatingPortProxy::Always),
      objFile(obj_file),
      argv(params->cmd), envp(params->env), cwd(params->cwd),
      executable(params->executable),
      _uid(params->uid), _euid(params->euid),
      _gid(params->gid), _egid(params->egid),
      _pid(params->pid), _ppid(params->ppid),
      _pgid(params->pgid), drivers(params->drivers),
      fds(make_shared<FDArray>(params->input, params->output, params->errout)),
      childClearTID(0)
{
    if (_pid >= System::maxPID)
        fatal("_pid is too large: %d", _pid);

    auto ret_pair = system->PIDs.emplace(_pid);
    if (!ret_pair.second)
        fatal("_pid %d is already used", _pid);

    /**
     * Linux bundles together processes into this concept called a thread
     * group. The thread group is responsible for recording which processes
     * behave as threads within a process context. The thread group leader
     * is the process who's tgid is equal to its pid. Other processes which
     * belong to the thread group, but do not lead the thread group, are
     * treated as child threads. These threads are created by the clone system
     * call with options specified to create threads (differing from the
     * options used to implement a fork). By default, set up the tgid/pid
     * with a new, equivalent value. If CLONE_THREAD is specified, patch
     * the tgid value with the old process' value.
     */
    _tgid = params->pid;

    exitGroup = new bool();
    sigchld = new bool();

    if (!debugSymbolTable) {
        debugSymbolTable = new SymbolTable();
        if (!objFile->loadGlobalSymbols(debugSymbolTable) ||
            !objFile->loadLocalSymbols(debugSymbolTable) ||
            !objFile->loadWeakSymbols(debugSymbolTable)) {
            delete debugSymbolTable;
            debugSymbolTable = nullptr;
        }
    }
}

void
Process::clone(ThreadContext *otc, ThreadContext *ntc,
               Process *np, TheISA::IntReg flags)
{

    if (P_CLONE_VM & flags) {
        /**
         * Share the process memory address space between the new process
         * and the old process. Changes in one will be visible in the other
         * due to the pointer use.
         */
        delete np->pTable;
        np->pTable = pTable;
        ntc->getMemProxy().setPageTable(np->pTable);

        np->memState = memState;
    } else {
        /**
         * Duplicate the process memory address space. The state needs to be
         * copied over (rather than using pointers to share everything).
         */
        typedef std::vector<pair<Addr,Addr>> MapVec;
        MapVec mappings;
        pTable->getMappings(&mappings);

        for (auto map : mappings) {
            Addr paddr, vaddr = map.first;
            bool alloc_page = !(np->pTable->translate(vaddr, paddr));
            np->replicatePage(vaddr, paddr, otc, ntc, alloc_page);
        }

        *np->memState = *memState;
    }

    if (P_CLONE_FILES & flags) {
        /**
         * The parent and child file descriptors are shared because the
         * two FDArray pointers are pointing to the same FDArray. Opening
         * and closing file descriptors will be visible to both processes.
         */
        np->fds = fds;
    } else {
        /**
         * Copy the file descriptors from the old process into the new
         * child process. The file descriptors entry can be opened and
         * closed independently of the other process being considered. The
         * host file descriptors are also dup'd so that the flags for the
         * host file descriptor is independent of the other process.
         */
        for (int tgt_fd = 0; tgt_fd < fds->getSize(); tgt_fd++) {
            std::shared_ptr<FDArray> nfds = np->fds;
            std::shared_ptr<FDEntry> this_fde = (*fds)[tgt_fd];
            if (!this_fde) {
                nfds->setFDEntry(tgt_fd, nullptr);
                continue;
            }
            nfds->setFDEntry(tgt_fd, this_fde->clone());

            auto this_hbfd = std::dynamic_pointer_cast<HBFDEntry>(this_fde);
            if (!this_hbfd)
                continue;

            int this_sim_fd = this_hbfd->getSimFD();
            if (this_sim_fd <= 2)
                continue;

            int np_sim_fd = dup(this_sim_fd);
            assert(np_sim_fd != -1);

            auto nhbfd = std::dynamic_pointer_cast<HBFDEntry>((*nfds)[tgt_fd]);
            nhbfd->setSimFD(np_sim_fd);
        }
    }

    if (P_CLONE_THREAD & flags) {
        np->_tgid = _tgid;
        delete np->exitGroup;
        np->exitGroup = exitGroup;
    }

    np->argv.insert(np->argv.end(), argv.begin(), argv.end());
    np->envp.insert(np->envp.end(), envp.begin(), envp.end());
}

void
Process::regStats()
{
    SimObject::regStats();

    using namespace Stats;

    numSyscalls
        .name(name() + ".numSyscalls")
        .desc("Number of system calls")
        ;
}

ThreadContext *
Process::findFreeContext()
{
    for (auto &it : system->threadContexts) {
        if (ThreadContext::Halted == it->status())
            return it;
    }
    return nullptr;
}

void
Process::revokeThreadContext(int context_id)
{
    std::vector<ContextID>::iterator it;
    for (it = contextIds.begin(); it != contextIds.end(); it++) {
        if (*it == context_id) {
            contextIds.erase(it);
            return;
        }
    }
    warn("Unable to find thread context to revoke");
}

void
Process::initState()
{
    if (contextIds.empty())
        fatal("Process %s is not associated with any HW contexts!\n", name());

    // first thread context for this process... initialize & enable
    ThreadContext *tc = system->getThreadContext(contextIds[0]);

    // mark this context as active so it will start ticking.
    tc->activate();

    pTable->initState(tc);
}

DrainState
Process::drain()
{
    fds->updateFileOffsets();
    return DrainState::Drained;
}

// local function for computing available memory from a set of nodes.
size_t available(const NUMAMemPool &pmem, const NUMA::Bitmask &b) {
    size_t tot = 0;
    for (long i = 0; i < pmem.get_num_nodes(); i++) {
        if (b.isset(i)) tot += pmem.available(i);
    }
    return tot;
}

void
Process::allocateMem(Addr vaddr, int64_t size, bool clobber,
                     NUMA::MemPolicy const* policy)
{

    const int64_t npages = divCeil(size, (int64_t)PageBytes);
    const Addr page_size = pTable->getPageSize();
    const Addr start = pTable->pageAlign(vaddr);
    const Addr end = start + (npages * page_size);
    if (policy == NULL)
        policy = &mempolicy;
    int err;
    NUMA::Bitmask others =
        NUMA::Bitmask(NUMA::get_num_nodes(*system)) & (~policy->nodeset());
    long node = (policy->mode() == NUMA::MPOL_INTERLEAVE) ?
        interleave_node : policy->nodeset().first();
    NUMAMemPool& pmem = system->getMemoryPool();
    Addr paddr;
    uint64_t flags = clobber ?
        EmulationPageTable::Clobber :
        EmulationPageTable::MappingFlags(0);
    EmulationPageTable::Entry::set_numa_mode(flags, policy->mode());


    for (vaddr = start; vaddr < end; vaddr += page_size) {
        while ((err = pmem.allocate(1, paddr, node)) == ENOMEM) {
            // Maybe some memory is left but not in selected nodes.
            if (available(pmem, policy->nodeset()) == 0){
                // If we can select from other nodes,
                // and they have memory, do it.
                if (policy->mode() == NUMA::MPOL_PREFERRED &&
                    pmem.available() != 0)
                    if (!others.isset(node)) // Switch to other nodes
                        node = others.next(node);
                    else { // We are already on other nodes
                        node = others.next(node);
                        if (node == others.first()) // break loop
                            fatal("Out of m5 memory.");
                    }
                // else panic
                else
                  fatal("Out of m5 memory.");
            }
            // There is memory left on some nodes,
            // then try the next one in policy.
            // ok for DEFAULT, LOCAL, INTERLEAVE and BIND policies.
            else {
                node = policy->nodeset().next(node);
            }
        }
        // Unexpected error occures ?
        if (err != 0)
            fatal("Allocation of physical memory failed.");

        // Map obtained page.
        pTable->map(vaddr, paddr, page_size, flags);

        // If policy is INTERLEAVE, use next node for next page.
        if (policy->mode() == NUMA::MPOL_INTERLEAVE)
            node = policy->nodeset().next(node);
    }
    if (policy->mode() == NUMA::MPOL_INTERLEAVE)
        interleave_node = node;
}

int
Process::movePages(Addr vaddr,
                   const int64_t size,
                   const NUMA::MemPolicy& mempolicy)
{
    NUMA::Bitmask system_nodes =
        NUMA::Bitmask(0, NUMA::get_num_nodes(*system) - 1);

    // Check if nodeset in mask is valid
    if ((mempolicy.nodeset() & system_nodes).iszero() ||
        !(mempolicy.nodeset() & ~system_nodes).iszero()) {
        warn("mbind nodeset out of system nodeset.");
        return -EFAULT;
    }

    // Check that page is aligned on page boundary
    if (pTable->pageAlign(vaddr) != vaddr) {
        warn("mbind addr not on a page boundary.");
        return -EINVAL;
    }

    // Check that vaddr + size does not overflow
    if ((vaddr + (size_t) size) < vaddr){
        warn("mbind size too large.");
        return -EINVAL;
    }

    // Ensure that empy nodeset is provided with default policy
    if (mempolicy.mode() == NUMA::MPOL_DEFAULT &&
        !mempolicy.nodeset().iszero()) {
        warn("mbind policy default requires a empty nodeset.");
        return -EINVAL;
    }

    // Ensure that BIND/INTERLEAVE policies are provided with a non empty
    // nodeset
    if ((mempolicy.mode() == NUMA::MPOL_BIND ||
         mempolicy.mode() == NUMA::MPOL_INTERLEAVE) &&
        mempolicy.nodeset().iszero()) {
        warn("mbind policy BIND/INTERLEAVE requires a set nodeset.");
        return -EINVAL;
    }

    Addr paddr, old_paddr;
    uint64_t flags = 0;
    const Addr page_size = pTable->getPageSize();
    Addr start = vaddr;
    const EmulationPageTable::Entry *pe;
    const Addr end = vaddr + size - size % page_size +
        (size % page_size == 0 ? 0 : page_size);
    int err = 0;
    NUMAMemPool& pmem = system->getMemoryPool();
    enum NUMA::mode mode = mempolicy.mode();
    NUMA::Bitmask nodeset = NUMA::Bitmask(mempolicy.nodeset());

    // If policy is default, then use the process nodeset.
    if (mode == NUMA::MPOL_DEFAULT) {
        mode = this->mempolicy.mode();
        nodeset.copy_from(this->mempolicy.nodeset());
    }

    NUMA::Bitmask others = system_nodes & (~nodeset);
    long target_node = nodeset.first();
    if (mode == NUMA::MPOL_INTERLEAVE)
        target_node = nodeset.next(interleave_node);

    // for each contiguous page
    for (vaddr = start; vaddr < end; vaddr += page_size) {

        // Page may not be in page table yet.
        // After mmap, if movepages is invoked() the pagefault handler
        // might not be. Then allocate (physically) unallocated region
        // with mempolicy.
        if ((pe = pTable->lookup(vaddr)) == NULL) {
            do {
                vaddr += page_size;
            } while (vaddr < end && (pe = pTable->lookup(vaddr)) == NULL);
            allocateMem(start, vaddr-start, false, &mempolicy);
        }
        else {
            // The page is already allocated, then migrate it.
            // 1. Allocate replacement physical page.
            // 2. Unmap the current mapping in the page table.
            // 3. Remap vaddr to new physical memory in the page table.
            // 4. Free old physical memory.

            // 1. Allocate replacement physical page.
            // If no memory is available we may use another node when mempolicy
            // allows it.
            while ((err = pmem.allocate(1, paddr, target_node)) == ENOMEM){
                // No need to lookup this node anymore because it is out
                // of memory.
                if (others.isset(target_node))
                    others.clear(target_node);
                if (nodeset.isset(target_node))
                    nodeset.clear(target_node);

                // Maybe some memory is left in selected nodes.
                if (available(pmem, nodeset) > 0)
                    target_node = nodeset.next(target_node);
                // Maybe we are allowed to use other nodes.
                else if (mode == NUMA::MPOL_PREFERRED &&
                         available(pmem, others) > 0)
                    target_node = others.next(target_node);
                // else no memory available for policy.
                else
                    target_node = -1;

                // No next node ?
                if (target_node == -1)
                    return -ENOMEM;
                else
                    continue;
            }

            // 2. Unmap the current mapping in the page table.
            old_paddr = pe->paddr;
            flags = pe->flags;
            pTable->unmap(vaddr, page_size);

            // 3. Remap to new physical memory.
            EmulationPageTable::Entry::set_numa_mode(flags, mode);
            pTable->map(vaddr, paddr, page_size, flags);

            // 4. Free old mapping
            pmem.free(1, old_paddr);

            // If interleave policy, move to next node.
            if (mode == NUMA::MPOL_INTERLEAVE)
                target_node = nodeset.next(target_node);
        }
    }
    return 0; // everything went ok.
}

int Process::getMemPolicy(Addr vaddr, int64_t size,
                          NUMA::MemPolicy &mempolicy) {
    if (mempolicy.flags() == 0) {
        mempolicy.set(this->mempolicy);
    } else if (mempolicy.flags() &
               NUMA::MPOL_F_MEMS_ALLOWED) {
        mempolicy.nodeset().copy_from(allowed_nodeset);
    } else if (mempolicy.flags() & NUMA::MPOL_F_ADDR) {
        NUMA::Bitmask pe_nodeset;
        int nid;
        const NUMAMemPool &pmem = system->getMemoryPool();
        const EmulationPageTable::Entry *pe = pTable->lookup(vaddr);

        if (pe == NULL) return -EFAULT;
        if ((nid = pmem.locate(pe->paddr)) == -1) return -EFAULT;

        mempolicy.mode() = pe->get_numa_mode();
        if (mempolicy.flags() & NUMA::MPOL_F_NODE) {
            mempolicy.nodeset().zero();
            mempolicy.nodeset().set(nid);
        } else
            mempolicy.nodeset().copy_from(this->mempolicy.nodeset());
    } else if (mempolicy.flags() & NUMA::MPOL_F_NODE) {
        if (this->mempolicy.mode() == NUMA::MPOL_INTERLEAVE) {
            mempolicy.mode() = static_cast<NUMA::mode>(interleave_node);
        }
    }

    return 0;
}

void
Process::replicatePage(Addr vaddr, Addr new_paddr, ThreadContext *old_tc,
                       ThreadContext *new_tc, bool allocate_page)
{
    if (allocate_page)
        new_paddr = system->allocPhysPages(1);

    // Read from old physical page.
    uint8_t *buf_p = new uint8_t[PageBytes];
    old_tc->getMemProxy().readBlob(vaddr, buf_p, PageBytes);

    // Create new mapping in process address space by clobbering existing
    // mapping (if any existed) and then write to the new physical page.
    bool clobber = true;
    pTable->map(vaddr, new_paddr, PageBytes, clobber);
    new_tc->getMemProxy().writeBlob(vaddr, buf_p, PageBytes);
    delete[] buf_p;
}

bool
Process::fixupStackFault(Addr vaddr)
{
    Addr stack_min = memState->getStackMin();
    Addr stack_base = memState->getStackBase();
    Addr max_stack_size = memState->getMaxStackSize();

    // Check if this is already on the stack and there's just no page there
    // yet.
    if (vaddr >= stack_min && vaddr < stack_base) {
        allocateMem(roundDown(vaddr, PageBytes), PageBytes);
        return true;
    }

    // We've accessed the next page of the stack, so extend it to include
    // this address.
    if (vaddr < stack_min && vaddr >= stack_base - max_stack_size) {
        while (vaddr < stack_min) {
            stack_min -= TheISA::PageBytes;
            if (stack_base - stack_min > max_stack_size)
                fatal("Maximum stack size exceeded\n");
            allocateMem(stack_min, TheISA::PageBytes);
            inform("Increasing stack size by one page.");
        }
        memState->setStackMin(stack_min);
        return true;
    }
    return false;
}

void
Process::serialize(CheckpointOut &cp) const
{
    memState->serialize(cp);
    pTable->serialize(cp);
    /**
     * Checkpoints for file descriptors currently do not work. Need to
     * come back and fix them at a later date.
     */

    warn("Checkpoints for file descriptors currently do not work.");
#if 0
    for (int x = 0; x < fds->getSize(); x++)
        (*fds)[x].serializeSection(cp, csprintf("FDEntry%d", x));
#endif

}

void
Process::unserialize(CheckpointIn &cp)
{
    memState->unserialize(cp);
    pTable->unserialize(cp);
    /**
     * Checkpoints for file descriptors currently do not work. Need to
     * come back and fix them at a later date.
     */
    warn("Checkpoints for file descriptors currently do not work.");
#if 0
    for (int x = 0; x < fds->getSize(); x++)
        (*fds)[x]->unserializeSection(cp, csprintf("FDEntry%d", x));
    fds->restoreFileOffsets();
#endif
    // The above returns a bool so that you could do something if you don't
    // find the param in the checkpoint if you wanted to, like set a default
    // but in this case we'll just stick with the instantiated value if not
    // found.
}

bool Process::map(Addr vaddr, Addr paddr, int size, bool cacheable) {
  uint64_t flags = cacheable ? EmulationPageTable::MappingFlags(0)
                             : EmulationPageTable::Uncacheable;
  EmulationPageTable::Entry::set_numa_mode(flags, mempolicy.mode());
  pTable->map(vaddr, paddr, size, flags);
  return true;
}

void
Process::syscall(int64_t callnum, ThreadContext *tc, Fault *fault)
{
    numSyscalls++;

    SyscallDesc *desc = getDesc(callnum);
    if (desc == nullptr)
        fatal("Syscall %d out of range", callnum);

    desc->doSyscall(callnum, this, tc, fault);
}

IntReg
Process::getSyscallArg(ThreadContext *tc, int &i, int width)
{
    return getSyscallArg(tc, i);
}

EmulatedDriver *
Process::findDriver(std::string filename)
{
    for (EmulatedDriver *d : drivers) {
        if (d->match(filename))
            return d;
    }

    return nullptr;
}

void
Process::updateBias()
{
    ObjectFile *interp = objFile->getInterpreter();

    if (!interp || !interp->relocatable())
        return;

    // Determine how large the interpreters footprint will be in the process
    // address space.
    Addr interp_mapsize = roundUp(interp->mapSize(), TheISA::PageBytes);

    // We are allocating the memory area; set the bias to the lowest address
    // in the allocated memory region.
    Addr mmap_end = memState->getMmapEnd();
    Addr ld_bias = mmapGrowsDown() ? mmap_end - interp_mapsize : mmap_end;

    // Adjust the process mmap area to give the interpreter room; the real
    // execve system call would just invoke the kernel's internal mmap
    // functions to make these adjustments.
    mmap_end = mmapGrowsDown() ? ld_bias : mmap_end + interp_mapsize;
    memState->setMmapEnd(mmap_end);

    interp->updateBias(ld_bias);
}

ObjectFile *
Process::getInterpreter()
{
    return objFile->getInterpreter();
}

Addr
Process::getBias()
{
    ObjectFile *interp = getInterpreter();

    return interp ? interp->bias() : objFile->bias();
}

Addr
Process::getStartPC()
{
    ObjectFile *interp = getInterpreter();

    return interp ? interp->entryPoint() : objFile->entryPoint();
}

Process *
ProcessParams::create()
{
    Process *process = nullptr;

    // If not specified, set the executable parameter equal to the
    // simulated system's zeroth command line parameter
    if (executable == "") {
        executable = cmd[0];
    }

    ObjectFile *obj_file = createObjectFile(executable);
    if (obj_file == nullptr) {
        fatal("Can't load object file %s", executable);
    }

#if THE_ISA == ALPHA_ISA
    if (obj_file->getArch() != ObjectFile::Alpha)
        fatal("Object file architecture does not match compiled ISA (Alpha).");

    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new AlphaLinuxProcess(this, obj_file);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == SPARC_ISA
    if (obj_file->getArch() != ObjectFile::SPARC64 &&
        obj_file->getArch() != ObjectFile::SPARC32)
        fatal("Object file architecture does not match compiled ISA (SPARC).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (obj_file->getArch() == ObjectFile::SPARC64) {
            process = new Sparc64LinuxProcess(this, obj_file);
        } else {
            process = new Sparc32LinuxProcess(this, obj_file);
        }
        break;

      case ObjectFile::Solaris:
        process = new SparcSolarisProcess(this, obj_file);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == X86_ISA
    if (obj_file->getArch() != ObjectFile::X86_64 &&
        obj_file->getArch() != ObjectFile::I386)
        fatal("Object file architecture does not match compiled ISA (x86).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (obj_file->getArch() == ObjectFile::X86_64) {
            process = new X86_64LinuxProcess(this, obj_file);
        } else {
            process = new I386LinuxProcess(this, obj_file);
        }
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == MIPS_ISA
    if (obj_file->getArch() != ObjectFile::Mips)
        fatal("Object file architecture does not match compiled ISA (MIPS).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new MipsLinuxProcess(this, obj_file);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == ARM_ISA
    ObjectFile::Arch arch = obj_file->getArch();
    if (arch != ObjectFile::Arm && arch != ObjectFile::Thumb &&
        arch != ObjectFile::Arm64)
        fatal("Object file architecture does not match compiled ISA (ARM).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        if (arch == ObjectFile::Arm64) {
            process = new ArmLinuxProcess64(this, obj_file,
                                            obj_file->getArch());
        } else {
            process = new ArmLinuxProcess32(this, obj_file,
                                            obj_file->getArch());
        }
        break;
      case ObjectFile::FreeBSD:
        if (arch == ObjectFile::Arm64) {
            process = new ArmFreebsdProcess64(this, obj_file,
                                              obj_file->getArch());
        } else {
            process = new ArmFreebsdProcess32(this, obj_file,
                                              obj_file->getArch());
        }
        break;
      case ObjectFile::LinuxArmOABI:
        fatal("M5 does not support ARM OABI binaries. Please recompile with an"
              " EABI compiler.");
      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == POWER_ISA
    if (obj_file->getArch() != ObjectFile::Power)
        fatal("Object file architecture does not match compiled ISA (Power).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new PowerLinuxProcess(this, obj_file);
        break;

      default:
        fatal("Unknown/unsupported operating system.");
    }
#elif THE_ISA == RISCV_ISA
    if (obj_file->getArch() != ObjectFile::Riscv)
        fatal("Object file architecture does not match compiled ISA (RISCV).");
    switch (obj_file->getOpSys()) {
      case ObjectFile::UnknownOpSys:
        warn("Unknown operating system; assuming Linux.");
        // fall through
      case ObjectFile::Linux:
        process = new RiscvLinuxProcess(this, obj_file);
        break;
      default:
        fatal("Unknown/unsupported operating system.");
    }
#else
#error "THE_ISA not set"
#endif

    if (process == nullptr)
        fatal("Unknown error creating process object.");
    return process;
}

std::string
Process::fullPath(const std::string &file_name)
{
    if (file_name[0] == '/' || cwd.empty())
        return file_name;

    std::string full = cwd;

    if (cwd[cwd.size() - 1] != '/')
        full += '/';

    return full + file_name;
}
