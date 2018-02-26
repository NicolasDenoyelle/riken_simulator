/*
 * Copyright (c) 2018 ARM Limited
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
 * Authors: Giacomo Gabrielli
 */

#ifndef __ARCH_ARM_SVE_MACROMEM_HH__
#define __ARCH_ARM_SVE_MACROMEM_HH__

#include "arch/arm/generated/decoder.hh"
#include "arch/arm/insts/pred_inst.hh"

namespace ArmISA {

template <typename RegElemType, typename MemElemType,
          template <typename, typename> class MicroopType>
class SveIndexedMemVI : public PredMacroOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex gp;
    IntRegIndex base;
    uint64_t imm;

  public:
    SveIndexedMemVI(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                    IntRegIndex _dest, IntRegIndex _gp, IntRegIndex _base,
                    uint64_t _imm)
        : PredMacroOp(mnem, machInst, __opClass),
          dest(_dest), gp(_gp), base(_base), imm(_imm)
    {
        numMicroops = ((machInst.sveLen + 1) * 16) / sizeof(RegElemType);

        microOps = new StaticInstPtr[numMicroops];

        StaticInstPtr *uop = microOps;

        for (int i = 0; i < numMicroops; i++, uop++) {
            *uop = new MicroopType<RegElemType, MemElemType>(
                mnem, machInst, __opClass, _dest, _gp, _base, _imm, i,
                numMicroops);
        }

        --uop;
        (*uop)->setLastMicroop();
        microOps[0]->setFirstMicroop();

        for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
            (*uop)->setDelayedCommit();
        }
    }

    Fault
    execute(ExecContext *, Trace::InstRecord *) const
    {
        panic("Execute method called when it shouldn't!");
        return NoFault;
    }

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const
    {
        // TODO: add suffix to transfer and base registers
        std::stringstream ss;
        printMnemonic(ss, "", false);
        ccprintf(ss, "{");
        printVecReg(ss, dest, true);
        ccprintf(ss, "}, ");
        printPredReg(ss, gp);
        ccprintf(ss, "/z, [");
        printVecReg(ss, base, true);
        if (imm != 0) {
            ccprintf(ss, ", #%d", imm * sizeof(MemElemType));
        }
        ccprintf(ss, "]");
        return ss.str();
    }
};

template <typename RegElemType, typename MemElemType,
          template <typename, typename> class MicroopType>
class SveIndexedMemSV : public PredMacroOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex gp;
    IntRegIndex base;
    IntRegIndex offset;

    bool offsetIs32;
    bool offsetIsSigned;
    bool offsetIsScaled;

  public:
    SveIndexedMemSV(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                    IntRegIndex _dest, IntRegIndex _gp, IntRegIndex _base,
                    IntRegIndex _offset, bool _offsetIs32,
                    bool _offsetIsSigned, bool _offsetIsScaled)
        : PredMacroOp(mnem, machInst, __opClass),
          dest(_dest), gp(_gp), base(_base), offset(_offset),
          offsetIs32(_offsetIs32), offsetIsSigned(_offsetIsSigned),
          offsetIsScaled(_offsetIsScaled)
    {
        numMicroops = ((machInst.sveLen + 1) * 16) / sizeof(RegElemType);

        microOps = new StaticInstPtr[numMicroops];

        StaticInstPtr *uop = microOps;

        for (int i = 0; i < numMicroops; i++, uop++) {
            *uop = new MicroopType<RegElemType, MemElemType>(
                mnem, machInst, __opClass, _dest, _gp, _base, _offset,
                _offsetIs32, _offsetIsSigned, _offsetIsScaled, i, numMicroops);
        }

        --uop;
        (*uop)->setLastMicroop();
        microOps[0]->setFirstMicroop();

        for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
            (*uop)->setDelayedCommit();
        }
    }

    Fault
    execute(ExecContext *, Trace::InstRecord *) const
    {
        panic("Execute method called when it shouldn't!");
        return NoFault;
    }

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const
    {
        // TODO: add suffix to transfer and base registers
        std::stringstream ss;
        printMnemonic(ss, "", false);
        ccprintf(ss, "{");
        printVecReg(ss, dest, true);
        ccprintf(ss, "}, ");
        printPredReg(ss, gp);
        ccprintf(ss, "/z, [");
        printIntReg(ss, base);
        ccprintf(ss, ", ");
        printVecReg(ss, offset, true);
        ccprintf(ss, "]");
        return ss.str();
    }
};

}  // namespace ArmISA

#endif  // __ARCH_ARM_SVE_MACROMEM_HH__
