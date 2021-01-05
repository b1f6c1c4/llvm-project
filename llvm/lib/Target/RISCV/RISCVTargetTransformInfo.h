//===- RISCVTargetTransformInfo.h - RISC-V specific TTI ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file defines a TargetTransformInfo::Concept conforming object specific
/// to the RISC-V target machine. It uses the target's detailed information to
/// provide more precise answers to certain TTI queries, while letting the
/// target independent and default TTI implementations handle the rest.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RISCV_RISCVTARGETTRANSFORMINFO_H
#define LLVM_LIB_TARGET_RISCV_RISCVTARGETTRANSFORMINFO_H

#include "RISCVSubtarget.h"
#include "RISCVTargetMachine.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/IR/Function.h"

namespace llvm {

class RISCVTTIImpl : public BasicTTIImplBase<RISCVTTIImpl> {
  using BaseT = BasicTTIImplBase<RISCVTTIImpl>;
  using TTI = TargetTransformInfo;

  friend BaseT;

  const RISCVSubtarget *ST;
  const RISCVTargetLowering *TLI;

  const RISCVSubtarget *getST() const { return ST; }
  const RISCVTargetLowering *getTLI() const { return TLI; }

public:
  explicit RISCVTTIImpl(const RISCVTargetMachine *TM, const Function &F)
      : BaseT(TM, F.getParent()->getDataLayout()), ST(TM->getSubtargetImpl(F)),
        TLI(ST->getTargetLowering()) {}

  /// \name Scalar TTI Implementations
  /// @{

  int getIntImmCost(const APInt &Imm, Type *Ty, TTI::TargetCostKind CostKind);
  int getIntImmCostInst(unsigned Opcode, unsigned Idx, const APInt &Imm,
                        Type *Ty, TTI::TargetCostKind CostKind,
                        Instruction *Inst = nullptr);
  int getIntImmCostIntrin(Intrinsic::ID IID, unsigned Idx, const APInt &Imm,
                          Type *Ty, TTI::TargetCostKind CostKind);

  /// @}

  /// \name Vector TTI Implementations
  /// @{

  unsigned getNumberOfRegisters(unsigned ClassID) const {
    bool Vector = (ClassID == 1);
    if (Vector) {
      if (ST->hasStdExtV())
        return 32;
      return 0;
    }

    return 31;
  }

  unsigned getRegisterBitWidth(bool Vector) const {
    if (Vector) {
      if (ST->hasStdExtV())
        return 134217728;
      return 0;
    }

    return 32;
  }

  unsigned getMinVectorRegisterBitWidth() const {
    return 8;
  }

  int getArithmeticInstrCost(
      unsigned Opcode, Type *Ty,
      TTI::TargetCostKind CostKind = TTI::TCK_RecipThroughput,
      TTI::OperandValueKind Op1Info = TTI::OK_AnyValue,
      TTI::OperandValueKind Op2Info = TTI::OK_AnyValue,
      TTI::OperandValueProperties Opd1PropInfo = TTI::OP_None,
      TTI::OperandValueProperties Opd2PropInfo = TTI::OP_None,
      ArrayRef<const Value *> Args = ArrayRef<const Value *>(),
      const Instruction *CxtI = nullptr) const {
    switch (CostKind) {
      case TTI::TCK_CodeSize:
        // TODO: Make compressed instructions to be 1x
        // TODO: Make vector magic instructions to be 8x
        return 2;
      case TTI::TCK_SizeAndLatency: {
        int S = getArithmeticInstrCost(Opcode, Ty, TTI::TCK_CodeSize,
                                       Op1Info, Op2Info, Opd1PropInfo,
                                       Opd2PropInfo, Args, CxtI);
        int L = getArithmeticInstrCost(Opcode, Ty, TTI::TCK_Latency,
                                       Op1Info, Op2Info, Opd1PropInfo,
                                       Opd2PropInfo, Args, CxtI);
        return (S + 2 * L) / 3;
      }
      default: {
        int C = 1;
        Type *InnerTy = Ty;
        if (Ty->isVectorTy()) {
          InnerTy = Ty->getScalarType();
          C *= 512;
        }
        if (InnerTy->isFloatingPointTy()) {
          int SZ = InnerTy->getScalarSizeInBits();
          C *= SZ * std::log2(SZ);
        } else {
          C *= 4;
        }
        return C;
      }
    }
  }

  unsigned getShuffleCost(TTI::ShuffleKind Kind, VectorType *Ty, int Index,
                          VectorType *SubTp) const {
    return 262144;
  }

  unsigned getCastInstrCost(unsigned Opcode, Type *Dst, Type *Src,
                            TTI::CastContextHint CCH,
                            TTI::TargetCostKind CostKind,
                            const Instruction *I = nullptr) {
    switch (CostKind) {
      case TTI::TCK_CodeSize:
        // TODO: Make compressed instructions to be 1x
        // TODO: Make vector magic instructions to be 8x
        return 2;
      case TTI::TCK_SizeAndLatency: {
        int S = getCastInstrCost(Opcode, Dst, Src, CCH, TTI::TCK_CodeSize, I);
        int L = getCastInstrCost(Opcode, Dst, Src, CCH, TTI::TCK_Latency, I);
        return (S + 2 * L) / 3;
      }
      default: {
        int C = BaseT::getCastInstrCost(Opcode, Dst, Src, CCH, CostKind, I);
        if (Dst->isVectorTy() || Src->isVectorTy())
          C *= 1024;
        return C;
      }
    }
  }

  unsigned getExtractWithExtendCost(unsigned Opcode, Type *Dst,
                                    VectorType *VecTy, unsigned Index) const {
    int SZ = VecTy->getScalarSizeInBits();
    return SZ * 4 + 18;
  }

  unsigned getCmpSelInstrCost(unsigned Opcode, Type *ValTy, Type *CondTy,
                              CmpInst::Predicate VecPred,
                              TTI::TargetCostKind CostKind,
                              const Instruction *I = nullptr) const {
    switch (CostKind) {
      case TTI::TCK_CodeSize:
        // TODO: Make compressed instructions to be 1x
        // TODO: Make vector magic instructions to be 8x
        return 2;
      case TTI::TCK_SizeAndLatency: {
        int S = getCmpSelInstrCost(Opcode, ValTy, CondTy, VecPred,
                                   TTI::TCK_CodeSize, I);
        int L = getCmpSelInstrCost(Opcode, ValTy, CondTy, VecPred,
                                   TTI::TCK_Latency, I);
        return (S + 2 * L) / 3;
      }
      default: {
        if (ValTy && ValTy->isVectorTy() || CondTy && CondTy->isVectorTy())
          return 2048;
        return 2;
      }
    }
  }

  unsigned getVectorInstrCost(unsigned Opcode, Type *Val,
                              unsigned Index) const {
    int SZ = Val->getScalarSizeInBits();
    return SZ * 4 + 16;
  }

  // TODO: getMemoryOpCost
  // TODO: getMaskedMemoryOpCost

  unsigned getGatherScatterOpCost(unsigned Opcode, Type *DataTy,
                                  const Value *Ptr, bool VariableMask,
                                  Align Alignment, TTI::TargetCostKind CostKind,
                                  const Instruction *I = nullptr) const {
    int SZ = DataTy->getScalarSizeInBits();
    return SZ * 65536;
  }

  unsigned getArithmeticReductionCost(unsigned Opcode, VectorType *Ty,
                                      bool IsPairwiseForm,
                                      TTI::TargetCostKind CostKind) const {
    switch (CostKind) {
      case TTI::TCK_CodeSize:
        // TODO: Make compressed instructions to be 1x
        // TODO: Make vector magic instructions to be 8x
        return 2;
      case TTI::TCK_SizeAndLatency: {
        int S = getArithmeticReductionCost(Opcode, Ty, IsPairwiseForm,
                                           TTI::TCK_CodeSize);
        int L = getArithmeticReductionCost(Opcode, Ty, IsPairwiseForm,
                                           TTI::TCK_Latency);
        return (S + 2 * L) / 3;
      }
      default: {
        int SZ = Ty->getScalarSizeInBits();
        return SZ * 16384;
      }
    }
  }

  unsigned getMinMaxReductionCost(VectorType *Ty, VectorType *CondTy,
                                  bool IsPairwiseForm, bool IsUnsigned,
                                  TTI::TargetCostKind CostKind) const {
    switch (CostKind) {
      case TTI::TCK_CodeSize:
        // TODO: Make compressed instructions to be 1x
        // TODO: Make vector magic instructions to be 8x
        return 2;
      case TTI::TCK_SizeAndLatency: {
        int S = getMinMaxReductionCost(Ty, CondTy, IsPairwiseForm,
                                       IsUnsigned, TTI::TCK_CodeSize);
        int L = getMinMaxReductionCost(Ty, CondTy, IsPairwiseForm,
                                       IsUnsigned, TTI::TCK_Latency);
        return (S + 2 * L) / 3;
      }
      default: {
        int SZ = Ty->getScalarSizeInBits();
        return SZ * 16384;
      }
    }
  }

  // TODO: getIntrinsicInstrCost

  bool supportsScalableVectors() const { return true; }

  bool hasActiveVectorLength() const { return true; }

  /// @}
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_RISCV_RISCVTARGETTRANSFORMINFO_H
