//===-- RISCVAsmPrinter.cpp - RISCV LLVM assembly writer ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to the RISCV assembly language.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/RISCVInstPrinter.h"
#include "MCTargetDesc/RISCVMCExpr.h"
#include "MCTargetDesc/RISCVTargetStreamer.h"
#include "RISCV.h"
#include "RISCVTargetMachine.h"
#include "TargetInfo/RISCVTargetInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

STATISTIC(RISCVNumInstrsCompressed,
          "Number of RISC-V Compressed instructions emitted");

namespace {
class RISCVAsmPrinter : public AsmPrinter {
  const MCSubtargetInfo *STI;

public:
  explicit RISCVAsmPrinter(TargetMachine &TM,
                           std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)), STI(TM.getMCSubtargetInfo()),
        MagicEmitter(nullptr) {
    if (STI->getFeatureBits()[RISCV::FeatureStdExtZvmagic])
      MagicEmitter = createRISCVMCCodeEmitter(*TM.getMCInstrInfo(),
                                              *TM.getMCRegisterInfo(),
                                              OutStreamer->getContext());
  }

  ~RISCVAsmPrinter() {
    if (MagicEmitter)
      delete MagicEmitter;
  }

  StringRef getPassName() const override { return "RISCV Assembly Printer"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void emitInstruction(const MachineInstr *MI) override;

  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                       const char *ExtraCode, raw_ostream &OS) override;
  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                             const char *ExtraCode, raw_ostream &OS) override;

  void EmitToStreamer(MCStreamer &S, const MCInst &Inst);
  bool emitPseudoExpansionLowering(MCStreamer &OutStreamer,
                                   const MachineInstr *MI);

  // Wrapper needed for tblgenned pseudo lowering.
  bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp) const {
    return LowerRISCVMachineOperandToMCOperand(MO, MCOp, *this);
  }

  void emitStartOfAsmFile(Module &M) override;
  void emitEndOfAsmFile(Module &M) override;

private:
  void emitAttributes();
  void emitToStreamer(MCStreamer &S, const MCInst &Inst);

  // For Zvmagic
  MCCodeEmitter *MagicEmitter;
};
}

#define GEN_COMPRESS_INSTR
#include "RISCVGenCompressInstEmitter.inc"
void RISCVAsmPrinter::emitToStreamer(MCStreamer &S, const MCInst &Inst) {
  MCInst CInst;
  bool Res = compressInst(CInst, Inst, *STI, OutStreamer->getContext());
  if (Res)
    ++RISCVNumInstrsCompressed;
  AsmPrinter::EmitToStreamer(*OutStreamer, Res ? CInst : Inst);
}

#define GEN_VECTOR_MAGIC
#include "RISCVGenVectorMagicEmitter.inc"
void RISCVAsmPrinter::EmitToStreamer(MCStreamer &S, const MCInst &Inst) {
  unsigned Rs2, Rs1, Rd;
  if (!MagicEmitter || !matchVectorMagic(Inst, Rs2, Rs1, Rd)) {
    emitToStreamer(S, Inst);
    return;
  }

  std::string Str;
  raw_string_ostream TmpOS{ Str };
  SmallVector<MCFixup> Fixups;
  MagicEmitter->encodeInstruction(Inst, TmpOS, Fixups, *STI);
  assert(Str.size() == 4 && "MCInst size expected to be 32 bits");

  emitToStreamer(S, MCInstBuilder(RISCV::SD)
                    .addReg(Rs2)
                    .addReg(RISCV::X0)
                    .addImm((Str[0] << 3) - 0x800));
  emitToStreamer(S, MCInstBuilder(RISCV::SD)
                    .addReg(Rs1)
                    .addReg(RISCV::X0)
                    .addImm((Str[1] << 3) - 0x800));
  emitToStreamer(S, MCInstBuilder(RISCV::SD)
                    .addReg(Rd)
                    .addReg(RISCV::X0)
                    .addImm((Str[2] << 3) - 0x800));
  emitToStreamer(S, MCInstBuilder(RISCV::LD)
                    .addReg(Rd)
                    .addReg(RISCV::X0)
                    .addImm((Str[3] << 3) - 0x800));
}

// Simple pseudo-instructions have their lowering (with expansion to real
// instructions) auto-generated.
#include "RISCVGenMCPseudoLowering.inc"

void RISCVAsmPrinter::emitInstruction(const MachineInstr *MI) {
  // Do any auto-generated pseudo lowerings.
  if (emitPseudoExpansionLowering(*OutStreamer, MI))
    return;

  MCInst TmpInst;
  LowerRISCVMachineInstrToMCInst(MI, TmpInst, *this);
  EmitToStreamer(*OutStreamer, TmpInst);
}

bool RISCVAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                      const char *ExtraCode, raw_ostream &OS) {
  // First try the generic code, which knows about modifiers like 'c' and 'n'.
  if (!AsmPrinter::PrintAsmOperand(MI, OpNo, ExtraCode, OS))
    return false;

  const MachineOperand &MO = MI->getOperand(OpNo);
  if (ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0)
      return true; // Unknown modifier.

    switch (ExtraCode[0]) {
    default:
      return true; // Unknown modifier.
    case 'z':      // Print zero register if zero, regular printing otherwise.
      if (MO.isImm() && MO.getImm() == 0) {
        OS << RISCVInstPrinter::getRegisterName(RISCV::X0);
        return false;
      }
      break;
    case 'i': // Literal 'i' if operand is not a register.
      if (!MO.isReg())
        OS << 'i';
      return false;
    }
  }

  switch (MO.getType()) {
  case MachineOperand::MO_Immediate:
    OS << MO.getImm();
    return false;
  case MachineOperand::MO_Register:
    OS << RISCVInstPrinter::getRegisterName(MO.getReg());
    return false;
  case MachineOperand::MO_GlobalAddress:
    PrintSymbolOperand(MO, OS);
    return false;
  case MachineOperand::MO_BlockAddress: {
    MCSymbol *Sym = GetBlockAddressSymbol(MO.getBlockAddress());
    Sym->print(OS, MAI);
    return false;
  }
  default:
    break;
  }

  return true;
}

bool RISCVAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                            unsigned OpNo,
                                            const char *ExtraCode,
                                            raw_ostream &OS) {
  if (!ExtraCode) {
    const MachineOperand &MO = MI->getOperand(OpNo);
    // For now, we only support register memory operands in registers and
    // assume there is no addend
    if (!MO.isReg())
      return true;

    OS << "0(" << RISCVInstPrinter::getRegisterName(MO.getReg()) << ")";
    return false;
  }

  return AsmPrinter::PrintAsmMemoryOperand(MI, OpNo, ExtraCode, OS);
}

bool RISCVAsmPrinter::runOnMachineFunction(MachineFunction &MF) {
  // Set the current MCSubtargetInfo to a copy which has the correct
  // feature bits for the current MachineFunction
  MCSubtargetInfo &NewSTI =
    OutStreamer->getContext().getSubtargetCopy(*TM.getMCSubtargetInfo());
  NewSTI.setFeatureBits(MF.getSubtarget().getFeatureBits());
  STI = &NewSTI;

  SetupMachineFunction(MF);
  emitFunctionBody();
  return false;
}

void RISCVAsmPrinter::emitStartOfAsmFile(Module &M) {
  if (TM.getTargetTriple().isOSBinFormatELF())
    emitAttributes();
}

void RISCVAsmPrinter::emitEndOfAsmFile(Module &M) {
  RISCVTargetStreamer &RTS =
      static_cast<RISCVTargetStreamer &>(*OutStreamer->getTargetStreamer());

  if (TM.getTargetTriple().isOSBinFormatELF())
    RTS.finishAttributeSection();
}

void RISCVAsmPrinter::emitAttributes() {
  RISCVTargetStreamer &RTS =
      static_cast<RISCVTargetStreamer &>(*OutStreamer->getTargetStreamer());

  const Triple &TT = TM.getTargetTriple();
  StringRef CPU = TM.getTargetCPU();
  StringRef FS = TM.getTargetFeatureString();
  const RISCVTargetMachine &RTM = static_cast<const RISCVTargetMachine &>(TM);
  const RISCVSubtarget STI(TT, CPU, FS, /*ABIName=*/"", RTM);

  RTS.emitTargetAttributes(STI);
}

// Force static initialization.
extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeRISCVAsmPrinter() {
  RegisterAsmPrinter<RISCVAsmPrinter> X(getTheRISCV32Target());
  RegisterAsmPrinter<RISCVAsmPrinter> Y(getTheRISCV64Target());
}
