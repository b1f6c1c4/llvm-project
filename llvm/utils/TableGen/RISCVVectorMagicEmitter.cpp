//===- RISCVVectorMagicEmitter.cpp - Generator for RISCV Vector Magic -===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// RISCVVectorMagicEmitter implements a tablegen-driven mechanism that
// translates RISCV Vector instruction into four LD/SD.
//
//===--------------------------------------------------------------===//

#include "llvm/TableGen/Record.h"
#include "llvm/TableGen/TableGenBackend.h"
#include <vector>

#define DEBUG_TYPE "vector-magic-emitter"

using namespace llvm;

namespace {
class RISCVVectorMagicEmitter {
private:
  RecordKeeper &Records;

  void genOperand(raw_ostream &OS, const std::string &Str, int V);
  void genCase(raw_ostream &OS, int Rs2, int Rs1, int Rd, bool Suffix = false);
  void genLabel(raw_ostream &OS, const std::string &Class);

public:
  RISCVVectorMagicEmitter(RecordKeeper &RK) : Records(RK) {}

  void run(raw_ostream &OS);
};
} // anonymous namespace.

void RISCVVectorMagicEmitter::genOperand(raw_ostream &OS, const std::string &Str, int V) {
  switch (V) {
    case -1:
      OS << "      " << Str << " = RISCV::X0;\n";
      break;
    default:
      OS << "      assert(Inst.getOperand(" << V << ").isReg() && \"operand is not a register\");\n";
      OS << "      " << Str << " = Inst.getOperand(" << V << ").getReg();\n";
      break;
  }
}

void RISCVVectorMagicEmitter::genCase(raw_ostream &OS, int Rs2, int Rs1, int Rd, bool Suffix) {
  OS << " {\n";
  genOperand(OS, "Rs2", Rs2);
  genOperand(OS, "Rs1", Rs1);
  genOperand(OS, "Rd", Rd);
  if (!Suffix)
    OS << "      return true;\n";
  else
    OS << "      break;\n";
  OS << "    }";
}

void RISCVVectorMagicEmitter::genLabel(raw_ostream &OS, const std::string &Class) {
  for (auto *R : Records.getAllDerivedDefinitions(Class))
    OS << "\n    case RISCV::" << R->getNameInitAsString() << ":";
}

void RISCVVectorMagicEmitter::run(raw_ostream &OS) {
  emitSourceFileHeader("Vector Magic Emitter Source Fragment", OS);

  OS << "#ifdef GEN_VECTOR_MAGIC\n";
  OS << "#undef GEN_VECTOR_MAGIC\n\n";
  OS << "bool matchVectorMagic(const MCInst &Inst, unsigned &Rs2, unsigned &Rs1, unsigned &Rd) {\n";
  OS << "  switch (Inst.getOpcode()) {";

  genLabel(OS, "RVInstSetVLi"), genCase(OS, -1, 1, 0);
  genLabel(OS, "RVInstSetVL"), genCase(OS, 2, 1, 0);
  genLabel(OS, "RVInstVV"), genCase(OS, -1, -1, -1);
  genLabel(OS, "RVInstVX"), genCase(OS, -1, 2, -1);
  genLabel(OS, "RVInstV2"), genCase(OS, -1, 1, -1);
  genLabel(OS, "RVInstIVI"), genCase(OS, -1, -1, -1);
  genLabel(OS, "RVInstV"), genCase(OS, -1, -1, -1);
  genLabel(OS, "RVInstVLU"), genCase(OS, -1, 1, -1);
  genLabel(OS, "RVInstVLS"), genCase(OS, 2, 1, -1);
  genLabel(OS, "RVInstVLX"), genCase(OS, -1, 1, -1);
  genLabel(OS, "RVInstVSU"), genCase(OS, -1, 1, -1);
  genLabel(OS, "RVInstVSS"), genCase(OS, 2, 1, -1);
  genLabel(OS, "RVInstVSX"), genCase(OS, -1, 1, -1);
  genLabel(OS, "RVInstVAMO"), genCase(OS, -1, 0, -1);
  genLabel(OS, "CSR_ir"), genCase(OS, -1, 2, 0, true);
  genLabel(OS, "CSR_ii"), genCase(OS, -1, -1, 0, true);

  OS << "\n    default:\n";
  OS << "      return false;\n";

  OS << "  }\n";

  OS << "  assert(Inst.getOperand(1).isImm() && \"operand is not a immediate\");\n";
  OS << "  switch (Inst.getOperand(1).getImm()) {\n";
  OS << "    case 0x008:\n";
  OS << "    case 0x009:\n";
  OS << "    case 0x00a:\n";
  OS << "    case 0x00f:\n";
  OS << "    case 0xc20:\n";
  OS << "    case 0xc21:\n";
  OS << "    case 0xc22:\n";
  OS << "      return true;\n";
  OS << "  }\n";
  OS << "  return false;\n";
  OS << "}\n\n";

  OS << "#endif //GEN_VECTOR_MAGIC\n\n";
}

namespace llvm {

void EmitVectorMagic(RecordKeeper &RK, raw_ostream &OS) {
  RISCVVectorMagicEmitter(RK).run(OS);
}

} // namespace llvm
