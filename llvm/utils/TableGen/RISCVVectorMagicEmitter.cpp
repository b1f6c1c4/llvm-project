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

namespace llvm {

void EmitVectorMagic(RecordKeeper &RK, raw_ostream &OS) {
  std::vector<Record *> Rs = RK.getAllDerivedDefinitions("RVInstSetVLi");

  // Emit file header.
  emitSourceFileHeader("Vector Magic Emitter Source Fragment", OS);

  OS << "#ifdef GEN_VECTOR_MAGIC\n";
  OS << "#undef GEN_VECTOR_MAGIC\n\n";
  OS << "bool matchVectorMagic(const MCInst &Inst, unsigned &Rs2, unsigned &Rs1, unsigned &Rd) {\n";
  OS << "  return false;\n"; // TODO
  OS << "}\n\n";
  OS << "#endif //GEN_VECTOR_MAGIC\n\n";
}

} // namespace llvm
