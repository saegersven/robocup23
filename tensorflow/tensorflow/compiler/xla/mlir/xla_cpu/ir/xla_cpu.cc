/* Copyright 2022 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "tensorflow/compiler/xla/mlir/xla_cpu/ir/xla_cpu.h"

#include "llvm/ADT/TypeSwitch.h"
#include "mlir/Dialect/Bufferization/IR/BufferizableOpInterface.h"  // from @llvm-project
#include "mlir/IR/Builders.h"  // from @llvm-project
#include "mlir/IR/BuiltinTypes.h"  // from @llvm-project
#include "mlir/IR/PatternMatch.h"  // from @llvm-project
#include "tensorflow/compiler/xla/mlir/xla_cpu/ir/xla_cpu_dialect.cc.inc"
#include "tensorflow/compiler/xla/mlir/xla_cpu/ir/xla_cpu_enums.cc.inc"
#define GET_ATTRDEF_CLASSES
#include "tensorflow/compiler/xla/mlir/xla_cpu/ir/xla_cpu_attrdefs.cc.inc"

namespace mlir {
namespace xla_cpu {

void XlaCpuDialect::initialize() {
  addOperations<
#define GET_OP_LIST
#include "tensorflow/compiler/xla/mlir/xla_cpu/ir/xla_cpu.cc.inc"
#undef GET_OP_LIST
      >();
}

template <typename Op>
LogicalResult BufferizeOp(Op op, RewriterBase &rewriter,
                          const bufferization::BufferizationOptions &options,
                          int64_t num_inputs) {
  if (op.getOperands().front().getType().template isa<MemRefType>()) {
    return success();
  }
  SmallVector<Value> new_operands;
  for (auto operand : op.getOperands()) {
    FailureOr<Value> maybe_buffer = getBuffer(rewriter, operand, options);
    if (failed(maybe_buffer)) {
      return failure();
    }
    new_operands.push_back(*maybe_buffer);
  }
  rewriter.create<Op>(op.getLoc(), TypeRange{}, new_operands,
                      op.getOperation()->getAttrs());
  bufferization::replaceOpWithBufferizedValues(
      rewriter, op.getOperation(),
      llvm::makeArrayRef(new_operands).drop_front(num_inputs));
  return success();
}

bool AllReduceOp::bufferizesToMemoryRead(OpOperand &opOperand,
                                         const bufferization::AnalysisState &) {
  return opOperand.getOperandNumber() < getNumOperands() / 2;
}

bool AllReduceOp::bufferizesToMemoryWrite(
    OpOperand &opOperand, const bufferization::AnalysisState &state) {
  return !bufferizesToMemoryRead(opOperand, state);
}

SmallVector<OpResult> AllReduceOp::getAliasingOpResult(
    OpOperand &opOperand, const bufferization::AnalysisState &) {
  if (opOperand.getOperandNumber() < getNumOperands() / 2) {
    return {};
  }
  return {getOperation()->getOpResult(opOperand.getOperandNumber() -
                                      getNumOperands() / 2)};
}

LogicalResult AllReduceOp::bufferize(
    RewriterBase &rewriter,
    const bufferization::BufferizationOptions &options) {
  return BufferizeOp(*this, rewriter, options, this->getNumOperands() / 2);
}

bufferization::BufferRelation AllReduceOp::bufferRelation(
    OpResult, const bufferization::AnalysisState &) {
  return bufferization::BufferRelation::Equivalent;
}

LogicalResult CollectivePermuteOp::bufferize(
    RewriterBase &rewriter,
    const bufferization::BufferizationOptions &options) {
  return BufferizeOp(*this, rewriter, options, this->getNumOperands() / 2);
}

LogicalResult AllToAllOp::bufferize(
    RewriterBase &rewriter,
    const bufferization::BufferizationOptions &options) {
  return BufferizeOp(*this, rewriter, options, this->getNumOperands() / 2);
}

LogicalResult FftOp::bufferize(
    RewriterBase &rewriter,
    const bufferization::BufferizationOptions &options) {
  return BufferizeOp(*this, rewriter, options, this->getNumOperands() / 2);
}

LogicalResult OutfeedOp::bufferize(
    RewriterBase &rewriter,
    const bufferization::BufferizationOptions &options) {
  return BufferizeOp(*this, rewriter, options, this->getNumOperands());
}

}  // namespace xla_cpu
}  // namespace mlir

#define GET_OP_CLASSES
#include "tensorflow/compiler/xla/mlir/xla_cpu/ir/xla_cpu.cc.inc"
