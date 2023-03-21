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
#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSet.h"
#include "mlir/Dialect/Func/IR/FuncOps.h"  // from @llvm-project
#include "mlir/IR/Builders.h"  // from @llvm-project
#include "mlir/IR/OperationSupport.h"  // from @llvm-project
#include "mlir/Pass/Pass.h"  // from @llvm-project
#include "tensorflow/compiler/mlir/quantization/tensorflow/passes/passes.h"
#include "tensorflow/compiler/mlir/tensorflow/ir/tf_ops.h"
#include "tensorflow/compiler/mlir/tensorflow/translate/import_model.h"
#include "tensorflow/core/platform/macros.h"

namespace mlir {
namespace quant {
namespace {

using ::tensorflow::kImportModelDefaultGraphFuncName;

constexpr char kEntryFunctionAttr[] = "tf.entry_function";
constexpr char kExportedNameAttr[] = "tf_saved_model.exported_names";
constexpr char kIndexPathAttr[] = "tf_saved_model.index_path";

// The ConvertMlirToGraphdef requires the provided input module to have a main
// function, which might not exist in case of multi-signature graphs. In that
// case, this pass will create a new main function, which calls signature
// functions.
class InsertMainFunctionPass
    : public PassWrapper<InsertMainFunctionPass, OperationPass<ModuleOp>> {
 public:
  MLIR_DEFINE_EXPLICIT_INTERNAL_INLINE_TYPE_ID(InsertMainFunctionPass)

  explicit InsertMainFunctionPass() = default;

  StringRef getArgument() const override { return "quant-add-main-function"; }

  StringRef getDescription() const override {
    return "Insert the main function to the module if it is missing.";
  }

  void runOnOperation() override;
};

// Checks if the module has a main function.
bool HasMainFunction(ModuleOp& module) {
  StringAttr main_func_id =
      StringAttr::get(module.getContext(), kImportModelDefaultGraphFuncName);
  for (auto function : module.getOps<func::FuncOp>()) {
    if (function.getName() == main_func_id) return true;
  }
  return false;
}

// Checks if a FuncOp is exported.
bool IsExported(func::FuncOp op) {
  auto exported_names = op->getAttrOfType<ArrayAttr>(kExportedNameAttr);
  return exported_names && !exported_names.empty();
}

// Check if a function is an entry function.
bool IsEntryFunction(func::FuncOp op) {
  return op->hasAttr(kEntryFunctionAttr);
}

// Returns true iff the provided FuncOp is qualified to be included in the main
// function.
bool ShouldIncludeInMainFunction(func::FuncOp func_op) {
  return !func_op.isPrivate() && IsExported(func_op) &&
         IsEntryFunction(func_op);
}

// Sets a function to be private so it can be referred internally.
void SetFunctionPrivate(func::FuncOp& func) {
  func.setVisibility(SymbolTable::Visibility::Private);

  // The `tf_saved_model` attributes can only be appied to public functions.
  for (auto& attr : func->getAttrs()) {
    StringRef attr_name = attr.getName().getValue();
    if (attr_name.startswith("tf_saved_model.")) {
      func->removeAttr(attr_name);
    }
  }

  for (int i = 0; i < func.getNumArguments(); ++i) {
    for (auto& attr : func.getArgAttrs(i)) {
      const StringAttr& attr_name = attr.getName();
      if (attr_name.getValue().startswith("tf_saved_model.")) {
        func.removeArgAttr(i, attr_name);
      }
    }
  }
  for (int i = 0; i < func.getNumResults(); ++i) {
    for (auto& attr : func.getResultAttrs(i)) {
      const StringAttr& attr_name = attr.getName();
      if (attr_name.getValue().startswith("tf_saved_model.")) {
        func.removeResultAttr(i, attr_name);
      }
    }
  }
}

// Information to identify an output in its node and in the model output list.
// Ex: If the model output list is ["add:0", "topk:0": "topk:1"], then the
// output corresponding to "topk:1" will have output_index=2 and tensor_index=1.
struct OutputInfo {
  // The index of this output in the model output list.
  int32_t output_index;
  // The index of this output in its node.
  int32_t tensor_index;
  // The output value.
  Value value;
};

// Makes input/output names across entry functions unique if necessary. If a
// dupliated name is found, this function will add signature prefix for all the
// input/output names.
void GetUniqueInputOutputNodeNames(ModuleOp& module,
                                   std::vector<std::string>& input_name_vec,
                                   std::vector<std::string>& output_name_vec) {
  bool need_prefix_for_input_name = false;
  bool need_prefix_for_output_name = false;
  std::vector<StringRef> fn_input_name_vec, fn_output_name_vec;
  llvm::StringSet<> input_name_set, output_name_set;
  for (auto function : module.getOps<func::FuncOp>()) {
    if (!ShouldIncludeInMainFunction(function)) continue;
    if (auto tf_attrs =
            function->getAttrOfType<DictionaryAttr>(kEntryFunctionAttr)) {
      StringRef function_name = function.getSymName();

      if (auto inputs_attr = tf_attrs.get("inputs")) {
        std::string inputs_attr_str =
            inputs_attr.cast<StringAttr>().getValue().str();
        std::vector<std::string> fn_input_names =
            absl::StrSplit(inputs_attr_str, ',', absl::SkipEmpty());

        for (StringRef input_name : fn_input_names) {
          if (input_name_set.count(input_name) > 0) {
            // Found a duplicated name, all input names will be prefixed by
            // their corresponding function names.
            need_prefix_for_input_name = true;
          }
          input_name_set.insert(input_name);
          fn_input_name_vec.push_back(function_name);
        }
        input_name_vec.insert(input_name_vec.end(),
                              std::make_move_iterator(fn_input_names.begin()),
                              std::make_move_iterator(fn_input_names.end()));
      }

      if (auto outputs_attr = tf_attrs.get("outputs")) {
        std::string outputs_attr_str =
            outputs_attr.cast<StringAttr>().getValue().str();
        std::vector<std::string> fn_output_names =
            absl::StrSplit(outputs_attr_str, ',', absl::SkipEmpty());

        for (StringRef output_name : fn_output_names) {
          if (output_name_set.count(output_name) > 0) {
            // Found a duplicated name, all output names will be prefixed by
            // their corresponding function names.
            need_prefix_for_output_name = true;
          }
          output_name_set.insert(output_name);
          fn_output_name_vec.push_back(function_name);
        }
        output_name_vec.insert(output_name_vec.end(),
                               std::make_move_iterator(fn_output_names.begin()),
                               std::make_move_iterator(fn_output_names.end()));
      }
    }
  }

  if (need_prefix_for_input_name) {
    absl::c_transform(input_name_vec, fn_input_name_vec, input_name_vec.begin(),
                      [](const std::string& input_name, StringRef fn_name) {
                        std::string new_name = fn_name.str();
                        absl::StrAppend(&new_name, "_", input_name);
                        return new_name;
                      });
  }
  if (need_prefix_for_output_name) {
    absl::c_transform(output_name_vec, fn_output_name_vec,
                      output_name_vec.begin(),
                      [](const std::string& output_name, StringRef fn_name) {
                        std::string new_name = fn_name.str();
                        absl::StrAppend(&new_name, "_", output_name);
                        return new_name;
                      });
  }
}

// Creates a main function which calls other exported functions.
bool CreateMainFunction(ModuleOp& module) {
  MLIRContext* context = module.getContext();
  OpBuilder builder(context);

  std::vector<std::string> input_names, output_names;
  GetUniqueInputOutputNodeNames(module, input_names, output_names);

  // Collects argument and result types.
  llvm::SmallVector<Location> arg_locs;
  llvm::SmallVector<Type> arg_types, result_types;

  for (auto function : module.getOps<func::FuncOp>()) {
    if (!ShouldIncludeInMainFunction(function)) continue;

    arg_types.append(function.getArgumentTypes().begin(),
                     function.getArgumentTypes().end());
    auto& return_op = function.getBody().getBlocks().front().back();
    result_types.append(return_op.getOperandTypes().begin(),
                        return_op.getOperandTypes().end());
    for (const auto& arg : function.getArguments()) {
      arg_locs.push_back(arg.getLoc());
    }
  }

  // Creates a new main function.
  auto func_type = FunctionType::get(context, arg_types, result_types);
  auto main_func = builder.create<func::FuncOp>(
      module.getLoc(), kImportModelDefaultGraphFuncName, func_type);
  builder.createBlock(&main_func.getBody(), main_func.begin(), arg_types,
                      arg_locs);
  SmallVector<NamedAttribute> func_attrs;
  func_attrs.push_back(
      {StringAttr::get(context, "inputs"),
       StringAttr::get(context, absl::StrJoin(input_names, ","))});
  func_attrs.push_back(
      {StringAttr::get(context, "outputs"),
       StringAttr::get(context, absl::StrJoin(output_names, ","))});
  auto dictAttr = DictionaryAttr::get(context, func_attrs);
  main_func->setAttr(StringAttr::get(context, kEntryFunctionAttr), dictAttr);
  main_func->setAttr(
      kExportedNameAttr,
      builder.getStrArrayAttr({kImportModelDefaultGraphFuncName}));

  if (input_names.size() != main_func.getNumArguments() ||
      output_names.size() != main_func.getNumResults()) {
    module.emitError()
        << "Number of inputs and outputs in the tf.entry_function attribute "
           "mismatched. [Input] Expected: "
        << input_names.size() << ", got: " << main_func.getNumArguments()
        << ". [Output] Expected: " << output_names.size()
        << ", got: " << main_func.getNumResults();
    return false;
  }

  const int num_args = main_func.getNumArguments();
  for (int i = 0; i < num_args; ++i) {
    main_func.setArgAttr(
        i, kIndexPathAttr,
        ArrayAttr::get(context, {StringAttr::get(context, input_names[i])}));
  }

  const int num_results = main_func.getNumResults();
  for (int i = 0; i < num_results; ++i) {
    main_func.setResultAttr(
        i, kIndexPathAttr,
        ArrayAttr::get(context, {StringAttr::get(context, output_names[i])}));
  }

  // Creates PartitionedCall ops to call exported functions.
  auto guard = OpBuilder::InsertionGuard(builder);
  int arg_idx = 0;
  int result_idx = 0;
  llvm::SmallVector<Value> call_op_returns;
  for (auto function : module.getOps<func::FuncOp>()) {
    if (!ShouldIncludeInMainFunction(function)) continue;

    llvm::ArrayRef<BlockArgument> new_args = llvm::makeArrayRef(
        main_func.getArguments().begin() + arg_idx, function.getNumArguments());
    arg_idx += function.getNumArguments();
    llvm::ArrayRef<Type> new_types = llvm::makeArrayRef(
        result_types.begin() + result_idx, function.getNumResults());
    result_idx += function.getNumResults();

    auto call_op = builder.create<TF::PartitionedCallOp>(
        module.getLoc(), new_types, new_args,
        SymbolRefAttr::get(context, function.getSymName()),
        /*config=*/builder.getStringAttr(""),
        /*config_proto=*/builder.getStringAttr(""),
        /*executor_type=*/builder.getStringAttr(""));
    call_op_returns.append(call_op.getResults().begin(),
                           call_op.getResults().end());
    SetFunctionPrivate(function);
  }

  // Creates Identity/IdentityN ops for returing values. This allows us to
  // restore the same output tensor names in python.
  int32_t output_count = 0;
  // Map from node name to the list of the OutputInfos of its outputs that are
  // used as the model outputs.
  llvm::StringMap<llvm::SmallVector<OutputInfo>> node_to_output_map;
  for (auto tensor_name_value_pair : llvm::zip(output_names, call_op_returns)) {
    std::vector<std::string> name_and_index = absl::StrSplit(
        std::get<0>(tensor_name_value_pair), ':', absl::SkipEmpty());
    llvm::StringRef node_name = name_and_index.front();
    int32_t tensor_index = 0;
    if (name_and_index.size() > 1) {
      tensor_index = std::stoi(name_and_index.back());
    }
    node_to_output_map[node_name].push_back(
        {output_count++, tensor_index, std::get<1>(tensor_name_value_pair)});
  }

  Value scalar_one =
      CreateScalarConstValue<float>(builder, builder.getUnknownLoc(), 1.0);
  llvm::SmallVector<Value> returning_values(output_count, Value());
  for (const auto& node_name : node_to_output_map.keys()) {
    auto node_output_tensors = node_to_output_map[node_name];

    NameLoc new_loc = NameLoc::get(builder.getStringAttr(node_name));
    int32_t max_tensor_index = 0;
    absl::c_for_each(node_output_tensors,
                     [&max_tensor_index](const OutputInfo& output_info) {
                       max_tensor_index =
                           std::max(max_tensor_index, output_info.tensor_index);
                     });

    // Create IdentityOp or IdentityNOp based on the number of outputs.
    Operation* identity_op;
    if (max_tensor_index == 0) {
      Value output_value = node_output_tensors.front().value;
      identity_op = builder.create<TF::IdentityOp>(
          new_loc, output_value.getType(), output_value);
    } else {
      llvm::SmallVector<Value> input_values(node_output_tensors.size(),
                                            scalar_one);
      for (const auto& [output_index, tensor_index, tensor_value] :
           node_output_tensors) {
        input_values[tensor_index] = tensor_value;
      }
      identity_op = builder.create<TF::IdentityNOp>(
          new_loc, TypeRange(ValueRange(input_values)), input_values);
    }

    for (const auto& [output_index, tensor_index, tensor_value] :
         node_output_tensors) {
      returning_values[output_index] = identity_op->getResult(tensor_index);
    }
  }
  builder.create<func::ReturnOp>(main_func.getBody().getLoc(),
                                 returning_values);

  // Adds the new function to symbol table.
  SymbolTable symbol_table(module);
  symbol_table.insert(main_func);
  return true;
}

void InsertMainFunctionPass::runOnOperation() {
  ModuleOp module = getOperation();
  if (!HasMainFunction(module)) {
    if (!CreateMainFunction(module)) {
      signalPassFailure();
    }
  }
}

}  // namespace

std::unique_ptr<OperationPass<ModuleOp>> CreateInsertMainFunctionPass() {
  return std::make_unique<InsertMainFunctionPass>();
}

static PassRegistration<InsertMainFunctionPass> pass([] {
  return CreateInsertMainFunctionPass();
});

}  // namespace quant
}  // namespace mlir
