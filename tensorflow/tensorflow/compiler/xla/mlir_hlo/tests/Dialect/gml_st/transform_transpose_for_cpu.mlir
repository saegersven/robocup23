// RUN: mlir-hlo-opt %s --gml-st-cpu-transform-transpose | FileCheck %s

func.func @transpose_permutation(%input: tensor<16x32x64xf32>,
    %init: tensor<32x64x16xf32>) -> tensor<32x64x16xf32> {
    %transpose = linalg.transpose
      ins(%input:tensor<16x32x64xf32>)
      outs(%init:tensor<32x64x16xf32>)
      permutation = [1, 2, 0]
  func.return %transpose : tensor<32x64x16xf32>
}

// CHECK-LABEL: func.func @transpose_permutation(
// CHECK-SAME:      %[[INPUT:.*]]: tensor<16x32x64xf32>,
// CHECK-SAME:      %[[INIT:.*]]: tensor<32x64x16xf32>)

// CHECK:             gml_st.parallel
// CHECK:               %[[INPUT_SUB:.*]] = gml_st.materialize %[[INPUT]]
// CHECK:      : tensor<16x32x64xf32>[!gml_st.tile<8x1x8>] to tensor<8x1x8xf32>

// CHECK:              %[[INIT_SUB:.*]] =  gml_st.materialize %[[INIT]]
// CHECK:      : tensor<32x64x16xf32>[!gml_st.tile<1x8x8>] to tensor<1x8x8xf32>

// CHECK:       linalg.transpose
// CHECK-NEXT:    ins(%[[INPUT_SUB]] : tensor<8x1x8xf32>)
// CHECK-NEXT:    outs(%[[INIT_SUB]] : tensor<1x8x8xf32>)
// CHECK-NEXT:    permutation = [1, 2, 0]