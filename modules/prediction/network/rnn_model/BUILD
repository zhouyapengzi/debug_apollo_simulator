load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rnn_model",
    srcs = [
        "rnn_model.cc",
    ],
    hdrs = [
        "rnn_model.h",
    ],
    deps = [
        "//modules/prediction/network:net_model",
"//cyber/common:log",    
],
)

cc_test(
    name = "rnn_model_test",
    size = "small",
    srcs = [
        "rnn_model_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
    ],
    deps = [
"//cyber/common:log",
        "//modules/prediction/network/rnn_model",
        "@gtest//:main",
    ],
)

cpplint()
