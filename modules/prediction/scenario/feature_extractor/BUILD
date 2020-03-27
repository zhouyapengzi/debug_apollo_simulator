load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feature_extractor",
    srcs = ["feature_extractor.cc"],
    hdrs = ["feature_extractor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container:container_manager",
        "//cyber/common:log",
    ],
)

cc_test(
    name = "feature_extractor_test",
    size = "small",
    srcs = ["feature_extractor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":feature_extractor",
        "//modules/prediction/common:kml_map_based_test",
        "//cyber/common:log",
    ],
)

cpplint()
