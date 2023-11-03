load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "affine_constraint",
    srcs = ["affine_constraint.cc"],
    hdrs = ["affine_constraint.h"],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_seg",
    srcs = ["spline_1d_seg.cc"],
    hdrs = ["spline_1d_seg.h"],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d",
    srcs = ["spline_1d.cc"],
    hdrs = ["spline_1d.h"],
    deps = [
        ":affine_constraint",
        ":spline_1d_seg",
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_constraint",
    srcs = ["spline_1d_constraint.cc"],
    hdrs = ["spline_1d_constraint.h"],
    deps = [
        ":affine_constraint",
        ":spline_1d",
        "//modules/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "spline_seg_kernel",
    srcs = ["spline_seg_kernel.cc"],
    hdrs = ["spline_seg_kernel.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common:macros",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_kernel",
    srcs = ["spline_1d_kernel.cc"],
    hdrs = ["spline_1d_kernel.h"],
    deps = [
        ":affine_constraint",
        ":spline_1d",
        ":spline_seg_kernel",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_solver",
    srcs = ["spline_1d_solver.cc"],
    hdrs = ["spline_1d_solver.h"],
    deps = [
        ":spline_1d",
        ":spline_1d_constraint",
        ":spline_1d_kernel",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:qp_problem_proto",
        "@eigen",
    ],
)

cc_library(
    name = "active_set_spline_1d_solver",
    srcs = ["active_set_spline_1d_solver.cc"],
    hdrs = ["active_set_spline_1d_solver.h"],
    deps = [
        ":spline_1d_solver",
        "//modules/common/time",
        "@eigen",
    ],
)


cc_library(
    name = "osqp_spline_1d_solver",
    srcs = ["osqp_spline_1d_solver.cc"],
    hdrs = ["osqp_spline_1d_solver.h"],
    deps = [
        ":spline_1d_solver",
        "//modules/common/math:matrix_operations",
        "//modules/common/time",
        "@eigen",
        "@osqp",
    ],
)


cc_library(
    name = "spline_2d_seg",
    srcs = ["spline_2d_seg.cc"],
    hdrs = ["spline_2d_seg.h"],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d",
    srcs = ["spline_2d.cc"],
    hdrs = ["spline_2d.h"],
    deps = [
        ":spline_2d_seg",
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_constraint",
    srcs = ["spline_2d_constraint.cc"],
    hdrs = ["spline_2d_constraint.h"],
    deps = [
        ":affine_constraint",
        ":spline_2d",
        "//modules/common/math:angle",
        "//modules/common/math:geometry",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_kernel",
    srcs = ["spline_2d_kernel.cc"],
    hdrs = ["spline_2d_kernel.h"],
    deps = [
        ":spline_2d",
        ":spline_seg_kernel",
        "//modules/common/math:geometry",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_solver",
    hdrs = [
        "active_set_spline_2d_solver.h",
        "osqp_spline_2d_solver.h",
        "spline_2d_solver.h",
    ],
    deps = [
        ":spline_2d",
        ":spline_2d_constraint",
        ":spline_2d_kernel",
        "//modules/common/math:geometry",
        "//modules/common/math:matrix_operations",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "//modules/common/time",
        "//modules/planning/common:planning_gflags",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "active_set_spline_2d_solver",
    srcs = ["active_set_spline_2d_solver.cc"],
    hdrs = ["active_set_spline_2d_solver.h"],
    deps = [
        ":spline_2d_solver",
    ],
)

cc_library(
    name = "osqp_spline_2d_solver",
    srcs = ["osqp_spline_2d_solver.cc"],
    hdrs = ["osqp_spline_2d_solver.h"],
    deps = [
        ":spline_2d_solver",
        "//modules/common/math:matrix_operations",
        "@osqp",
    ],
)


cc_library(
    name = "piecewise_linear_constraint",
    srcs = ["piecewise_linear_constraint.cc"],
    hdrs = ["piecewise_linear_constraint.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "piecewise_linear_kernel",
    srcs = ["piecewise_linear_kernel.cc"],
    hdrs = ["piecewise_linear_kernel.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "piecewise_linear_generator",
    srcs = ["piecewise_linear_generator.cc"],
    hdrs = ["piecewise_linear_generator.h"],
    deps = [
        ":piecewise_linear_constraint",
        ":piecewise_linear_kernel",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "@eigen",
    ],
)


cpplint()
