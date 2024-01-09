load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)

drake_cc_binary(
    name = "sim",
    srcs = ["main.cc"],
    deps = [
        "//examples/acrobot:acrobot_geometry", #examples/acrobot/acrobot_geometry.h
        "@gflags"
        # "//multibody:multibody_plant",
        # ":aerodynamics_system",
        # ... other necessary deps
    ],
)
