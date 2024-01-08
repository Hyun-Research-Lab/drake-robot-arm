load(
    "//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)

# drake_cc_binary(
#     name = 
# )
# load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

drake_cc_library(
    name = "multibody_plant",
    srcs = ["multibody_plant.cc"],
    hdrs = ["multibody_plant.h"],
    deps = [
        "@drake//multibody",
        # ... other necessary deps
    ],
)

drake_cc_library(
    name = "aerodynamics_system",
    srcs = ["aerodynamics_system.cc"],
    hdrs = ["aerodynamics_system.h"],
    deps = [
        "@drake//systems",
        # ... other necessary deps
    ],
)

drake_cc_binary(
    name = "my_simulation",
    srcs = ["main.cc"],
    deps = [
        ":multibody_plant",
        ":aerodynamics_system",
        # ... other necessary deps
    ],
)
