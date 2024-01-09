load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)

load("//tools/skylark:drake_data.bzl", "models_filegroup")

models_filegroup(
    name = "models",
    #  Creates a filegroup with the given name, using a Bazel glob() to find
    # files typically associated with model data (e.g., `*.urdf` or `*.obj`).

    # Models within a `test/` directory or named `test*` are skipped; this macro
    # is only intended to locate non-test models. Tests should prefer to declare
    # their dependencies without relying on globbing, either by listing the data
    # files directly, or by writing out a specific filegroup() with the details.

    # Use `extra_srcs` to add more files beyond what's matched by the glob.

    # Use `glob_exclude` to exclude patterns that would otherwise be found.
    
)

drake_cc_binary(
    name = "sim",
    srcs = ["main.cc"],
    data = [":models"],
    deps = [
        # "//examples/acrobot:acrobot_geometry", #examples/acrobot/acrobot_geometry.h
        "//multibody/plant",
        "//multibody/parsing",
        "//systems/analysis:simulator",
        "//systems/framework:diagram",
        "//systems/framework:leaf_system",
        "//systems/framework:diagram_builder",
        "//visualization:visualization_config_functions",
        "@gflags",
        # ":aerodynamics_system",
        # ... other necessary deps
    ],
)
