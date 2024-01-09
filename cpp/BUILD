# # This file is named BUILD.bazel instead of the more typical BUILD, so that on
# # OSX it won't conflict with a build artifacts directory named "build".

# load("@drake//tools/skylark:py.bzl", "py_library")
# load("@drake//tools/install:install.bzl", "install", "install_test")
# load("@drake//tools/lint:lint.bzl", "add_lint_tests")

# package(
#     default_visibility = ["@drake//visibility:public"],
# )

# exports_files([
#     "CPPLINT.cfg",
#     ".bazelproject",
#     ".clang-format",
#     ".drake-find_resource-sentinel",
#     "package.xml",
# ])

# # Drake's top-level module; all drake_py_stuff rules add this to deps.
# # (We use py_library here because drake_py_library would be circular.)
# # This file should NOT be installed (see commits in __init__.py).
# py_library(
#     name = "module_py",
#     srcs = ["__init__.py"],
# )

# # Expose shared library for (a) installed binaries, (b) Drake Python bindings,
# # and (c) downstream C++ libraries which will also provide Python bindings.
# alias(
#     name = "drake_shared_library",
#     actual = "@drake//tools/install/libdrake:drake_shared_library",
#     visibility = ["@drake//visibility:public"],
# )

# # A manually-curated collection of most model files in Drake, so that we can
# # easily provide access to them for tools like @drake//tools:model_visualizer.
# filegroup(
#     name = "all_models",
#     data = [
#         "@drake//bindings/pydrake/multibody:models",
#         "@drake//examples/acrobot:models",
#         "@drake//examples/atlas:models",
#         "@drake//examples/hardware_sim:demo_data",
#         "@drake//examples/hydroelastic/ball_plate:floor_files",
#         "@drake//examples/hydroelastic/ball_plate:plate_files",
#         "@drake//examples/hydroelastic/python_ball_paddle:ball_paddle_files",
#         "@drake//examples/hydroelastic/python_nonconvex_mesh:models",
#         "@drake//examples/hydroelastic/spatula_slip_control:models",
#         "@drake//examples/kuka_iiwa_arm/models",
#         "@drake//examples/manipulation_station:models",
#         "@drake//examples/multibody/cart_pole:models",
#         "@drake//examples/multibody/four_bar:models",
#         "@drake//examples/pendulum:models",
#         "@drake//examples/planar_gripper:models",
#         "@drake//examples/pr2:models",
#         "@drake//examples/quadrotor:models",
#         "@drake//examples/scene_graph:models",
#         "@drake//examples/simple_gripper:simple_gripper_models",
#         "@drake//manipulation/models/allegro_hand_description:models",
#         "@drake//manipulation/models/franka_description:models",
#         "@drake//manipulation/models/iiwa_description:models",
#         "@drake//manipulation/models/jaco_description:models",
#         "@drake//manipulation/models/realsense2_description:models",
#         "@drake//manipulation/models/tri_homecart:models",
#         "@drake//manipulation/models/ur3e:models",
#         "@drake//manipulation/models/wsg_50_description:models",
#         "@drake//manipulation/models/ycb:models",
#         "@drake//multibody/benchmarks/acrobot:models",
#     ],
#     visibility = ["@drake//:__subpackages__"],
# )

# # A manually-curated collection of some test model files in Drake, for use by
# # @drake//tools:model_visualizer_private.
# filegroup(
#     name = "some_test_models",
#     testonly = True,
#     data = [
#         # It's okay to add more items to this list, as needed.
#         "@drake//geometry/render:test_models",
#         "@drake//manipulation/util:test_models",
#         "@drake//manipulation/util:test_directives",
#         "@drake//multibody/parsing:test_models",
#         "@drake//geometry:test_obj_files",
#         "@drake//geometry:test_stl_files",
#         "@drake//geometry:test_vtk_files",
#         "@drake//geometry:environment_maps",
#         "@drake//geometry/render_gltf_client:merge_resources",
#         "@drake//geometry/render_gltf_client:gltf_client_test_models",
#     ],
#     visibility = ["@drake//tools:__pkg__"],
# )

# # To create a manifest of all installed files for use by drake_bazel_installed,
# # we declare an install target that contains almost everything -- but it can't
# # contain the bazel logic that is generated based on the manifest, so we'll add
# # that in below in the final :install target.
# install(
#     name = "all_install_targets_except_bazel",
#     data = ["package.xml"],
#     docs = ["LICENSE.TXT"],
#     visibility = ["@drake//tools/install/bazel:__pkg__"],
#     deps = [
#         "@drake//bindings/pydrake:install",
#         "@drake//common:install",
#         "@drake//examples:install",
#         "@drake//geometry:install",
#         "@drake//lcmtypes:install",
#         "@drake//manipulation/models:install_data",
#         "@drake//multibody/parsing:install",
#         "@drake//setup:install",
#         "@drake//tools/install/libdrake:install",
#         "@drake//tools/workspace:install_external_packages",
#         "@drake//tutorials:install",
#     ],
# )

# _INSTALL_TEST_COMMANDS = "install_test_commands"

# install(
#     name = "install",
#     install_tests_script = _INSTALL_TEST_COMMANDS,
#     deps = [
#         ":all_install_targets_except_bazel",
#         "@drake//tools/install/bazel:install",
#     ],
# )

# install_test(
#     name = "install_test",
#     args = ["--install_tests_filename=$(location :{})".format(
#         _INSTALL_TEST_COMMANDS,
#     )],
#     data = [
#         ":install",
#         _INSTALL_TEST_COMMANDS,
#     ],
#     tags = [
#         # Running acceptance tests under coverage (kcov) can fail for presently
#         # unknown reasons when it comes to drake_visualizer, so skip them.
#         "no_kcov",
#         # Running acceptance tests under Valgrind tools is extremely slow and
#         # of limited value, so skip them.
#         "no_valgrind_tools",
#     ],
# )

# add_lint_tests(
#     bazel_lint_extra_srcs = glob(
#         [
#             "cmake/external/workspace/**/*.bazel.in",
#             "cmake/external/workspace/**/*.bzl",
#         ],
#         allow_empty = False,
#     ),
# )