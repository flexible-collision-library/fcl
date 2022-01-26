## FCL 0

### FCL 0.7.0 (2021-09-09)

* Breaking changes

  * Macros `FCL_SUPPRESS_MAYBE_UNINITIALIZED_BEGIN` and `FCL_SUPPRESS_MAYBE_UNINITIALIZED_END` defined in `fcl/common/warning.h` have been removed:
    [#489](https://github.com/flexible-collision-library/fcl/pull/489)
  * Require CMake 3.10:
    [#506](https://github.com/flexible-collision-library/fcl/pull/506)
  * Check SSE support and enable SSE if support is found:
    [#506](https://github.com/flexible-collision-library/fcl/pull/506)
    [#514](https://github.com/flexible-collision-library/fcl/pull/514)

* Core/Common

  * Use package format 3 with conditional dependency on catkin:
    [#536](https://github.com/flexible-collision-library/fcl/pull/536)
  * Fix compilation on Windows. Do not use "not" in preprocessor:
    [#530](https://github.com/flexible-collision-library/fcl/pull/530)
  * Use std::copy instead of memcpy. Patches imported from Debian:
    [#517](https://github.com/flexible-collision-library/fcl/pull/517)
  * Fix finding of ccd with pkg-config:
    [#499](https://github.com/flexible-collision-library/fcl/pull/499)
    [#497](https://github.com/flexible-collision-library/fcl/pull/497)

* Math

  * constants::eps() is now constexpr:
    [#494](https://github.com/flexible-collision-library/fcl/pull/494)
  * Fix shape conservative advancement normal computation:
    [#505](https://github.com/flexible-collision-library/fcl/pull/505)

* Geometry

  * OcTree logic for determining free/occupied:
    [#467](https://github.com/flexible-collision-library/fcl/pull/467)
  * Bugs in RSS distance queries fixed:
    [#467](https://github.com/flexible-collision-library/fcl/pull/467)
  * Convex gets *some* validation and improved support for the GJK `supportVertex()` API:
    [#488](https://github.com/flexible-collision-library/fcl/pull/488)
  * Fixed bug in collision function matrix that only allowed calculation of
    collision between ellipsoid and half space *with that ordering*. Now also
    supports half space and ellipsoid.
    [#520](https://github.com/flexible-collision-library/fcl/pull/520)
  * Do not flush error messages on cerr:
    [#542](https://github.com/flexible-collision-library/fcl/pull/542)

* Broadphase

* Narrowphase

  * Primitive convex-half space collision algorithm introduced:
    [#469](https://github.com/flexible-collision-library/fcl/pull/469)
  * Contact and distance query results types changed to be compatible with OcTree:
    [#472](https://github.com/flexible-collision-library/fcl/pull/472)
  * Documentation for OcTree no longer mistakenly excluded from doxygen:
    [#472](https://github.com/flexible-collision-library/fcl/pull/472)
  * Another failure mode in the GJK/EPA signed distance query patched:
    [#494](https://github.com/flexible-collision-library/fcl/pull/494)
  * Fix build when ccd_real_t == float:
    [#498](https://github.com/flexible-collision-library/fcl/pull/498)
  * Remove accidental recursive include:
    [#496](https://github.com/flexible-collision-library/fcl/pull/496)

* Build/Test/Misc

  * Fixed syntax which prevented building in Visual Studio 2015:
    [#459](https://github.com/flexible-collision-library/fcl/pull/459)
  * Fix compilation errors using default options on Emscripten:
    [#470](https://github.com/flexible-collision-library/fcl/pull/470)
  * Change supported linux build to Ubuntu 18.04 and Mac OS 10.14.6:
    [#489](https://github.com/flexible-collision-library/fcl/pull/489)

### FCL 0.6.1 (2020-02-26)

* Math

  * Replace M_PI instance with constants::pi():
     [#450](https://github.com/flexible-collision-library/fcl/pull/450)

* Narrowphase

  * Various corrections and clarifications of the GJK algorithm used for general
    convex distance:
     [#446](https://github.com/flexible-collision-library/fcl/pull/446)

* Build/Test/Misc

  * Clean up install config files and ensure find_dependency is called as
    appropriate:
     [#452](https://github.com/flexible-collision-library/fcl/pull/452)

### FCL 0.6.0 (2020-02-10)

* Core/Common

  * Removed dependency on boost:
     [#140](https://github.com/flexible-collision-library/fcl/pull/140),
     [#146](https://github.com/flexible-collision-library/fcl/pull/146),
     [#147](https://github.com/flexible-collision-library/fcl/pull/147),
     [#148](https://github.com/flexible-collision-library/fcl/pull/148)
  * Fix incorrect use of `FCL_BUILD_TYPE_DEBUG`:
     [#153](https://github.com/flexible-collision-library/fcl/pull/153)
  * Replaced `NULL` with `nullptr`:
     [#158](https://github.com/flexible-collision-library/fcl/pull/158)
  * Templatized FCL for scalar type:
     [#154](https://github.com/flexible-collision-library/fcl/pull/154),
     [#165](https://github.com/flexible-collision-library/fcl/pull/165),
     [#188](https://github.com/flexible-collision-library/fcl/pull/188)
  * Reorganized source tree:
     [#163](https://github.com/flexible-collision-library/fcl/pull/163),
     [#175](https://github.com/flexible-collision-library/fcl/pull/175)

* Math

  * Switched to Eigen for math operations:
     [#150](https://github.com/flexible-collision-library/fcl/pull/150)
  * Rework `fcl::constants`; replace M_PI instances and include scalar-dependent
    numerical tolerances:
     [#264](https://github.com/flexible-collision-library/fcl/pull/264),
     [#279](https://github.com/flexible-collision-library/fcl/pull/279)
  * `fcl::Transform` defined to be an `Isometry` to optimize inverses:
     [#318](https://github.com/flexible-collision-library/fcl/pull/318)

* Geometry

  * BVH Model throws intelligent errors when it runs out of memory:
     [#237](https://github.com/flexible-collision-library/fcl/pull/237)
  * Generate a BVH Model from multiple primitives:
     [#308](https://github.com/flexible-collision-library/fcl/pull/308)
  * Clean up `Convex` class:
     [#325](https://github.com/flexible-collision-library/fcl/pull/325),
     [#338](https://github.com/flexible-collision-library/fcl/pull/338),
     [#369](https://github.com/flexible-collision-library/fcl/pull/369)
  * Computation of `Capsule` moment of inertia corrected:
     [#420](https://github.com/flexible-collision-library/fcl/pull/420)
  * Added tests on local AABB computation for `Capsule`:
     [#423](https://github.com/flexible-collision-library/fcl/pull/423)
  * Fixed interpretation of capsule parameters in primitive capsule-capsule
    distance computation.
     [#436](https://github.com/flexible-collision-library/fcl/pull/436)

* Broadphase

  * Fixed redundant pair checking of `SpatialHashingCollisionManager`:
     [#156](https://github.com/flexible-collision-library/fcl/pull/156)
  * Clean up of hierarchy tree code:
     [#439](https://github.com/flexible-collision-library/fcl/pull/439)
  * Default callback functions for broadphase collision managers have been moved
    out of `fcl::test` and into `fcl` namespace (with a corresponding name
    change, e.g., `defaultDistanceFunction` --> `DefaultDistanceFunction`).
     [#438](https://github.com/flexible-collision-library/fcl/pull/438)
    * This includes the removal of the stub function
      `defaultContinuousDistanceFunction()`.

* Narrowphase

  * Added distance request option for computing exact negative distance:
     [#172](https://github.com/flexible-collision-library/fcl/pull/172)
  * Adjust tolerance on cylinder-cone unit test to pass on MacOS:
     [#198](https://github.com/flexible-collision-library/fcl/pull/198)
  * Unify computation of nearest point in convexity-based distance algorithms:
     [#215](https://github.com/flexible-collision-library/fcl/pull/215)
  * Fixed bug in cylinder-half space collision query:
     [#255](https://github.com/flexible-collision-library/fcl/pull/255),
     [#267](https://github.com/flexible-collision-library/fcl/pull/267)
  * Errors in box-box collision function addressed -- this changes the semantics
    of the old results: penetration depth is a *positive* value and the position
    of the penetration will *not* lie on the surface of one box, but lies at the
    midpoint between the two penetrating surfaces:
     [#259](https://github.com/flexible-collision-library/fcl/pull/259)
  * Fixed bug in `meshConservativeAdvancementOrientedNodeCanStop`:
     [#271](https://github.com/flexible-collision-library/fcl/pull/271)
  * `CollisionRequest` gets a "GJK tolerance":
     [#283](https://github.com/flexible-collision-library/fcl/pull/283)
  * Correct distance queries to report nearest point in _world_ frame:
     [#288](https://github.com/flexible-collision-library/fcl/pull/288)
  * Various corrections and clarifications of the GJK algorithm used for general
    convex distance:
     [#290](https://github.com/flexible-collision-library/fcl/pull/290),
     [#296](https://github.com/flexible-collision-library/fcl/pull/296),
     [#324](https://github.com/flexible-collision-library/fcl/pull/324),
     [#365](https://github.com/flexible-collision-library/fcl/pull/365),
     [#367](https://github.com/flexible-collision-library/fcl/pull/367),
     [#373](https://github.com/flexible-collision-library/fcl/pull/373)
  * Remove duplicated code between GJKDistance and GJKSignedDistance:
     [#292](https://github.com/flexible-collision-library/fcl/pull/292)
  * Significant bug fixes in the EPA algorithm for computing signed distance on
    penetrating convex shapes:
     [#305](https://github.com/flexible-collision-library/fcl/pull/305),
     [#314](https://github.com/flexible-collision-library/fcl/pull/314),
     [#336](https://github.com/flexible-collision-library/fcl/pull/336),
     [#352](https://github.com/flexible-collision-library/fcl/pull/352),
     [#388](https://github.com/flexible-collision-library/fcl/pull/388),
     [#397](https://github.com/flexible-collision-library/fcl/pull/397),
     [#417](https://github.com/flexible-collision-library/fcl/pull/417),
     [#434](https://github.com/flexible-collision-library/fcl/pull/434),
     [#435](https://github.com/flexible-collision-library/fcl/pull/435),
     [#437](https://github.com/flexible-collision-library/fcl/pull/437)
  * Add custom sphere-box collision and distance algorithms for both solvers:
     [#316](https://github.com/flexible-collision-library/fcl/pull/316)
  * Add custom sphere-cylinder collision and distance algorithms for both
    solvers:
     [#321](https://github.com/flexible-collision-library/fcl/pull/321)
  * Octree-mesh distance query returns witness points:
     [#427](https://github.com/flexible-collision-library/fcl/pull/427)

* Build/Test/Misc

  * Ensure the locally generated config.h is used:
     [#142](https://github.com/flexible-collision-library/fcl/pull/142)
  * Use major.minor version for ABI soversion:
     [#143](https://github.com/flexible-collision-library/fcl/pull/143)
  * Added missing copyright headers:
     [#149](https://github.com/flexible-collision-library/fcl/pull/149)
  * Enable Win32 builds on AppVeyor CI:
     [#157](https://github.com/flexible-collision-library/fcl/pull/157)
  * Enabled build with SSE option by default:
     [#159](https://github.com/flexible-collision-library/fcl/pull/159)
  * Show build status of master branch in README.md:
     [#166](https://github.com/flexible-collision-library/fcl/pull/166)
  * Added CMake targets for generating API documentation:
     [#174](https://github.com/flexible-collision-library/fcl/pull/174)
  * Clean up finding external dependencies and use imported targets where
    available:
     [#181](https://github.com/flexible-collision-library/fcl/pull/181),
     [#182](https://github.com/flexible-collision-library/fcl/pull/182),
     [#196](https://github.com/flexible-collision-library/fcl/pull/196)
  * Added version check for Visual Studio in CMake (VS2015 or greater required):
     [#189](https://github.com/flexible-collision-library/fcl/pull/189)
  * Add dedicated SSE CMake option:
     [#191](https://github.com/flexible-collision-library/fcl/pull/191)
  * Remove unused references to TinyXML from build:
     [#193](https://github.com/flexible-collision-library/fcl/pull/193)
  * Minor corrections to signed distance tests:
     [#199](https://github.com/flexible-collision-library/fcl/pull/199)
  * Fix various compiler warnings and enable warnings as errors in CI:
     [#197](https://github.com/flexible-collision-library/fcl/pull/197),
     [#200](https://github.com/flexible-collision-library/fcl/pull/200),
     [#204](https://github.com/flexible-collision-library/fcl/pull/204),
     [#205](https://github.com/flexible-collision-library/fcl/pull/205)
  * Allow the CMake RPATH to be configured:
     [#203](https://github.com/flexible-collision-library/fcl/pull/203)
  * Set SSE flags for the Apple compiler:
     [#206](https://github.com/flexible-collision-library/fcl/pull/206)
  * Windows CI always uses double-valued libccd:
     [#216](https://github.com/flexible-collision-library/fcl/pull/216)
  * Clean up of CMake install configuration:
     [#230](https://github.com/flexible-collision-library/fcl/pull/230)
  * Formalize visibility of binary symbols:
     [#233](https://github.com/flexible-collision-library/fcl/pull/233)
  * Remove tapping deprecated homebrew-science:
     [#262](https://github.com/flexible-collision-library/fcl/pull/262)
  * Move travis CI to use xcode 9 instead of 7.3:
     [#266](https://github.com/flexible-collision-library/fcl/pull/266)
  * Fix VS2017 incompatibility:
     [#277](https://github.com/flexible-collision-library/fcl/pull/277)
  * Mention Visual Studio version requirement in INSTALL file:
     [#284](https://github.com/flexible-collision-library/fcl/pull/284)
  * Correct CMake error message for the wrong version of libccd:
     [#286](https://github.com/flexible-collision-library/fcl/pull/286)
  * Added test utility for performing equality between Eigen matrix-types
    (`CompareMatrices` in `test/eign_matrix_compare.h`):
     [#316](https://github.com/flexible-collision-library/fcl/pull/316)
  * Toward enabling dashboards on CI:
     [#328](https://github.com/flexible-collision-library/fcl/pull/328)
  * Add configuration files for various static analyzers:
     [#332](https://github.com/flexible-collision-library/fcl/pull/332)
  * Update AppVeyor badge URL in README:
     [#342](https://github.com/flexible-collision-library/fcl/pull/342)
  * CMake fixes and cleanup:
     [#360](https://github.com/flexible-collision-library/fcl/pull/360)
  * Enable --output-on-failure for CI builds:
     [#362](https://github.com/flexible-collision-library/fcl/pull/362)
  * Corrected test of the distance function to be compatible with libccd 2:
     [#371](https://github.com/flexible-collision-library/fcl/pull/371)
  * Provides the `UnexpectedConfigurationException` so that when narrowphase
    operations encounter an error, they can throw this new exception which
    will trigger a logging of the types and poses of the geometries that led to
    the error:
     [#381](https://github.com/flexible-collision-library/fcl/pull/381)
  * Provide catkin packaage.xml per ROS REP 136:
     [#409](https://github.com/flexible-collision-library/fcl/pull/409)
  * Updated README.md to reflect FCL 0.6.0 syntax changes:
     [#410](https://github.com/flexible-collision-library/fcl/pull/410)

### FCL 0.5.0 (2016-07-19)

* Added safe-guards to allow octree headers only if octomap enabled:
   [#136](https://github.com/flexible-collision-library/fcl/pull/136)
* Added CMake option to disable octomap in build:
   [#135](https://github.com/flexible-collision-library/fcl/pull/135)
* Added automatic coverage test reporting:
   [#125](https://github.com/flexible-collision-library/fcl/pull/125),
   [#98](https://github.com/flexible-collision-library/fcl/pull/98)
* Added CMake exported targets:
   [#116](https://github.com/flexible-collision-library/fcl/pull/116)
* Fixed API to support Octomap 1.8:
   [#129](https://github.com/flexible-collision-library/fcl/pull/129),
   [#126](https://github.com/flexible-collision-library/fcl/issues/126)
* Fixed continuousCollisionNaive() wasn't resetting the returned result when no
  collision:
   [#123](https://github.com/flexible-collision-library/fcl/pull/123)
* Fixed uninitialized tf in TranslationMotion:
   [#121](https://github.com/flexible-collision-library/fcl/pull/121)
* Fixed fcl.pc populated incorrect installation paths:
   [#118](https://github.com/flexible-collision-library/fcl/pull/118)
* Fixed octree vs mesh CollisionResult now returns triangle id:
   [#114](https://github.com/flexible-collision-library/fcl/pull/114)
* Fixed minor typo:
   [#113](https://github.com/flexible-collision-library/fcl/pull/113)
* Fixed fallback finding of libccd:
   [#112](https://github.com/flexible-collision-library/fcl/pull/112)
* Fixed a nasty bug in propagate propagateBVHFrontListCollisionRecurse():
   [#110](https://github.com/flexible-collision-library/fcl/pull/110)
* Fixed test_fcl_math failures on Windows 64 bit due to non-portable use of
  long:
   [#108](https://github.com/flexible-collision-library/fcl/pull/108),
   [#107](https://github.com/flexible-collision-library/fcl/issues/107)
* Fixed compilation in Visual Studio 2015, and suppressed some warnings:
   [#99](https://github.com/flexible-collision-library/fcl/pull/99)
* Fixed build when libccd package config not found:
   [#94](https://github.com/flexible-collision-library/fcl/pull/94)
* Removing dependency on boost:
   [#108](https://github.com/flexible-collision-library/fcl/pull/108),
   [#105](https://github.com/flexible-collision-library/fcl/pull/105),
   [#104](https://github.com/flexible-collision-library/fcl/pull/104),
   [#103](https://github.com/flexible-collision-library/fcl/pull/103)
