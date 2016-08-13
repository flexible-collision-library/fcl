## FCL 0

### FCL 0.6.0 (2016-XX-XX)

* Added missing copyright headers:  [#149](https://github.com/flexible-collision-library/fcl/pull/149)
* Enabled build with SSE option by default: [#159](https://github.com/flexible-collision-library/fcl/pull/159)
* Switched to Eigen for math operations: [#150](https://github.com/flexible-collision-library/fcl/pull/150), [#96](https://github.com/flexible-collision-library/fcl/issues/96)
* Fixed redundant pair checking of SpatialHashingCollisionManager: [#156](https://github.com/flexible-collision-library/fcl/pull/156)
* Removed dependency on boost: [#148](https://github.com/flexible-collision-library/fcl/pull/148), [#147](https://github.com/flexible-collision-library/fcl/pull/147), [#146](https://github.com/flexible-collision-library/fcl/pull/146), [#140](https://github.com/flexible-collision-library/fcl/pull/140)

### FCL 0.5.0 (2016-07-19)

* Added safe-guards to allow octree headers only if octomap enabled: [#136](https://github.com/flexible-collision-library/fcl/pull/136)
* Added CMake option to disable octomap in build: [#135](https://github.com/flexible-collision-library/fcl/pull/135)
* Added automatic coverage test reporting: [#125](https://github.com/flexible-collision-library/fcl/pull/125), [#98](https://github.com/flexible-collision-library/fcl/pull/98)
* Added CMake exported targets: [#116](https://github.com/flexible-collision-library/fcl/pull/116)
* Fixed API to support Octomap 1.8: [#129](https://github.com/flexible-collision-library/fcl/pull/129), [#126](https://github.com/flexible-collision-library/fcl/issues/126)
* Fixed continuousCollisionNaive() wasn't resetting the returned result when no collision: [#123](https://github.com/flexible-collision-library/fcl/pull/123)
* Fixed uninitialized tf in TranslationMotion: [#121](https://github.com/flexible-collision-library/fcl/pull/121)
* Fixed fcl.pc populated incorrect installation paths: [#118](https://github.com/flexible-collision-library/fcl/pull/118)
* Fixed octree vs mesh CollisionResult now returns triangle id: [#114](https://github.com/flexible-collision-library/fcl/pull/114)
* Fixed minor typo: [#113](https://github.com/flexible-collision-library/fcl/pull/113)
* Fixed fallback finding of libccd: [#112](https://github.com/flexible-collision-library/fcl/pull/112)
* Fixed a nasty bug in propagate propagateBVHFrontListCollisionRecurse(): [#110](https://github.com/flexible-collision-library/fcl/pull/110)
* Fixed test_fcl_math failures on Windows 64 bit due to non-portable use of long: [#108](https://github.com/flexible-collision-library/fcl/pull/108), [#107](https://github.com/flexible-collision-library/fcl/issues/107)
* Fixed compilation in Visual Studio 2015, and suppressed some warnings: [#99](https://github.com/flexible-collision-library/fcl/pull/99)
* Fixed build when libccd package config not found: [#94](https://github.com/flexible-collision-library/fcl/pull/94)
* Removing dependency on boost: [#108](https://github.com/flexible-collision-library/fcl/pull/108), [#105](https://github.com/flexible-collision-library/fcl/pull/105), [#104](https://github.com/flexible-collision-library/fcl/pull/104), [#103](https://github.com/flexible-collision-library/fcl/pull/103)
