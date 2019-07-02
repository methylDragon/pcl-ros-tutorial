0.2.0 (2016-07-10)
------------------
* Fix: RViz uses Qt5 in kinetic
* Contributors: Armin Hornung

0.1.0 (2016-07-07)
------------------
* Remove -ldefault_plugin from linker options, fixes `#19 <https://github.com/OctoMap/octomap_rviz_plugins/issues/19>`_
* Support for displaying ColorOcTree and OcTreeStamped, templated rviz plugins 
* Trim Z values in the octomap visualization
* Add alpha property to OccupancyGridDisplay
* add fix for qt moc BOOST_JOIN problem for osx yosemite build
* Contributors: Armin Hornung, Felix Endres, Gautham Manoharan, Javier V. Gómez, Oleksandr Lavrushchenko, Ryohei Ueda

0.0.5 (2013-09-06)
------------------
* fix a crash when the destructor is called before onInitialize
* Fix descriptions, limit octree depth range
* Porting fixes from groovy branch (QT4_WRAP macro, OCTOMAP_INCLUDE_DIRS)

0.0.4 (2013-07-05)
------------------
* Safer checking for octree conversion from stream
* Create octomap using AbstracOcTree (Fix issue #1)

0.0.3 (2013-06-26)
------------------
* correct CMakeLists. octomap_INCLUDE_DIRS(or LIB..) to OCTOMAP_INCLUDE_DIRS

0.0.2 (2013-05-08)
------------------
* 0.0.1 -> 0.0.2
* adding OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED option to make QT4_WRAP macro happy

0.0.1 (2013-05-04)
------------------
* removing dependency to octomap_ros package + clean-up
* removing rosbuild Makefile
* major revision
* adjusting to recent rviz api changes
* removed pcl dependancy
* fixed threading issue
* working version with ogre point cloud structures and colored boxes
* added max depth filter
