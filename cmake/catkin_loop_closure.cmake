find_package(PCL REQUIRED COMPONENTS common filters io registration kdtree)
list(APPEND livo_atlas_LIBRARIES pcl_registration pcl_kdtree)
