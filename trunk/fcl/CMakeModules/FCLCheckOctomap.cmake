macro(fcl_check_for_octomap)
  #check for Octomap
  include(CheckCXXSourceRuns)
  set(OCTOMAP_FLAGS)
  
  check_cxx_source_runs("
    #include <octomap/octomap.h>
    int main()
    {
      octomap::OcTree* tree = new octomap::OcTree(0.1);
      
      for(int x = -20; x < 20; ++x)
      {
        for(int y = -20; y < 20; ++y)
	{
	  for(int z = -20; z < 20; ++z)
	  {
	    tree->updateNode(octomap::point3d(x * 0.05, y * 0.05, z * 0.05), true);
	  }
	}
      }
      
      for(int x = -30; x < 30; ++x)
      {
        for(int y = -30; y < 30; ++y)
	{
	  for(int z = -30; z < 30; ++z)
	  {
	    tree->updateNode(octomap::point3d(x * 0.02 - 1.0, y * 0.02 - 1.0, z * 0.02 - 1.0), false);
	  }
	}
      }

      return 1;
    }"
    HAS_OCTOMAP)
  
  set(CMAKE_REQUIRED_FLAGS)
  
  if(HAS_OCTOMAP)
    set(OCTOMAP_FLAGS "-loctomap")
    message(STATUS " Found octomap, using flags: ${OCTOMAP_FLAGS}")
  endif()

endmacro(fcl_check_for_octomap)