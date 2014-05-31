#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestFixture.h>

#include "partial_pointcloud/partial_view.h"

class Partial_Pointcloud_Test : public CppUnit::TestFixture{
  
  CPPUNIT_TEST_SUITE( Partial_Pointcloud_Test );
  CPPUNIT_TEST( testNoSegfaultOnDestruction );
  CPPUNIT_TEST( testEnoughPointInCloud );
  CPPUNIT_TEST_SUITE_END();

public:

  void testNoSegfaultOnDestruction() {
    PartialViewComputer * comp = new PartialViewComputer();
    delete comp;
    CPPUNIT_ASSERT( true );
  }

  void testEnoughPointInCloud() {
  	PartialViewComputer comp;
  	std::string path = "../partial_pointcloud/test/fa61e604661d4aa66658ecd96794a1cd.tri";
  	comp.loadMesh( path );
  	DefaultPointCloud cloud = comp.compute_view( 0.0, 0.0);
  	// One cloud can have width * height = 480000 points
  	// We expect at least 30% of it
  	CPPUNIT_ASSERT( cloud.size() >= 480000 * 0.33);
  }

};

CPPUNIT_TEST_SUITE_REGISTRATION( Partial_Pointcloud_Test );
