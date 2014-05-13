#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestFixture.h>

#include "partial_pointcloud/partial_view.h"

class Partial_Pointcloud_Test : public CppUnit::TestFixture{
  
  CPPUNIT_TEST_SUITE( Partial_Pointcloud_Test );
  CPPUNIT_TEST( testNoSegfaultOnDestruction );
  CPPUNIT_TEST_SUITE_END();

public:

  void testNoSegfaultOnDestruction()
  {
    PartialViewComputer * comp = new PartialViewComputer();
    delete comp;
    CPPUNIT_ASSERT( true );
  }

};

CPPUNIT_TEST_SUITE_REGISTRATION( Partial_Pointcloud_Test );
