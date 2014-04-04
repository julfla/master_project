#include "../src/point_3d.h"
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestFixture.h>

class Point_3D_Test : public CppUnit::TestFixture{
  
  CPPUNIT_TEST_SUITE( Point_3D_Test );
  CPPUNIT_TEST( testEquality );
  CPPUNIT_TEST( testAddition );
  CPPUNIT_TEST( testMultiplication );
  CPPUNIT_TEST( testSoustraction );
  CPPUNIT_TEST_SUITE_END();

private:
  Point_3D *p_1_1_1, *p_1_2_3;
public:
  void setUp()
  {
    p_1_1_1 = new Point_3D(1, 1, 1);
    p_1_2_3= new Point_3D(1, 2, 3);
  }

  void tearDown() 
  {
    delete p_1_1_1;
    delete p_1_2_3;
  }

  void testEquality()
  {
    CPPUNIT_ASSERT_MESSAGE( "Point is (0,0,0)", !(Point_3D(1,0,0) == Point_3D()));
    CPPUNIT_ASSERT_MESSAGE( "Error with little difference", !(Point_3D(0.01,0,0) == Point_3D()));
    CPPUNIT_ASSERT_EQUAL( *p_1_1_1, Point_3D(1.0, 1.0, 1.0 + 1e-9) );
    CPPUNIT_ASSERT( !(*p_1_1_1 == *p_1_2_3) );
  }

  void testAddition()
  {
    CPPUNIT_ASSERT_EQUAL( *p_1_1_1 + *p_1_1_1, Point_3D(2,2,2) );
  }
  
  void testSoustraction()
  {
    CPPUNIT_ASSERT_EQUAL( *p_1_1_1 - *p_1_1_1, Point_3D(0,0,0) );
  }
  
  void testMultiplication() {
    CPPUNIT_ASSERT_EQUAL( *p_1_1_1 * 2, Point_3D(2,2,2));
    CPPUNIT_ASSERT_EQUAL( 2 * *p_1_1_1, Point_3D(2,2,2));
    CPPUNIT_ASSERT_EQUAL( *p_1_1_1 * 2, 2 * *p_1_1_1);
  }

  /*void testToString() {
      std::string expected = "x:1.0 y:1.0 z:1.0";
      std::string actual;
      actual << Point_3D(1,1,1);
      CPPUNIT_ASSERT_EQUAL( expected, actual);
  }*/
};

CPPUNIT_TEST_SUITE_REGISTRATION( Point_3D_Test );
