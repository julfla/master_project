#include "../include/mesh/vector_3d.hpp"
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestFixture.h>

class Vector_3D_Test : public CppUnit::TestFixture{
  
  CPPUNIT_TEST_SUITE( Vector_3D_Test );
  CPPUNIT_TEST( testEquality );
  CPPUNIT_TEST( testAddition );
  CPPUNIT_TEST( testMultiplication );
  CPPUNIT_TEST( testSoustraction );
  CPPUNIT_TEST( testDot );
  CPPUNIT_TEST( testCross );
  CPPUNIT_TEST( testNorm );
  CPPUNIT_TEST_SUITE_END();

private:
  Vector_3D *v_1_1_1, *v_1_0_0, *v_0_1_0, *v_0_0_1;
public:
  void setUp()
  {
    v_1_1_1 = new Vector_3D(1, 1, 1);
    v_1_0_0 = new Vector_3D(1, 0, 0);
    v_0_1_0 = new Vector_3D(0, 1, 0);
    v_0_0_1 = new Vector_3D(0, 0, 1);
  }

  void tearDown() 
  {
    delete v_1_1_1;
    delete v_1_0_0;
    delete v_0_1_0;
    delete v_0_0_1;
  }

  void testEquality()
  {
    CPPUNIT_ASSERT_EQUAL( *v_1_1_1, *v_1_1_1 );
    CPPUNIT_ASSERT( !(*v_1_1_1 == *v_1_0_0) );
    CPPUNIT_ASSERT_EQUAL( *v_1_1_1, Vector_3D(1,1,1+1e-7) );
  }
  
  void testAddition()
  {
    CPPUNIT_ASSERT_EQUAL( *v_1_1_1 + *v_1_1_1, Vector_3D(2,2,2) );
  }
  
  void testSoustraction()
  {
    CPPUNIT_ASSERT_EQUAL( *v_1_1_1 - *v_1_1_1, Vector_3D(0,0,0) );
  }
  
  void testMultiplication() {
    CPPUNIT_ASSERT_EQUAL( *v_1_1_1 * 2, Vector_3D(2,2,2));
    CPPUNIT_ASSERT_EQUAL( 2 * *v_1_1_1, Vector_3D(2,2,2));
    CPPUNIT_ASSERT_EQUAL( *v_1_1_1 * 2, 2 * *v_1_1_1);
  }
  
  void testDot() {
    CPPUNIT_ASSERT_EQUAL( *v_1_0_0 * *v_1_0_0, 1.0);
    CPPUNIT_ASSERT_EQUAL( *v_1_0_0 * *v_0_0_1, 0.0);
  }
  
  void testCross() {
    CPPUNIT_ASSERT_EQUAL( *v_1_0_0 % *v_0_1_0, *v_0_0_1);
    CPPUNIT_ASSERT_EQUAL( (*v_1_0_0 % *v_1_0_0).norm(), 0.0);
  }
  
  void testNorm() {
    CPPUNIT_ASSERT_EQUAL( Vector_3D().norm(),0.0);
    CPPUNIT_ASSERT_EQUAL( v_1_0_0->norm(), 1.0);
    CPPUNIT_ASSERT_EQUAL( v_1_1_1->norm(),sqrt(3));
  }
};


CPPUNIT_TEST_SUITE_REGISTRATION( Vector_3D_Test );