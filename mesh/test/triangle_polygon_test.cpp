#include "../include/mesh/triangle_polygon.hpp"
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestFixture.h>


class Triangle_Mesh_Test : public CppUnit::TestFixture{
  
  CPPUNIT_TEST_SUITE( Triangle_Mesh_Test );
  CPPUNIT_TEST( testArea );
  CPPUNIT_TEST( testRandomPoint );
  CPPUNIT_TEST( testCreateFromString );
  CPPUNIT_TEST( testDefaultConstructor );
  CPPUNIT_TEST_SUITE_END();

private:
  Point_3D *p_0_0_0, *p_1_0_0, *p_0_1_0, *p_2_0_0, *p_0_2_0;
public:
  
  void setUp() {
    p_0_0_0 = new Point_3D();
    p_1_0_0 = new Point_3D(1,0,0);
    p_2_0_0 = new Point_3D(2,0,0);
    p_0_1_0 = new Point_3D(0,1,0);
    p_0_2_0 = new Point_3D(0,2,0);
  }
  
  void tearDown() {
    delete p_0_0_0;
    delete p_1_0_0;
    delete p_0_1_0;
    delete p_2_0_0;
    delete p_0_2_0;
  }

  /*void testToString() {
      std::string expected, actual;
      expected << *p_0_0_0 << " " << *p_1_0_0 << " " << *p_0_1_0;
      actual << TrianglePolygon(*p_0_0_0,*p_1_0_0,*p_0_1_0);
      CPPUNIT_ASSERT_EQUAL(expected, actual);
  }*/

  void testDefaultConstructor() {
      TrianglePolygon();
  }

  void testArea() {
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, TrianglePolygon(*p_0_0_0,*p_1_0_0,*p_0_1_0).getArea(), 1e-3);
  }
   
  void testRandomPoint() {
    TrianglePolygon mesh(*p_0_0_0,*p_1_0_0,*p_0_1_0);

    Point_3D rand_pts;
    //test if the point is "random".
    //we could use R to test the randomness of it more trusfully.
    rand_pts = mesh.getRandomPoint();
    CPPUNIT_ASSERT_MESSAGE("Random point is (0,0,0).", !(rand_pts == Point_3D()));
    CPPUNIT_ASSERT_MESSAGE("Consecutive random points are equal.", !(mesh.getRandomPoint() == mesh.getRandomPoint()) );

    //test if the point is inside the mesh.
    Vector_3D v(*p_0_0_0, mesh.getRandomPoint());
    Vector_3D u1(*p_0_0_0, *p_1_0_0);
    Vector_3D u2(*p_0_0_0, *p_0_1_0);
    CPPUNIT_ASSERT_MESSAGE( "Random point is out of the mesh.", u1*v <= u1*u1 && u2*v <= u2*u2 );
  }
  
  void testCreateFromString() {
    std::string line(
      "0.0 1.14904851942814 4.0 0.420580948291596 0.420580948291597 0.0 0.420580948291596 0.420580948291597 4.0");
    TrianglePolygon actual(line);

    Point_3D point1(0.0,1.14904851942814,4.0);
    Point_3D point2(0.420580948291596, 0.420580948291597, 0.0);
    Point_3D point3(0.420580948291596, 0.420580948291597, 4.0);
    TrianglePolygon expected(point1,point2,point3);
    
    CPPUNIT_ASSERT_EQUAL(expected, actual);
  }
};

CPPUNIT_TEST_SUITE_REGISTRATION( Triangle_Mesh_Test );
