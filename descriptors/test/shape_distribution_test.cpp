//#include <cppunit/extensions/HelperMacros.h>
//#include <cppunit/TestFixture.h>

//#include "../include/descriptors/shape_distribution.h"
//#include <cstdio> // needed to create a temp file

//class Distribution_Test : public CppUnit::TestFixture{

//  typedef std::vector<double> vector_d;

//  CPPUNIT_TEST_SUITE( Distribution_Test );
//  //CPPUNIT_TEST( testCreateFromPath );
//  // CPPUNIT_TEST( testWriteToFile );
//  CPPUNIT_TEST( testSerialization );
//  CPPUNIT_TEST( distribution_is_density );
//  CPPUNIT_TEST( distance );
//  CPPUNIT_TEST_SUITE_END();

//private:
//  Point_3D *p_0_0, *p_0_1, *p_1_0, *p_1_1;
//  Distribution *distrib;
//  Mesh *mesh;
//  char* temp_path;

//public:
//  void setUp()
//  {
//    p_0_0 = new Point_3D(0,0,0);
//    p_0_1 = new Point_3D(0,1,0);
//    p_1_0 = new Point_3D(1,0,0);
//    p_1_1 = new Point_3D(1,1,0);

//    std::vector<TrianglePolygon> polygons;
//    polygons.push_back( TrianglePolygon(*p_0_0,*p_0_1,*p_1_0) );
//    polygons.push_back( TrianglePolygon(*p_0_1,*p_1_0,*p_1_1) );
//    mesh = new Mesh(polygons);
//    distrib = new Distribution(*mesh, 100);
//    temp_path = tmpnam(NULL);
//  }

//  void tearDown()
//  {
//    delete p_0_0;
//    delete p_0_1;
//    delete p_1_0;
//    delete p_1_1;
//    delete mesh;
//    delete distrib;
//  }

//  void distribution_is_density() {
//    CPPUNIT_ASSERT_EQUAL_MESSAGE("Size should match the parameter", 100, (int) distrib->getDensity().size());
//    double sum = 0;
//    double max_value = 0;

//    for(vector_d::iterator it = distrib->getDensity().begin(); it < distrib->getDensity().end(); ++it) {
//        sum += *it;
//        max_value = std::max(max_value, *it);
//    }
//    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("Cumulative sum is 1.0", 1.0, sum, 1e-2);
//    CPPUNIT_ASSERT_MESSAGE("Max value < 1", max_value < 1.0);
//  }

//  void testSerialization() {
//      std::string buff;
//      Distribution restore_distrib;
//      {
//          std::ostringstream archive_stream;
//          boost::archive::text_oarchive oa(archive_stream);
//          oa << *distrib;
//          buff = archive_stream.str();
//      }
//      {
//          std::istringstream archive_stream(buff);
//          boost::archive::text_iarchive oa(archive_stream);
//          oa >> restore_distrib;
//      }
//      CPPUNIT_ASSERT_EQUAL_MESSAGE("Size should match.", distrib->getDensity().size(), restore_distrib.getDensity().size());
//      CPPUNIT_ASSERT_MESSAGE("Vector should match.", distrib->getDistribution()== restore_distrib.getDistribution());
//  }

//  void distance() {
//      Distribution distribution1(*mesh, 100);
//      Distribution distribution2(*mesh, 100);
//      double dist1 = distribution1.distance(distribution2);
//      double dist2 = distribution2.distance(distribution1);
//      CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("Distance to itself should be 0.0", 0.0, distribution1.distance(distribution1), 1e-7);
//      CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("Distance should be symetric.", dist1, dist2, 1e-7);
//  }

//  /*void testCreateFromPath(){
//      CPPUNIT_ASSERT_THROW(Distribution(std::string(""), 10), std::ifstream::failure);
//      CPPUNIT_ASSERT_NO_THROW( Distribution(temp_path, 10));
//  }*/

////  void testWriteToFile(){
////      CPPUNIT_ASSERT_THROW(distrib->write(""), std::ifstream::failure);
////      CPPUNIT_ASSERT_NO_THROW(distrib->write(temp_path));
////  }
  
//};

//CPPUNIT_TEST_SUITE_REGISTRATION( Distribution_Test );
