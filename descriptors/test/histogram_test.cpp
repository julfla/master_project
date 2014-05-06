#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestFixture.h>

#include "../include/descriptors/shape_distribution.h"
#include <cstdio> // needed to create a temp file

class Histogram_Test : public CppUnit::TestFixture{

  CPPUNIT_TEST_SUITE( Histogram_Test );
  CPPUNIT_TEST( easyOne );
  CPPUNIT_TEST( scale_down );
  CPPUNIT_TEST_SUITE_END();

private:
  std::vector<double> *linear_100, *vect_void;

public:

  void setUp()
  {
      vect_void = new std::vector<double>();
      linear_100 = new std::vector<double>();
      for(int i=0; i < 100; ++i) {
          linear_100->push_back(i);
      }
  }

  void tearDown()
  {
    delete vect_void;
    delete linear_100;
  }

  void easyOne() {

      CPPUNIT_ASSERT_EQUAL_MESSAGE("Empty data should return empty an histogram containing 0.", 0, (int) Histogram(vect_void, 1).data->at(0));

      CPPUNIT_ASSERT_EQUAL_MESSAGE("One bin should return unit Histogram", (size_t) 1, Histogram(linear_100, 1).data->size());
      CPPUNIT_ASSERT_EQUAL_MESSAGE("One bin should return an histogram containing data_size.", (int) linear_100->size(), (int) Histogram(linear_100, 1).data->at(0));

      std::vector<double> data_hist = *(Histogram(linear_100, linear_100->size()).data);
      bool is_constant = true;
      int sum = 0;
      std::ostringstream message;
      message << "Linear data should return an constant histogram. Data is :" << std::endl;
      for(int i=0; i < data_hist.size(); ++i) {
          sum += data_hist.at(i);
          message << data_hist.at(i);
          if(i+1 < data_hist.size())
              message << ", ";
          if(data_hist.at(i) != 1)
            is_constant = false;
      }
      CPPUNIT_ASSERT_EQUAL_MESSAGE("The sum of the bins' values should be the data_size", (int) linear_100->size(), sum);
      CPPUNIT_ASSERT_MESSAGE(message.str(), is_constant);
  }

  void scale_down() {
      Histogram hist_100(linear_100, 100);
      hist_100.scale_down(2);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("Scale down down by two should divide the length by two", 50, (int) hist_100.data->size());
      hist_100.scale_down(3);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("Scale down a 50 size by three is impossible", 50, (int) hist_100.data->size());
  }
  
};

CPPUNIT_TEST_SUITE_REGISTRATION( Histogram_Test );
