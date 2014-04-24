#include "cerevoice_tts/CerevoiceTts.h"

// Bring in gtest
#include <gtest/gtest.h>

/*void SetUp()
{

}*/

// Declare a test
TEST(TtsTestSuite, testCreateObject)
{
  cerevoice_tts::CerevoiceTts tts;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);

return RUN_ALL_TESTS();
}
