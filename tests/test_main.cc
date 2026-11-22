#include <gtest/gtest.h>

#include "livo_atlas/common/logging.h"

int main(int argc, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
