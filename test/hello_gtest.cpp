#include <gtest/gtest.h>
#include "test-generate-AABB.cu"
TEST(HelloTest, BasicAssertions) {
    EXPECT_EQ(1 + 1, 2);
    EXPECT_TRUE(true);
}

// Confirm that AABB's are generated correctly
TEST(AABB_Test, PreProcessing) {
    const int numConfigs = 2;
    AABB botBoundsBaseline[numConfigs];
    AABB botBoundsParallel[numConfigs];
    test_generateAABBBaseline(botBoundsBaseline, numConfigs);
    test_generateAABB(botBoundsParallel, numConfigs);
    EXPECT_TRUE(verify_generateAABB(botBoundsBaseline, botBoundsParallel, numConfigs))
}

// Confirm that 
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}