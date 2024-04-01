#include <gtest/gtest.h>
#include "control/pid.hpp"

PIDController pid = PIDController(1.0f, 0.0f, 0.0f);

TEST(PIDController, SetCoefficients) {
    PIDCoefficients coefficients = {2.0f, 3.0f, 4.0f};
    pid.setCoefficients(coefficients);

    auto result = pid.getCoefficients();
    EXPECT_EQ(coefficients.kp, result.kp);
    EXPECT_EQ(coefficients.ki, result.ki);
    EXPECT_EQ(coefficients.kd, result.kd);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}