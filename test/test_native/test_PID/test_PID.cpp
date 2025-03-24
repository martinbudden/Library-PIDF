#include <PIDF.h>
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_PID_init() {
    const PIDF pid;
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getF());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    const PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(error.P, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.I, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.D, 0.0);
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
void test_PID() {
    PIDF pid(PIDF::PIDF_t { 5.0F, 3.0F, 1.0F, 0.0F });

    TEST_ASSERT_EQUAL_FLOAT(5.0F, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(3.0F, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(1.0F, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getF());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    const float deltaT {0.01F};
    const float input0 {0.0F};
    const float input  {0.5F};
    const float output = pid.update(input, deltaT);

    const PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-input * 5.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-0.5F*(input0 + input) * 3.0F * deltaT, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-input * 1.0F / deltaT, error.D);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I + error.D, output);
}

void test_P_Controller()
{
    PIDF pid(PIDF::PIDF_t { 1.0, 0.0, 0.0, 0.0 });
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(1.0, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getF());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    float output = pid.update(0, deltaT);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0, error.I);
    TEST_ASSERT_EQUAL_FLOAT(0, error.D);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I + error.D, output);

    pid.setSetpoint(5.0);
    output = pid.update(0.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(5.0, output);
    TEST_ASSERT_EQUAL_FLOAT(5.0, error.P);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I + error.D, output);

    output = pid.update(1.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(4.0, output);
    TEST_ASSERT_EQUAL_FLOAT(4.0, error.P);

    output = pid.update(2.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(3.0, output);
    TEST_ASSERT_EQUAL_FLOAT(3.0, error.P);

    output = pid.update(3.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(2.0, output);
    TEST_ASSERT_EQUAL_FLOAT(2.0, error.P);

    output = pid.update(4.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(1.0, output);
    TEST_ASSERT_EQUAL_FLOAT(1.0, error.P);

    output = pid.update(5.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0, error.P);

    output = pid.update(6.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-1.0, output);
    TEST_ASSERT_EQUAL_FLOAT(-1.0, error.P);

    output = pid.update(5.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0, error.P);
}

void test_PI_Controller()
{
    PIDF pid(PIDF::PIDF_t { 0.3F, 0.2F, 0.0F, 0.0F });
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(0.3F, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.2F, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getF());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0, deltaT);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.D);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I + error.D, output);

    pid.setSetpoint(5.0F);
    output = pid.update(0.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(2.0F, output);

    output = pid.update(1.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(1.2F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(2.6F, output);

    output = pid.update(4.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.3F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.9F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(2.2F, output);

    output = pid.update(7.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.8F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(1.2F, output);

    output = pid.update(6.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(1.2F, output);

    output = pid.update(5.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, output);

    output = pid.update(5.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, output);
}

void test_integral_max()
{
    PIDF pid(PIDF::PIDF_t { 0.2F, 0.3F, 0.0F, 0.0F });
    pid.setIntegralMax(2.0F);
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0.0F, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(-0.7F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-0.9F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.3F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.9F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-2.4F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-2.4F, output);
}

void test_integral_saturation_positive()
{
    PIDF pid(PIDF::PIDF_t { 0.2F, 0.3F, 0.0F, 0.0F });
    pid.setOutputSaturationValue(1.5F);
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0.0F, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(-0.7F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-0.9F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.3F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.1F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.1F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(1.5F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.2F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(1.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.2F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.3F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(0.5F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.4F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(0.1F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.02F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.48F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(0.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.48F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.48F, output);

    output = pid.update(0.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.48F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.48F, output);
}

void test_integral_saturation_negative()
{
    PIDF pid(PIDF::PIDF_t { 0.2F, 0.3F, 0.0F, 0.0F });
    pid.setOutputSaturationValue(1.5F);
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0.0F, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);

    output = pid.update(-2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.3F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, output);

    output = pid.update(-2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.9F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(1.3F, output);

    output = pid.update(-2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.1F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, output);

    output = pid.update(-2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.1F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, output);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_PID_init);
    RUN_TEST(test_PID);
    RUN_TEST(test_P_Controller);
    RUN_TEST(test_PI_Controller);
    RUN_TEST(test_integral_max);
    RUN_TEST(test_integral_saturation_positive);
    RUN_TEST(test_integral_saturation_negative);

    UNITY_END();
}
