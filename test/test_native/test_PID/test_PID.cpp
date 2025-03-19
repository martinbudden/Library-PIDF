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
    PIDF pid(PIDF::PIDF_t { 5.0, 3.0, 1.0, 0.0 });

    TEST_ASSERT_EQUAL_FLOAT(5.0, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(3.0, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(1.0, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getF());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    const float deltaT {0.01};
    const float input0 {0.0};
    const float input  {0.5};
    const float output = pid.update(input, deltaT);

    const PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-input * 5.0, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-0.5F*(input0 + input) * 3.0 * deltaT, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-input * 1.0 / deltaT, error.D);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I + error.D, output);
}

void test_P_Controller() {
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

void test_PI_Controller() {
    PIDF pid(PIDF::PIDF_t { 0.3, 0.2, 0.0, 0.0 });
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(0.3, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.2, pid.getI());
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
    TEST_ASSERT_EQUAL_FLOAT(1.5, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.5, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(2.0, output);

    output = pid.update(1.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(1.2, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.4, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(2.6, output);

    output = pid.update(4.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.3, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.9, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(2.2, output);

    output = pid.update(7.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.6, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.8, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(1.2, output);

    output = pid.update(6.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.3, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.5, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(1.2, output);

    output = pid.update(5.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.5, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(1.5, output);

    output = pid.update(5.0, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.5, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(1.5, output);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_PID_init);
    RUN_TEST(test_PID);
    RUN_TEST(test_P_Controller);
    RUN_TEST(test_PI_Controller);

    UNITY_END();
}
