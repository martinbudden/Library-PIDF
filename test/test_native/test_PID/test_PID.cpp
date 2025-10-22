#include <PIDF.h>
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
void test_PID_init() {
    const PIDF pid;
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    const PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(error.P, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.I, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.D, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.S, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.K, 0.0);
}

void test_PID() {
    PIDF pid(PIDF::PIDF_t { 5.0F, 3.0F, 1.0F, 0.0F, 0.0F });

    TEST_ASSERT_EQUAL_FLOAT(5.0F, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(3.0F, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(1.0F, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getK());
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

void test_P_controller()
{
    PIDF pid(PIDF::PIDF_t { 1.0, 0.0, 0.0, 0.0, 0.0F });
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(1.0, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    float output = pid.update(0, deltaT);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0, error.I);
    TEST_ASSERT_EQUAL_FLOAT(0, error.D);
    TEST_ASSERT_EQUAL_FLOAT(0, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0, error.K);
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

void test_PI_controller()
{
    PIDF pid(PIDF::PIDF_t { 0.3F, 0.2F, 0.0F, 0.0F, 0.0F });
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(0.3F, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.2F, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0, deltaT);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.D);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I + error.D, output);

    pid.setSetpoint(5.0F);
    output = pid.update(0.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(5.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.5F, error.I); // (5.0 + 0.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(2.0F, output);

    output = pid.update(1.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(1.2F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(4.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.I); // 0.5 + (4.0 + 5.0) * 0.2 /2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(2.6F, output);

    output = pid.update(4.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.3F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.9F, error.I); // 1.4 + (1.0 + 4.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(2.2F, output);

    output = pid.update(7.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.8F, error.I); // 1.9 + (-2.0 + 1.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(1.2F, output);

    output = pid.update(6.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.I); // 1.8 + (-1.0 -2.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(1.2F, output);

    output = pid.update(5.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.I); // 1.5 + (0.0 - 1.0) * 0.2 /2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, output);

    output = pid.update(5.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.I); // 1.4 + (0.0 + 0.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, output);
}

void test_update_PI()
{
    PIDF pid(PIDF::PIDF_t { 0.3F, 0.2F, 0.0F, 0.0F, 0.0F });
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(0.3F, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.2F, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.updateSPI(0, deltaT);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.D);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I + error.D, output);

    pid.setSetpoint(5.0F);
    output = pid.updateSPI(0.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(5.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.5F, error.I); // (5.0 + 0.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(2.0F, output);

    output = pid.updateSPI(1.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(1.2F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(4.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.I); // 0.5 + (4.0 + 5.0) * 0.2 /2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(2.6F, output);

    output = pid.updateSPI(4.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.3F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.9F, error.I); // 1.4 + (1.0 + 4.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(2.2F, output);

    output = pid.updateSPI(7.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.8F, error.I); // 1.9 + (-2.0 + 1.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(1.2F, output);

    output = pid.updateSPI(6.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.I); // 1.8 + (-1.0 -2.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(1.2F, output);

    output = pid.updateSPI(5.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.I); // 1.5 + (0.0 - 1.0) * 0.2 /2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, output);

    output = pid.updateSPI(5.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.I); // 1.4 + (0.0 + 0.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, output);
}

void test_integration_on_off()
{
    PIDF pid(PIDF::PIDF_t { 0.2F, 0.3F, 0.0F, 0.0F, 0.0F });
    const float deltaT {1};

    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0.0F, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    PIDF::error_t error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.I); // 0.0 + (2.0 + 0.0) * 0.3 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(-0.7F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-0.9F, error.I); // -0.3 + (-2.0 - 2.0) * 0.3 / 2
    TEST_ASSERT_EQUAL_FLOAT(-1.3F, output);

    // Integration OFF
    pid.switchIntegrationOff();
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);

    output = pid.update(0.0F, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, output);

    // Integration back ON
    pid.switchIntegrationOn();
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);

    output = pid.update(0.0F, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, output);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.I); // 0.0 + (0.0 - 2.0) * 0.3 / 2

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.I); // - 0.3 + (0.0 - 2.0) * 0.3 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.P + error.I, output);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, output);

    output = pid.update(2.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-1.2F, error.I); // -0.6 + (-2.0 - 2.0) * 0.3 / 2
    TEST_ASSERT_EQUAL_FLOAT(-1.6F, output);

    pid.resetAll();
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousSetpoint());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpointDelta());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousMeasurement());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.D);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.S);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.K);
}

void test_integral_limit()
{
    PIDF pid(PIDF::PIDF_t { 0.2F, 0.3F, 0.0F, 0.0F, 0.0F });
    pid.setIntegralLimit(2.0F);
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
    PIDF pid(PIDF::PIDF_t { 0.2F, 0.3F, 0.0F, 0.0F, 0.0F });
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
    TEST_ASSERT_EQUAL_FLOAT(-0.5F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-1.4F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(0.1F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.02F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-1.48F, error.I);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(0.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(-0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-1.495F, error.I); // -1.48 + (0.0 - 1.0) * 0.3 / 2
    TEST_ASSERT_EQUAL_FLOAT(-1.495F, output);

    output = pid.update(0.0F, deltaT);
    error = pid.getError();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.P);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousError());
    TEST_ASSERT_EQUAL_FLOAT(-1.495F, error.I); // -1.495 + (0.0 + 0.0) * 0.3 / 2
    TEST_ASSERT_EQUAL_FLOAT(-1.495F, output);
}

void test_integral_saturation_negative()
{
    PIDF pid(PIDF::PIDF_t { 0.2F, 0.3F, 0.0F, 0.0F, 0.0F });
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
    RUN_TEST(test_P_controller);
    RUN_TEST(test_PI_controller);
    RUN_TEST(test_update_PI);
    RUN_TEST(test_integration_on_off);
    RUN_TEST(test_integral_limit);
    RUN_TEST(test_integral_saturation_positive);
    RUN_TEST(test_integral_saturation_negative);

    UNITY_END();
}
