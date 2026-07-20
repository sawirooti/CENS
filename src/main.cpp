#include <Arduino.h>
#include <math.h>

#include "Map.h"
#include "filter.h"
#include "MPU6050/MPU6050.h"
#include "VL53L0X/VL53L0X.h"
#include "AngleEstimator.h"
#include "Movement.h"

enum class WorkState
{
    CalcStart,
    Calibration,
    CourseCalculation,
    Rotation,
    ForwardMovement,
    Localization,
    Finish,
    Error
};

const float PI_VALUE = 3.14159265358979323846f;
const float DEG_TO_RAD_VALUE = PI_VALUE / 180.0f;
const float FINISH_RADIUS = 1.0f;
const float ANGLE_TOLERANCE_DEG = 3.0f;
const float COURSE_ERROR_LIMIT_DEG = 7.0f;
const float MAX_DISTANCE_CHANGE_MM = 100.0f;
const uint32_t CALIBRATION_TIME_MS = 1000;
const uint16_t CALIBRATION_RATE_HZ = 100;

Position start(X_START,
               Y_START,
               PSI_START * DEG_TO_RAD_VALUE);
Map karta;
CENS cens(karta);
KensKalmanFilter3D Kalman(start,
                          5.5,
                          2.0 * DEG_TO_RAD_VALUE,
                          0.01,
                          0.5 * DEG_TO_RAD_VALUE);
MPU6050 gyro;
VL53L0X dist;
AngleEstimator teta(PSI_START);
Movement Car;

static WorkState state = WorkState::Calibration;
static WorkState stateAfterCalibration = WorkState::CourseCalculation;
static Position position = start;

// Дальности до стен по направлениям +Y и +X в системе координат карты.
static float distanceFront = LENGTH - Y_START * DELTA;
static float distanceRight = WIDTH - X_START * DELTA;
static float targetAngle = PSI_START;
static float segmentLength = 0.0f;
static float segmentPassed = 0.0f;
static float filteredDistance = 0.0f;
static float previousDistance = 0.0f;
static float gyroZ = 0.0f;

static uint16_t rawDistance = 0;
static uint8_t distanceSampleCount = 0;
static uint32_t lastMicros = 0;
static uint32_t lastPrintMs = 0;
static uint32_t stateStartMs = 0;
static bool sensorsReady = false;

void setup()
{
    Serial.begin(115200);
    Serial.println("BOOT");

    Car.init_motors();
    Car.Stop();
    Serial.println("INIT motors: OK, command: STOP");
    Serial.print("INIT goal x/y coordinates: ");
    Serial.print(X_GOAL);
    Serial.print(" / ");
    Serial.println(Y_GOAL);

    const bool gyroStarted = gyro.begin();
    Serial.print("INIT MPU6050/MPU6500: ");
    Serial.println(gyroStarted ? "OK" : "ERROR");
    if (!gyroStarted)
    {
        Serial.print("INIT gyro status: ");
        Serial.println(gyro.lastError());
    }

    const bool distanceStarted = dist.begin();
    Serial.print("INIT VL53L0X: ");
    Serial.println(distanceStarted ? "OK" : "ERROR");
    if (!distanceStarted)
    {
        Serial.print("INIT range status: ");
        Serial.println(dist.lastError());
    }

    Serial.print("INIT Kalman: ");
    Serial.println(Kalman.isValid() ? "OK" : "ERROR");

    if (!gyroStarted || !distanceStarted || !Kalman.isValid())
    {
        state = WorkState::Error;
        Serial.println("STATE -> ERROR: initialization failed");
    }
    else
    {
        stateStartMs = millis();
        Serial.println("STATE -> CALIBRATION: initial stop");
    }

}

void loop()
{

    if (state == WorkState::Finish)
    {
        Car.Stop();
        if ((uint32_t)(millis() - lastPrintMs) >= 1000)
        {
            lastPrintMs = millis();
            Serial.print("FINISH filtered x/y coordinates: ");
            Serial.print(position.x);
            Serial.print(" / ");
            Serial.println(position.y);
        }
        delay(50);
        return;
    }

    if (state == WorkState::Error)
    {
        Car.Stop();
        if ((uint32_t)(millis() - lastPrintMs) >= 1000)
        {
            lastPrintMs = millis();
            Serial.println("ERROR: motors remain stopped");
        }
        delay(50);
        return;
    }

    if (!gyro.readGyroZDegreesPerSecond(&gyroZ))
    {
        Car.Stop();
        state = WorkState::Error;
        Serial.print("STATE -> ERROR: gyro read status ");
        Serial.println(gyro.lastError());
        return;
    }

    if (!dist.readDistanceMillimeters(&rawDistance))
    {
        Car.Stop();
        state = WorkState::Error;
        Serial.print("STATE -> ERROR: range read status ");
        Serial.println(dist.lastError());
        return;
    }

    filteredDistance = getMedianFilter((float)rawDistance);
    if (distanceSampleCount < WINDOW_SIZE)
    {
        ++distanceSampleCount;
        Serial.print("MEDIAN warmup: ");
        Serial.print(distanceSampleCount);
        Serial.print("/");
        Serial.print(WINDOW_SIZE);
        Serial.print(" raw mm=");
        Serial.println(rawDistance);

        if (distanceSampleCount == WINDOW_SIZE)
        {
            sensorsReady = true;
            stateStartMs = millis();
            Serial.println("MEDIAN ready");
        }
        delay(10);
        return;
    }

    if (!sensorsReady)
    {
        delay(10);
        return;
    }

    switch (state)
    {
    case WorkState::Calibration:
    {
        Car.Stop();

        if ((uint32_t)(millis() - lastPrintMs) >= 250)
        {
            lastPrintMs = millis();
            Serial.print("CALIBRATION elapsed/required ms: ");
            Serial.print((uint32_t)(millis() - stateStartMs));
            Serial.print(" / ");
            Serial.print(CALIBRATION_TIME_MS);
            Serial.print(" gyroZ dps: ");
            Serial.println(gyroZ);
        }

        if (teta.calibrate(gyroZ,
                           millis(),
                           CALIBRATION_TIME_MS,
                           CALIBRATION_RATE_HZ))
        {
            lastMicros = micros();
            state = stateAfterCalibration;
            Serial.println("CALIBRATION complete");

            if (state == WorkState::CourseCalculation)
            {
                Serial.println("STATE -> COURSE_CALCULATION");
            }
            else if (state == WorkState::ForwardMovement)
            {
                previousDistance = filteredDistance;
                segmentPassed = 0.0f;
                Car.Forward();
                Serial.println("STATE -> FORWARD_MOVEMENT, command: FORWARD");
            }
            else
            {
                Serial.println("STATE -> LOCALIZATION");
            }
        }
        break;
    }

    case WorkState::CourseCalculation:
    {
        Car.Stop();
        const float deltaX = X_GOAL - position.x;
        const float deltaY = Y_GOAL - position.y;
        const float remainingDistance = sqrtf(deltaX * deltaX + deltaY * deltaY);

        Serial.print("COURSE position x/y coordinates: ");
        Serial.print(position.x);
        Serial.print(" / ");
        Serial.println(position.y);
        Serial.print("COURSE delta x/y, remaining coordinates: ");
        Serial.print(deltaX);
        Serial.print(" / ");
        Serial.print(deltaY);
        Serial.print(" / ");
        Serial.println(remainingDistance);

        if (remainingDistance <= FINISH_RADIUS)
        {
            state = WorkState::Finish;
            Serial.println("STATE -> FINISH: inside target radius");
            break;
        }

        // Курс 0 градусов направлен по +Y, положительный угол - поворот влево.
        targetAngle = atan2f(-deltaX, deltaY) / DEG_TO_RAD_VALUE;
        targetAngle = fmodf(targetAngle, 360.0f);
        if (targetAngle < 0.0f) targetAngle += 360.0f;

        // Длина физического участка нужна в миллиметрах для дальномера.
        segmentLength = remainingDistance * DELTA * 0.5f;
        segmentPassed = 0.0f;

        float angleError = targetAngle - teta.getAngle();
        if (angleError > 180.0f) angleError -= 360.0f;
        if (angleError < -180.0f) angleError += 360.0f;

        Serial.print("COURSE current/target/error deg: ");
        Serial.print(teta.getAngle());
        Serial.print(" / ");
        Serial.print(targetAngle);
        Serial.print(" / ");
        Serial.println(angleError);
        Serial.print("COURSE half segment mm: ");
        Serial.println(segmentLength);

        if (fabsf(angleError) <= ANGLE_TOLERANCE_DEG)
        {
            previousDistance = filteredDistance;
            lastMicros = micros();
            Car.Forward();
            state = WorkState::ForwardMovement;
            Serial.println("STATE -> FORWARD_MOVEMENT: rotation not required");
        }
        else
        {
            lastMicros = micros();
            state = WorkState::Rotation;
            Serial.println("STATE -> ROTATION");
        }
        break;
    }

    case WorkState::Rotation:
    {
        const uint32_t currentMicros = micros();
        const float dt = (uint32_t)(currentMicros - lastMicros) / 1000000.0f;
        lastMicros = currentMicros;

        Serial.print("ROTATION gyroZ: ");
        Serial.println(gyroZ);

        teta.update(gyroZ, dt);
        targetAngle = 270;
        float angleError = targetAngle - teta.getAngle();
        if (angleError > 180.0f) angleError -= 360.0f;
        if (angleError < -180.0f) angleError += 360.0f;

        if ((uint32_t)(millis() - lastPrintMs) >= 250)
        {
            lastPrintMs = millis();
            Serial.print("ROTATION current/target/error deg: ");
            Serial.print(teta.getAngle());
            Serial.print(" / ");
            Serial.print(targetAngle);
            Serial.print(" / ");
            Serial.print(angleError);
            Serial.print(" | command: ");
            if (fabsf(angleError) <= ANGLE_TOLERANCE_DEG)
            {
                Serial.println("STOP");
            }
            else if (angleError > 0.0f)
            {
                Serial.println("TURN_LEFT");
            }
            else
            {
                Serial.println("TURN_RIGHT");
            }
        }

        if (fabsf(angleError) <= ANGLE_TOLERANCE_DEG)
        {
            Car.Stop();
            stateAfterCalibration = WorkState::ForwardMovement;
            state = WorkState::Calibration;
            stateStartMs = millis();
            Serial.println("ROTATION complete, command: STOP");
            Serial.println("STATE -> CALIBRATION after rotation");
        }
        else if (angleError > 0.0f)
        {
            Car.TurnLeft();
        }
        else
        {
            Car.TurnRight();
        }
        break;
    }

    case WorkState::ForwardMovement:
    {
        const uint32_t currentMicros = micros();
        const float dt = (uint32_t)(currentMicros - lastMicros) / 1000000.0f;
        lastMicros = currentMicros;
        teta.update(gyroZ, dt);

        float courseError = targetAngle - teta.getAngle();
        if (courseError > 180.0f) courseError -= 360.0f;
        if (courseError < -180.0f) courseError += 360.0f;

        const float distanceChange = previousDistance - filteredDistance;
        previousDistance = filteredDistance;

        if (fabsf(distanceChange) <= MAX_DISTANCE_CHANGE_MM)
        {
            const float angleRad = teta.getAngle() * DEG_TO_RAD_VALUE;
            const float passedX = -distanceChange * sinf(angleRad);
            const float passedY = distanceChange * cosf(angleRad);

            distanceRight -= passedX;
            distanceFront -= passedY;
            segmentPassed += distanceChange;

            if (distanceRight < 0.0f) distanceRight = 0.0f;
            if (distanceRight > WIDTH) distanceRight = WIDTH;
            if (distanceFront < 0.0f) distanceFront = 0.0f;
            if (distanceFront > LENGTH) distanceFront = LENGTH;
            if (segmentPassed < 0.0f) segmentPassed = 0.0f;
        }
        else
        {
            Serial.print("MOVE rejected range jump mm: ");
            Serial.println(distanceChange);
        }

        if ((uint32_t)(millis() - lastPrintMs) >= 250)
        {
            lastPrintMs = millis();
            Serial.print("MOVE raw/median mm: ");
            Serial.print(rawDistance);
            Serial.print(" / ");
            Serial.print(filteredDistance);
            Serial.print(" | passed/target mm: ");
            Serial.print(segmentPassed);
            Serial.print(" / ");
            Serial.print(segmentLength);
            Serial.print(" | angle/error deg: ");
            Serial.print(teta.getAngle());
            Serial.print(" / ");
            Serial.print(courseError);
            Serial.print(" | virtual front/right mm: ");
            Serial.print(distanceFront);
            Serial.print(" / ");
            Serial.println(distanceRight);
        }

        if (fabsf(courseError) > COURSE_ERROR_LIMIT_DEG)
        {
            Car.Stop();
            stateAfterCalibration = WorkState::Localization;
            state = WorkState::Calibration;
            stateStartMs = millis();
            Serial.println("MOVE course error exceeded, command: STOP");
            Serial.println("STATE -> CALIBRATION, then LOCALIZATION");
        }
        else if (segmentPassed >= segmentLength)
        {
            Car.Stop();
            stateAfterCalibration = WorkState::Localization;
            state = WorkState::Calibration;
            stateStartMs = millis();
            Serial.println("MOVE half segment complete, command: STOP");
            Serial.println("STATE -> CALIBRATION, then LOCALIZATION");
        }
        else
        {
            Car.Forward();
        }
        break;
    }

    case WorkState::Localization:
    {
        Car.Stop();
        const float censInputFront = distanceFront;
        const float censInputRight = distanceRight;
        Position censPosition = cens.calculatePosition(censInputFront,
                                                       censInputRight);
        if (!censPosition.valid)
        {
            state = WorkState::Error;
            Serial.println("STATE -> ERROR: CENS invalid result");
            break;
        }

        censPosition.theta = teta.getAngle() * DEG_TO_RAD_VALUE;
        if (!Kalman.predict() || !Kalman.update(censPosition))
        {
            state = WorkState::Error;
            Serial.println("STATE -> ERROR: Kalman predict/update failed");
            break;
        }

        position = Kalman.getState();
        if (position.theta < 0)
        {
            teta.setAngle(position.theta * RAD_TO_DEG + 360);
        }
        else
        {
            teta.setAngle(position.theta * RAD_TO_DEG);
        }
        // position = censPosition;

        Serial.print("LOCALIZATION input front/right mm: ");
        Serial.print(censInputFront);
        Serial.print(" / ");
        Serial.println(censInputRight);
        Serial.print("LOCALIZATION CENS x/y coordinates: ");
        Serial.print(censPosition.x);
        Serial.print(" / ");
        Serial.println(censPosition.y);
        Serial.print("LOCALIZATION Kalman x/y coordinates: ");
        Serial.print(position.x);
        Serial.print(" / ");
        Serial.println(position.y);
        Serial.print("LOCALIZATION theta / Kalman_theta: ");
        Serial.print(censPosition.theta * RAD_TO_DEG);
        Serial.print(" / ");
        Serial.println(position.theta * RAD_TO_DEG);
        Serial.print("LOCALIZATION Kalman-CENS error: ");
        Serial.print(position.x - censPosition.x);
        Serial.print(" / ");
        Serial.println(position.y - censPosition.y);
        Serial.println("LOCALIZATION virtual distances preserved from odometry");

        state = WorkState::CourseCalculation;
        Serial.println("STATE -> COURSE_CALCULATION");
        break;
    }

    case WorkState::Finish:
    case WorkState::Error:
        break;
    }

    delay(10);
}
