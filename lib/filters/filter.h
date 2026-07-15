#pragma once

#include <Arduino.h>
#include <math.h>

#define WINDOW_SIZE 3

float getMedianFilter(float newVal) {
    static float buffer[WINDOW_SIZE] = {0};
    static int index = 0;

    // Записываем новое значение в круговой буфер
    buffer[index] = newVal;
    index = (index + 1) % WINDOW_SIZE;

    // Копируем во временный массив для сортировки
    float sorted[WINDOW_SIZE];
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sorted[i] = buffer[i];
    }

    // Простая сортировка вставками (быстрее для малых массивов)
    for (int i = 1; i < WINDOW_SIZE; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j = j - 1;
        }
        sorted[j + 1] = key;
    }

    // Возвращаем центральный элемент
    return sorted[WINDOW_SIZE / 2];
}

// ============================================================
// Состояние машинки
// ============================================================

struct RobotState {
    double x = 0.0;      // mm
    double y = 0.0;      // mm
    double theta = 0.0;  // rad

    RobotState()
        : x(0.0),
          y(0.0),
          theta(0.0) {
    }

    RobotState(
        double xValue,
        double yValue,
        double thetaValue)
        : x(xValue),
          y(yValue),
          theta(thetaValue) {
    }
};

// ============================================================
// Простой вектор из трёх элементов
// ============================================================

struct Vector3 {
    double data[3] = {0.0, 0.0, 0.0};

    double& operator[](int index) {
        return data[index];
    }

    double operator[](int index) const {
        return data[index];
    }
};

// ============================================================
// Матрица 3x3
// ============================================================

struct Matrix3x3 {
    double data[3][3] = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    static Matrix3x3 identity() {
        Matrix3x3 result;

        result.data[0][0] = 1.0;
        result.data[1][1] = 1.0;
        result.data[2][2] = 1.0;

        return result;
    }

    Matrix3x3 operator+(
        const Matrix3x3& other) const {

        Matrix3x3 result;

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result.data[row][column] =
                    data[row][column] +
                    other.data[row][column];
            }
        }

        return result;
    }

    Matrix3x3 operator-(
        const Matrix3x3& other) const {

        Matrix3x3 result;

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result.data[row][column] =
                    data[row][column] -
                    other.data[row][column];
            }
        }

        return result;
    }

    Matrix3x3 operator*(
        const Matrix3x3& other) const {

        Matrix3x3 result;

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                for (int k = 0; k < 3; ++k) {
                    result.data[row][column] +=
                        data[row][k] *
                        other.data[k][column];
                }
            }
        }

        return result;
    }

    Vector3 operator*(
        const Vector3& vector) const {

        Vector3 result;

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result[row] +=
                    data[row][column] *
                    vector[column];
            }
        }

        return result;
    }

    Matrix3x3 transpose() const {
        Matrix3x3 result;

        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result.data[row][column] =
                    data[column][row];
            }
        }

        return result;
    }

    bool inverse(Matrix3x3& result) const {
        const double determinant =
            data[0][0] *
                (data[1][1] * data[2][2] -
                 data[1][2] * data[2][1])
            -
            data[0][1] *
                (data[1][0] * data[2][2] -
                 data[1][2] * data[2][0])
            +
            data[0][2] *
                (data[1][0] * data[2][1] -
                 data[1][1] * data[2][0]);

        if (!isfinite(determinant) ||
            fabs(determinant) < 1e-12) {
            return false;
        }

        const double inverseDeterminant =
            1.0 / determinant;

        result = Matrix3x3{};

        result.data[0][0] =
            (data[1][1] * data[2][2] -
             data[1][2] * data[2][1]) *
            inverseDeterminant;

        result.data[0][1] =
            (data[0][2] * data[2][1] -
             data[0][1] * data[2][2]) *
            inverseDeterminant;

        result.data[0][2] =
            (data[0][1] * data[1][2] -
             data[0][2] * data[1][1]) *
            inverseDeterminant;

        result.data[1][0] =
            (data[1][2] * data[2][0] -
             data[1][0] * data[2][2]) *
            inverseDeterminant;

        result.data[1][1] =
            (data[0][0] * data[2][2] -
             data[0][2] * data[2][0]) *
            inverseDeterminant;

        result.data[1][2] =
            (data[0][2] * data[1][0] -
             data[0][0] * data[1][2]) *
            inverseDeterminant;

        result.data[2][0] =
            (data[1][0] * data[2][1] -
             data[1][1] * data[2][0]) *
            inverseDeterminant;

        result.data[2][1] =
            (data[0][1] * data[2][0] -
             data[0][0] * data[2][1]) *
            inverseDeterminant;

        result.data[2][2] =
            (data[0][0] * data[1][1] -
             data[0][1] * data[1][0]) *
            inverseDeterminant;

        return true;
    }

    void symmetrize() {
        for (int row = 0; row < 3; ++row) {
            for (int column = row + 1;
                 column < 3;
                 ++column) {

                const double average =
                    0.5 *
                    (data[row][column] +
                     data[column][row]);

                data[row][column] = average;
                data[column][row] = average;
            }
        }
    }
};

// ============================================================
// Фильтр Калмана
// ============================================================

class KensKalmanFilter3D {
public:
    KensKalmanFilter3D(
        const RobotState& initialState,
        double sigmaQPosition,
        double sigmaQAngle,
        double sigmaRPosition,
        double sigmaRAngle)
        : state_(initialState) {

        valid_ =
            isValidSigmaNonNegative(sigmaQPosition) &&
            isValidSigmaNonNegative(sigmaQAngle) &&
            isValidSigmaPositive(sigmaRPosition) &&
            isValidSigmaPositive(sigmaRAngle) &&
            isValidState(initialState);

        if (!valid_) {
            return;
        }

        state_.theta =
            normalizeAngle(state_.theta);

        // Начальная стандартная неопределённость.
        constexpr double initialSigmaPosition =
            100.0; // mm

        constexpr double initialSigmaAngle =
            0.05; // rad

        // P — начальная ковариация оценки.
        P_.data[0][0] =
            square(initialSigmaPosition);

        P_.data[1][1] =
            square(initialSigmaPosition);

        P_.data[2][2] =
            square(initialSigmaAngle);

        // Q — шум модели движения за один шаг.
        Q_.data[0][0] =
            square(sigmaQPosition);

        Q_.data[1][1] =
            square(sigmaQPosition);

        Q_.data[2][2] =
            square(sigmaQAngle);

        // R — шум измерений КЭНС.
        R_.data[0][0] =
            square(sigmaRPosition);

        R_.data[1][1] =
            square(sigmaRPosition);

        R_.data[2][2] =
            square(sigmaRAngle);
    }

    bool isValid() const {
        return valid_;
    }

    bool predict() {
        if (!valid_) {
            return false;
        }

        // F = I:
        // state не изменяется,
        // неопределённость увеличивается.
        P_ = P_ + Q_;

        return true;
    }

    bool update(const RobotState& measurement) {
        if (!valid_ || !isValidState(measurement)) {
            return false;
        }

        // H = I, КЭНС напрямую измеряет x, y и theta.

        // 1. Инновация:
        // innovation = measurement - prediction
        Vector3 innovation;

        innovation[0] =
            measurement.x - state_.x;

        innovation[1] =
            measurement.y - state_.y;

        innovation[2] =
            normalizeAngle(
                measurement.theta -
                state_.theta);

        // 2. Ковариация инновации:
        // S = HPH^T + R
        //
        // При H = I:
        // S = P + R
        const Matrix3x3 S =
            P_ + R_;

        Matrix3x3 inverseS;

        if (!S.inverse(inverseS)) {
            return false;
        }

        // 3. Коэффициент Калмана:
        // K = PH^T S^-1
        //
        // При H = I:
        // K = P S^-1
        const Matrix3x3 K =
            P_ * inverseS;

        // 4. Коррекция состояния:
        // state = state + K * innovation
        const Vector3 correction =
            K * innovation;

        state_.x += correction[0];
        state_.y += correction[1];

        state_.theta =
            normalizeAngle(
                state_.theta +
                correction[2]);

        // 5. Форма Джозефа:
        //
        // P = (I - KH)P(I - KH)^T + KRK^T
        //
        // При H = I:
        // P = (I - K)P(I - K)^T + KRK^T

        const Matrix3x3 IminusK =
            Matrix3x3::identity() - K;

        P_ =
            IminusK *
            P_ *
            IminusK.transpose()
            +
            K *
            R_ *
            K.transpose();

        P_.symmetrize();

        if (!isValidState(state_)) {
            valid_ = false;
            return false;
        }

        return true;
    }

    RobotState getState() const {
        return state_;
    }

private:
    static double square(double value) {
        return value * value;
    }

    static double normalizeAngle(double angle) {
        return atan2(
            sin(angle),
            cos(angle));
    }

    static bool isValidSigmaPositive(double value) {
        return isfinite(value) && value > 0.0;
    }

    static bool isValidSigmaNonNegative(double value) {
        return isfinite(value) && value >= 0.0;
    }

    static bool isValidState(
        const RobotState& state) {

        return
            isfinite(state.x) &&
            isfinite(state.y) &&
            isfinite(state.theta);
    }

    bool valid_ = false;

    RobotState state_{};

    Matrix3x3 P_{};
    Matrix3x3 Q_{};
    Matrix3x3 R_{};
};