#pragma once

#include <Arduino.h>
#include <math.h>
#include "CENS.h"

#define WINDOW_SIZE 3

float getMedianFilter(float newVal)
{
    static float buffer[WINDOW_SIZE] = {0};
    static int index = 0;
    
    // Записываем новое значение в круговой буфер
    buffer[index] = newVal;
    index = (index + 1) % WINDOW_SIZE;
    
    // Копируем во временный массив для сортировки
    float sorted[WINDOW_SIZE];
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        sorted[i] = buffer[i];
    }
    
    // Простая сортировка вставками (быстрее для малых массивов)
    for (int i = 1; i < WINDOW_SIZE; i++)
    {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key)
        {
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

// ============================================================
// Простой вектор из трёх элементов
// ============================================================

struct Vector3
{
    double data[3] = {0.0, 0.0, 0.0};
    
    double &operator[](int index)
    {
        return data[index];
    }
    
    double operator[](int index) const
    {
        return data[index];
    }
};

// ============================================================
// Матрица 3x3
// ============================================================

struct Matrix3x3
{
    double data[3][3] = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};
    
    static Matrix3x3 identity()
    {
        Matrix3x3 result;
        
        result.data[0][0] = 1.0;
        result.data[1][1] = 1.0;
        result.data[2][2] = 1.0;
        
        return result;
    }
    
    Matrix3x3 operator+(
        const Matrix3x3 &other) const
    {
        
        Matrix3x3 result;
        
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                result.data[row][column] =
                    data[row][column] +
                    other.data[row][column];
            }
        }
        
        return result;
    }
    
    Matrix3x3 operator-(
        const Matrix3x3 &other) const
    {
        
        Matrix3x3 result;
        
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                result.data[row][column] =
                    data[row][column] -
                    other.data[row][column];
            }
        }
        
        return result;
    }
    
    Matrix3x3 operator*(
        const Matrix3x3 &other) const
    {
        
        Matrix3x3 result;
        
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                for (int k = 0; k < 3; ++k)
                {
                    result.data[row][column] +=
                        data[row][k] *
                        other.data[k][column];
                }
            }
        }
        
        return result;
    }
    
    Vector3 operator*(
        const Vector3 &vector) const
    {
        
        Vector3 result;
        
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                result[row] +=
                    data[row][column] *
                    vector[column];
            }
        }
        
        return result;
    }
    
    Matrix3x3 transpose() const
    {
        Matrix3x3 result;
        
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                result.data[row][column] =
                    data[column][row];
            }
        }
        
        return result;
    }
    
    bool inverse(Matrix3x3 &result) const
    {
        const double determinant =
            data[0][0] *
                (data[1][1] * data[2][2] -
                 data[1][2] * data[2][1]) -
            data[0][1] *
                (data[1][0] * data[2][2] -
                 data[1][2] * data[2][0]) +
            data[0][2] *
                (data[1][0] * data[2][1] -
                 data[1][1] * data[2][0]);
        
        if (!isfinite(determinant) ||
            fabs(determinant) < 1e-12)
        {
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
    
    void symmetrize()
    {
        for (int row = 0; row < 3; ++row)
        {
            for (int column = row + 1;
                 column < 3;
                 ++column)
            {
                
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

class KensKalmanFilter3D
{
public:
    KensKalmanFilter3D(
        const Position &initialState,
        double sigmaQPosition,
        double sigmaQAngle,
        double sigmaRPosition,
        double sigmaRAngle)
        : state_(initialState)
    {
        
        valid_ =
            isValidSigmaNonNegative(sigmaQPosition) &&
            isValidSigmaNonNegative(sigmaQAngle) &&
            isValidSigmaPositive(sigmaRPosition) &&
            isValidSigmaPositive(sigmaRAngle) &&
            isValidState(initialState);
        
        if (!valid_)
        {
            return;
        }
        
        state_.theta =
            normalizeAngle(state_.theta);
        
        // Начальная стандартная неопределённость.
        constexpr double initialSigmaPosition =
            10.0; // координаты сетки
        
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
    
    bool isValid() const
    {
        return valid_;
    }
    
    bool predict(const Position &delta)
    {
        if (!valid_ ||
            !isfinite(delta.x) ||
            !isfinite(delta.y) ||
            !isfinite(delta.theta))
        {
            return false;
        }
        
        state_.x += delta.x;
        state_.y += delta.y;
        
        state_.theta =
            normalizeAngle(
                state_.theta + delta.theta);
        
        P_ = P_ + Q_;
        
        return true;
    }
    
    bool update(const Position &measurement)
    {
        if (!valid_ ||
            !isfinite(measurement.x) ||
            !isfinite(measurement.y))
        {
            return false;
        }
        
        const double oldPx = P_.data[0][0];
        const double oldPy = P_.data[1][1];
        
        const double sx =
            oldPx + R_.data[0][0];
        
        const double sy =
            oldPy + R_.data[1][1];
        
        if (sx <= 0.0 || sy <= 0.0)
        {
            return false;
        }
        
        const double kx = oldPx / sx;
        const double ky = oldPy / sy;
        
        state_.x +=
            kx * (measurement.x - state_.x);
        
        state_.y +=
            ky * (measurement.y - state_.y);
        
        // Форма Джозефа для X.
        P_.data[0][0] =
            (1.0 - kx) *
                (1.0 - kx) *
                oldPx +
            kx * kx *
                R_.data[0][0];
        
        // Форма Джозефа для Y.
        P_.data[1][1] =
            (1.0 - ky) *
                (1.0 - ky) *
                oldPy +
            ky * ky *
                R_.data[1][1];
        
        // theta здесь не обновляем:
        // его источник — тот же гироскоп,
        // который использовался в predict().
        
        return true;
    }
    
    Position getState() const
    {
        return state_;
    }
    
    void setState(const Position &newState)
    {
        if (!isValidState(newState))
        {
            valid_ = false;
            return;
        }
        
        state_ = newState;
        state_.theta =
            normalizeAngle(state_.theta);
    }
    
    static double normalizeAngle(double angle)
    {
        return atan2(
            sin(angle),
            cos(angle));
    }
private:
    static double square(double value)
    {
        return value * value;
    }
    
    static bool isValidSigmaPositive(double value)
    {
        return isfinite(value) && value > 0.0;
    }
    
    static bool isValidSigmaNonNegative(double value)
    {
        return isfinite(value) && value >= 0.0;
    }
    
    static bool isValidState(
        const Position &state)
    {
        
        return isfinite(state.x) &&
               isfinite(state.y) &&
               isfinite(state.theta);
    }
    
    bool valid_ = false;
    
    Position state_{};
    
    Matrix3x3 P_{};
    Matrix3x3 Q_{};
    Matrix3x3 R_{};
};
