#pragma once

#include <math.h>
#include <Arduino.h>

template <typename T>
class Median3 {
   public:
    T filter(T val) {
        if (++_i >= 3) _i = 0;
        _buf[_i] = val;
        return getMedian(_buf[0], _buf[1], _buf[2]);
    }

    void init(T val) {
        _buf[0] = _buf[1] = _buf[2] = val;
    }

    static inline T getMedian(T a, T b, T c) {
        return (a < b) ? ((b < c) ? b : ((c < a) ? a : c)) : ((a < c) ? a : ((c < b) ? b : c));
    }

   private:
    T _buf[3] = {};
    uint8_t _i = 0;
};

class KSimple {
   public:
    void init(float val) {
        _last_est = val;
    }

    // стд. отклонение шума
    void config(float stddev_noise) {
        _noise = stddev_noise;
        _err_est = stddev_noise;
    }

    // k - коэффициент силы фильтра
    float filter(float val, float k) {
        float gain = _err_est / (_err_est + _noise);
        float est = _last_est + gain * (val - _last_est);
        _err_est = (1.0 - gain) * _err_est + fabs(_last_est - est) * k;
        _last_est = est;
        return est;
    }

   private:
    float _noise = 0;
    float _err_est = 0;
    float _last_est = 0;
};

template <typename T>
class PeakFilter {
   public:
    void init(T val) {
        _filt = _raw = val;
        _count = 0;
    }

    // порог, кол-во значений
    void config(T thrsh, uint8_t N) {
        _thrsh = thrsh;
        _N = N ? N : 1;
    }

    T filter(T val) {
        _raw = val;

        T diff = (val > _filt) ? (val - _filt) : (_filt - val);
        if (diff > _thrsh) {
            if (++_count >= _N) {
                _filt = val;
                _count = 0;
            }
        } else {
            _filt = val;
            _count = 0;
        }

        return _filt;
    }

    T filtered() {
        return _filt;
    }

   private:
    T _filt = 0;
    T _raw = 0;
    T _thrsh = 100;
    uint8_t _N = 3;
    uint8_t _count = 0;
};

class MinQuad {
   public:
    // тип вычислений, тип массивов
    template <typename Ts, typename Tx, typename Ty>
    void compute(const Tx *x_array, const Ty *y_array, size_t len) {
        if (len < 2) {
            a = b = delta = 0;
            return;
        }
        Ts sumX = 0, sumY = 0, sumX2 = 0, sumXY = 0;

        for (size_t i = 0; i < len; i++) {
            sumX += x_array[i];
            sumY += y_array[i];
            sumX2 += (Ts)x_array[i] * x_array[i];
            sumXY += (Ts)x_array[i] * y_array[i];
        }

        Ts den = len * sumX2 - sumX * sumX;
        if (!den) {
            a = b = delta = 0;
            return;
        }
        a = (float)(len * sumXY - sumX * sumY) / den;
        b = (float)(sumY - a * sumX) / len;
        delta = a * (x_array[len - 1] - x_array[0]);
    }

    // получить коэффициент А
    float getA() {
        return a;
    }

    // получить коэффициент В
    float getB() {
        return b;
    }

    // получить аппроксимированное изменение
    float getDelta() {
        return delta;
    }

   private:
    float a, b, delta;
};

/*
Примеры взаимодействия со всеми фильтрами:

Median3<int> med;

void setup() {
    Serial.begin(115200);
    med.init(analogRead(0));
}

void loop() {
    Serial.println(med.filter(analogRead(0)));

    delay(50);  // или таймер на millis()
}

KSimple ks;

void setup() {
    Serial.begin(115200);
    ks.init(analogRead(0));
    ks.config(3);
}

void loop() {
    Serial.println(ks.filter(analogRead(0), 0.3));

    delay(50);  // или таймер на millis()
}

PeakFilter<int> peak;

void setup() {
    Serial.begin(115200);
    peak.config(100, 3);
    peak.init(analogRead(0));
}

void loop() {
    Serial.println(peak.filter(analogRead(0)));

    delay(50);  // или таймер на millis()
}

void setup() {
    Serial.begin(115200);

    int x_array[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    int y_array[] = {1, 5, 2, 8, 3, 9, 10, 5, 15, 12};

    MinQuad mq;
    mq.compute<long>(x_array, y_array, 10);

    // Уравнение вида у = A*x + B
    Serial.println(mq.getA());      // получить коэффициент А
    Serial.println(mq.getB());      // получить коэффициент В
    Serial.println(mq.getDelta());  // получить изменение (аппроксимированное)
    // A=1.19, B=0.47, D=10.69
}

void loop() {
}
*/

