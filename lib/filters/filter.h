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

