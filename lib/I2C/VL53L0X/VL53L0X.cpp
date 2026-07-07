#include "VL53L0X.h"

bool VL53L0X::begin()
{
    // В проекте используется стандартная шина Wire и фиксированный адрес 0x29.
    _i2c.setWire(&Wire);
    _i2c.begin();
    _distanceMm = 0;
    _didTimeout = false;
    _initialized = false;

    if (!initializeSensor())
    {
        return false;
    }

    _initialized = true;
    return true;
}

bool VL53L0X::readDistanceMillimeters(uint16_t *distanceMm)
{
    if (!_initialized || distanceMm == 0)
    {
        // Нельзя читать датчик до успешного begin().
        _lastStatus = 4;
        return false;
    }

    if (!startSingleMeasurement())
    {
        return false;
    }

    // Измерение выполняется именно здесь — в момент вызова чтения.
    _distanceMm = readRangeMillimeters();
    *distanceMm = _distanceMm;

    return !timeoutOccurred();
}

bool VL53L0X::update()
{
    return readDistanceMillimeters(&_distanceMm);
}

uint16_t VL53L0X::getDistanceMillimeters() const
{
    return _distanceMm;
}

bool VL53L0X::isInitialized() const
{
    return _initialized;
}

bool VL53L0X::initializeSensor()
{
  // Проверяем модель датчика перед записью настроек.
  if (readRegister(REG_IDENTIFICATION_MODEL_ID) != EXPECTED_MODEL_ID) { return false; }

  // Включаем режим 2.8 В для линий I2C.
  writeRegister(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
    readRegister(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) | 0x01);

  // Переводим I2C датчика в стандартный режим.
  writeRegister(0x88, 0x00);

  // Сохраняем внутреннюю stop-переменную, она нужна для одиночного измерения.
  writeRegister(0x80, 0x01);
  writeRegister(0xFF, 0x01);
  writeRegister(0x00, 0x00);
  _stopVariable = readRegister(0x91);
  writeRegister(0x00, 0x01);
  writeRegister(0xFF, 0x00);
  writeRegister(0x80, 0x00);

  writeRegister(REG_MSRC_CONFIG_CONTROL, readRegister(REG_MSRC_CONFIG_CONTROL) | 0x12);

  // Минимальный уровень сигнала для final range: 0.25 MCPS в формате Q9.7.
  writeRegister16(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);

  // На время первичной настройки включаем все шаги измерительной последовательности.
  writeRegister(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // Конец этапа DataInit из API ST.

  // Начало статической настройки датчика.

  uint8_t spadCount;
  bool spadTypeIsAperture;
  if (!getSpadInfo(&spadCount, &spadTypeIsAperture)) { return false; }

  // Карта SPAD показывает, какие фотодиоды можно использовать как опорные.
  // Берем ее напрямую из регистров GLOBAL_CONFIG_SPAD_ENABLES_REF_0..5.
  uint8_t refSpadMap[6];
  readRegisters(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadMap, 6);

  // Настройка опорных SPAD. Значения из NVM считаем корректными.

  writeRegister(0xFF, 0x01);
  writeRegister(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeRegister(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeRegister(0xFF, 0x00);
  writeRegister(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  // Для aperture SPAD первые 12 элементов пропускаются.
  uint8_t firstSpadToEnable = spadTypeIsAperture ? 12 : 0;
  uint8_t spadsEnabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < firstSpadToEnable || spadsEnabled == spadCount)
    {
      // Этот SPAD ниже первого разрешенного или нужное количество уже включено.
      refSpadMap[i / 8] &= ~(1 << (i % 8));
    }
    else if ((refSpadMap[i / 8] >> (i % 8)) & 0x1)
    {
      spadsEnabled++;
    }
  }

  writeRegisters(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadMap, 6);

  // Опорные SPAD настроены.

  // Загрузка стандартных tuning-настроек из vl53l0x_tuning.h.

  writeRegister(0xFF, 0x01);
  writeRegister(0x00, 0x00);

  writeRegister(0xFF, 0x00);
  writeRegister(0x09, 0x00);
  writeRegister(0x10, 0x00);
  writeRegister(0x11, 0x00);

  writeRegister(0x24, 0x01);
  writeRegister(0x25, 0xFF);
  writeRegister(0x75, 0x00);

  writeRegister(0xFF, 0x01);
  writeRegister(0x4E, 0x2C);
  writeRegister(0x48, 0x00);
  writeRegister(0x30, 0x20);

  writeRegister(0xFF, 0x00);
  writeRegister(0x30, 0x09);
  writeRegister(0x54, 0x00);
  writeRegister(0x31, 0x04);
  writeRegister(0x32, 0x03);
  writeRegister(0x40, 0x83);
  writeRegister(0x46, 0x25);
  writeRegister(0x60, 0x00);
  writeRegister(0x27, 0x00);
  writeRegister(0x50, 0x06);
  writeRegister(0x51, 0x00);
  writeRegister(0x52, 0x96);
  writeRegister(0x56, 0x08);
  writeRegister(0x57, 0x30);
  writeRegister(0x61, 0x00);
  writeRegister(0x62, 0x00);
  writeRegister(0x64, 0x00);
  writeRegister(0x65, 0x00);
  writeRegister(0x66, 0xA0);

  writeRegister(0xFF, 0x01);
  writeRegister(0x22, 0x32);
  writeRegister(0x47, 0x14);
  writeRegister(0x49, 0xFF);
  writeRegister(0x4A, 0x00);

  writeRegister(0xFF, 0x00);
  writeRegister(0x7A, 0x0A);
  writeRegister(0x7B, 0x00);
  writeRegister(0x78, 0x21);

  writeRegister(0xFF, 0x01);
  writeRegister(0x23, 0x34);
  writeRegister(0x42, 0x00);
  writeRegister(0x44, 0xFF);
  writeRegister(0x45, 0x26);
  writeRegister(0x46, 0x05);
  writeRegister(0x40, 0x40);
  writeRegister(0x0E, 0x06);
  writeRegister(0x20, 0x1A);
  writeRegister(0x43, 0x40);

  writeRegister(0xFF, 0x00);
  writeRegister(0x34, 0x03);
  writeRegister(0x35, 0x44);

  writeRegister(0xFF, 0x01);
  writeRegister(0x31, 0x04);
  writeRegister(0x4B, 0x09);
  writeRegister(0x4C, 0x05);
  writeRegister(0x4D, 0x04);

  writeRegister(0xFF, 0x00);
  writeRegister(0x44, 0x00);
  writeRegister(0x45, 0x20);
  writeRegister(0x47, 0x08);
  writeRegister(0x48, 0x28);
  writeRegister(0x67, 0x00);
  writeRegister(0x70, 0x04);
  writeRegister(0x71, 0x01);
  writeRegister(0x72, 0xFE);
  writeRegister(0x76, 0x00);
  writeRegister(0x77, 0x00);

  writeRegister(0xFF, 0x01);
  writeRegister(0x0D, 0x01);

  writeRegister(0xFF, 0x00);
  writeRegister(0x80, 0x01);
  writeRegister(0x01, 0xF8);

  writeRegister(0xFF, 0x01);
  writeRegister(0x8E, 0x01);
  writeRegister(0x00, 0x01);
  writeRegister(0xFF, 0x00);
  writeRegister(0x80, 0x00);

  // Tuning-настройки загружены.

  // Настраиваем прерывание как сигнал "новое измерение готово".

  writeRegister(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeRegister(REG_GPIO_HV_MUX_ACTIVE_HIGH, readRegister(REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // активный низкий уровень
  writeRegister(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

  // Оставляем только нужные шаги измерения и задаем базовый бюджет 33 мс.
  writeRegister(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
  setMeasurementTimingBudget(33000);

  // Статическая настройка завершена.

  // Выполняем обязательные внутренние калибровки: VHV и phase calibration.

  writeRegister(REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return false; }

  writeRegister(REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return false; }

  // Возвращаем рабочую конфигурацию последовательности после калибровки.
  writeRegister(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

  return true;
}

// Запись 8-битного регистра.
void VL53L0X::writeRegister(uint8_t reg, uint8_t value)
{
  _lastStatus = _i2c.writeRegister(ADDRESS, reg, value);
}

// Запись 16-битного регистра, старший байт передается первым.
void VL53L0X::writeRegister16(uint8_t reg, uint16_t value)
{
  uint8_t data[2] = {
    (uint8_t)(value >> 8),
    (uint8_t)(value)
  };
  _lastStatus = _i2c.writeRegisters(ADDRESS, reg, data, 2);
}

// Чтение 8-битного регистра.
uint8_t VL53L0X::readRegister(uint8_t reg)
{
  uint8_t value = 0;
  _lastStatus = _i2c.readRegister(ADDRESS, reg, &value);
  return value;
}

// Чтение 16-битного регистра, старший байт приходит первым.
uint16_t VL53L0X::readRegister16(uint8_t reg)
{
  uint8_t data[2] = {0, 0};
  _lastStatus = _i2c.readRegisters(ADDRESS, reg, data, 2);
  uint16_t value = ((uint16_t)data[0] << 8) | data[1];
  return value;
}

// Запись нескольких байт подряд, начиная с указанного регистра.
void VL53L0X::writeRegisters(uint8_t reg, uint8_t const * src, uint8_t count)
{
  _lastStatus = _i2c.writeRegisters(ADDRESS, reg, src, count);
}

// Чтение нескольких байт подряд, начиная с указанного регистра.
void VL53L0X::readRegisters(uint8_t reg, uint8_t * dst, uint8_t count)
{
  _lastStatus = _i2c.readRegisters(ADDRESS, reg, dst, count);
}

// Задает бюджет времени на одно измерение в микросекундах.
// Чем больше бюджет, тем стабильнее измерение, но тем ниже максимальная частота.
// Основано на VL53L0X_set_measurement_timing_budget_micro_seconds().
bool VL53L0X::setMeasurementTimingBudget(uint32_t budgetUs)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t usedBudgetUs = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    usedBudgetUs += (timeouts.msrcDssTccUs + TccOverhead);
  }

  if (enables.dss)
  {
    usedBudgetUs += 2 * (timeouts.msrcDssTccUs + DssOverhead);
  }
  else if (enables.msrc)
  {
    usedBudgetUs += (timeouts.msrcDssTccUs + MsrcOverhead);
  }

  if (enables.preRange)
  {
    usedBudgetUs += (timeouts.preRangeUs + PreRangeOverhead);
  }

  if (enables.finalRange)
  {
    usedBudgetUs += FinalRangeOverhead;

    // Таймаут final range получает оставшееся время после остальных шагов.

    if (usedBudgetUs > budgetUs)
    {
      // Бюджета не хватило даже на обязательные шаги.
      return false;
    }

    uint32_t finalRangeTimeoutUs = budgetUs - usedBudgetUs;

    // Переводим оставшееся время в MCLK для final range.
    uint32_t finalRangeTimeoutMclks =
      timeoutMicrosecondsToMclks(finalRangeTimeoutUs,
                                 timeouts.finalRangeVcselPeriodPclks);

    if (enables.preRange)
    {
      // Если был pre-range, его таймаут добавляется к final range.
      finalRangeTimeoutMclks += timeouts.preRangeMclks;
    }

    writeRegister16(REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(finalRangeTimeoutMclks));
  }
  return true;
}

// Запускает одно измерение дальности.
// Основано на VL53L0X_PerformSingleRangingMeasurement().
bool VL53L0X::startSingleMeasurement()
{
  writeRegister(0x80, 0x01);
  writeRegister(0xFF, 0x01);
  writeRegister(0x00, 0x00);
  writeRegister(0x91, _stopVariable);
  writeRegister(0x00, 0x01);
  writeRegister(0xFF, 0x00);
  writeRegister(0x80, 0x00);

  writeRegister(REG_SYSRANGE_START, 0x01); // одиночное измерение

  startTimeout();
  while (readRegister(REG_SYSRANGE_START) & 0x01)
  {
    // Ждем, пока датчик примет команду запуска.
    if (checkTimeoutExpired())
    {
      _didTimeout = true;
      return false;
    }
  }

  return _lastStatus == 0;
}

// Возвращает дальность в миллиметрах после запуска одиночного измерения.
uint16_t VL53L0X::readRangeMillimeters()
{
  startTimeout();
  while ((readRegister(REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired())
    {
      // Если датчик не сообщил о готовности, возвращаем максимальное значение.
      _didTimeout = true;
      return 65535;
    }
  }

  // В проекте используется стандартный режим: Linearity Corrective Gain = 1000,
  // fractional ranging не включен.
  uint16_t range = readRegister16(REG_RESULT_RANGE_STATUS + 10);

  // Сбрасываем флаг готовности измерения.
  writeRegister(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Проверяет, был ли timeout с прошлого вызова, и сразу сбрасывает флаг.
bool VL53L0X::timeoutOccurred()
{
  bool tmp = _didTimeout;
  _didTimeout = false;
  return tmp;
}

void VL53L0X::startTimeout()
{
  // Запоминаем момент начала ожидания готовности датчика.
  _timeoutStartMs = millis();
}

bool VL53L0X::checkTimeoutExpired() const
{
  return (millis() - _timeoutStartMs) > TIMEOUT_MS;
}

// Получает количество и тип опорных SPAD.
// Основано на VL53L0X_get_info_from_device(), но оставлены только нужные поля.
bool VL53L0X::getSpadInfo(uint8_t * count, bool * typeIsAperture)
{
  uint8_t tmp;

  writeRegister(0x80, 0x01);
  writeRegister(0xFF, 0x01);
  writeRegister(0x00, 0x00);

  writeRegister(0xFF, 0x06);
  writeRegister(0x83, readRegister(0x83) | 0x04);
  writeRegister(0xFF, 0x07);
  writeRegister(0x81, 0x01);

  writeRegister(0x80, 0x01);

  writeRegister(0x94, 0x6b);
  writeRegister(0x83, 0x00);
  startTimeout();
  while (readRegister(0x83) == 0x00)
  {
    // Ждем, пока датчик подготовит SPAD-информацию.
    if (checkTimeoutExpired()) { return false; }
  }
  writeRegister(0x83, 0x01);
  tmp = readRegister(0x92);

  // Младшие 7 бит — количество SPAD, старший бит — тип aperture.
  *count = tmp & 0x7f;
  *typeIsAperture = (tmp >> 7) & 0x01;

  writeRegister(0x81, 0x00);
  writeRegister(0xFF, 0x06);
  writeRegister(0x83, readRegister(0x83)  & ~0x04);
  writeRegister(0xFF, 0x01);
  writeRegister(0x00, 0x01);

  writeRegister(0xFF, 0x00);
  writeRegister(0x80, 0x00);

  return true;
}

// Читает, какие шаги измерительной последовательности сейчас включены.
// Основано на VL53L0X_GetSequenceStepEnables().
void VL53L0X::getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequenceConfig = readRegister(REG_SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequenceConfig >> 4) & 0x1;
  enables->dss          = (sequenceConfig >> 3) & 0x1;
  enables->msrc         = (sequenceConfig >> 2) & 0x1;
  enables->preRange    = (sequenceConfig >> 6) & 0x1;
  enables->finalRange  = (sequenceConfig >> 7) & 0x1;
}

// Читает таймауты всех нужных шагов измерения.
// Основано на get_sequence_step_timeout(), но сразу сохраняет промежуточные значения.
void VL53L0X::getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->preRangeVcselPeriodPclks = decodeVcselPeriod(readRegister(REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));

  timeouts->msrcDssTccMclks = readRegister(REG_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrcDssTccUs =
    timeoutMclksToMicroseconds(timeouts->msrcDssTccMclks,
                               timeouts->preRangeVcselPeriodPclks);

  timeouts->preRangeMclks =
    decodeTimeout(readRegister16(REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->preRangeUs =
    timeoutMclksToMicroseconds(timeouts->preRangeMclks,
                               timeouts->preRangeVcselPeriodPclks);

  timeouts->finalRangeVcselPeriodPclks = decodeVcselPeriod(readRegister(REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));

  timeouts->finalRangeMclks =
    decodeTimeout(readRegister16(REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->preRange)
  {
    // В регистре final range хранится значение вместе с pre-range.
    timeouts->finalRangeMclks -= timeouts->preRangeMclks;
  }

  timeouts->finalRangeUs =
    timeoutMclksToMicroseconds(timeouts->finalRangeMclks,
                               timeouts->finalRangeVcselPeriodPclks);
}

// Декодирует таймаут шага из формата регистра в MCLK.
// Основано на VL53L0X_decode_timeout().
uint16_t VL53L0X::decodeTimeout(uint16_t reg_val)
{
  // Формат: (LSByte * 2^MSByte) + 1.
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Кодирует таймаут в MCLK обратно в формат регистра.
// Основано на VL53L0X_encode_timeout().
uint16_t VL53L0X::encodeTimeout(uint32_t timeoutMclks)
{
  // Формат: (LSByte * 2^MSByte) + 1.

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeoutMclks > 0)
  {
    ls_byte = timeoutMclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Переводит таймаут из MCLK в микросекунды для заданного периода VCSEL.
// Основано на VL53L0X_calc_timeout_us().
uint32_t VL53L0X::timeoutMclksToMicroseconds(uint16_t timeoutPeriodMclks, uint8_t vcselPeriodPclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcselPeriodPclks);

  return ((timeoutPeriodMclks * macro_period_ns) + 500) / 1000;
}

// Переводит таймаут из микросекунд в MCLK для заданного периода VCSEL.
// Основано на VL53L0X_calc_timeout_mclks().
uint32_t VL53L0X::timeoutMicrosecondsToMclks(uint32_t timeoutPeriodUs, uint8_t vcselPeriodPclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcselPeriodPclks);

  return (((timeoutPeriodUs * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

uint8_t VL53L0X::decodeVcselPeriod(uint8_t value)
{
  // В регистрах период VCSEL хранится в сжатом виде.
  return (value + 1) << 1;
}

uint32_t VL53L0X::calcMacroPeriod(uint8_t vcselPeriodPclks)
{
  // Расчет длительности макропериода по формуле из API ST.
  return (((uint32_t)2304 * vcselPeriodPclks * 1655) + 500) / 1000;
}

// Выполняет одну опорную калибровку датчика.
// Основано на VL53L0X_perform_single_ref_calibration().
bool VL53L0X::performSingleRefCalibration(uint8_t vhvInitByte)
{
  writeRegister(REG_SYSRANGE_START, 0x01 | vhvInitByte); // start/stop режим

  startTimeout();
  while ((readRegister(REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    // Ждем завершения калибровки.
    if (checkTimeoutExpired()) { return false; }
  }

  // Сбрасываем прерывание и останавливаем одиночное измерение.
  writeRegister(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeRegister(REG_SYSRANGE_START, 0x00);

  return true;
}
