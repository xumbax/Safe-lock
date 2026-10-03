/*
  ============================================================================
   КОДОВЫЙ ЗАМОК СЕЙФА                                       версия 2.2
   Плата: Arduino Pro Mini 3.3V / 8 MHz (ATmega328P)
   Arduino IDE: Плата "Arduino Pro or Pro Mini",
                Процессор "ATmega328P (3.3V, 8 MHz)"
   Документация: README.md и папка docs/ в корне репозитория
  ============================================================================

  ПОДКЛЮЧЕНИЕ (кратко)
     4xAA -- ТУМБЛЕР -->|-- (1N5819) --+-- шина +  -> Pro Mini RAW, XY-MOS VIN+
     USB-C VBUS -----------------------+
     XY-MOS: PWM/TRIG <- D10, OUT+/OUT- -> соленоид (+ диод 1N5819 катодом к "+")
     Буззер: D11.   Кнопка программирования (внутри): D12 -- GND
     Клавиатура: разъём 8 pin прямо на D2..D9 подряд (вывод 1 -> D2 ... 8 -> D9).
                 Клавиши A/B/C/D включаются настройкой USE_ABCD.
     D13 (светодиод на плате) не используется - паять ничего не нужно.

  РАБОТА (кратко)
     Тумблер ВКЛ -> писк "готов". Код 4..8 символов, '#' - ввод, '*' - стереть.
     Заводской код 0000#.  3 ошибки -> 5 мин блокировки -> 1 попытка -> ...
     Открыто 3 с: 1 с полный ток, 2 с спад ШИМ до 50%; в конце каждой секунды
     тон: высокий, средний, низкий - после низкого соленоид выключается.
     Смена кода: открыть -> внутренняя кнопка -> новый код # -> повтор #.
     Сброс на 0000: держать внутреннюю кнопку при включении 10 с.

  ХРАНЕНИЕ
     Код и состояние (счётчик ошибок, блокировка) хранятся каждый в 3 копиях
     в разных местах EEPROM. У каждой копии: метка, порядковый номер записи,
     данные, CRC16. Копии пишутся по очереди с проверкой чтением.
     При включении берётся целая копия с наибольшим номером, остальные
     перезаписываются ею. Обрыв питания в любой момент записи оставляет
     либо старое, либо новое значение - никогда мусор.
  ============================================================================
*/

#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/crc16.h>

// ============================== НАСТРОЙКИ ===================================

#define BUZZER_PASSIVE   1     // 1 - пассивный буззер (тоны), 0 - активный
#define USE_WATCHDOG     1     // 1 - перезагрузка при зависании (см. README)
#define CHECK_BOD        1     // 1 - предупреждать при старте, если BOD выключен

// Соленоид: SOL_FULL_MS полный ток, затем за SOL_RAMP_MS заполнение ШИМ
// плавно спадает со 100% до SOL_END_DUTY%. Всего открыто SOL_FULL_MS + SOL_RAMP_MS.
const uint16_t SOL_FULL_MS     = 1000;   // полный ток, мс
const uint16_t SOL_RAMP_MS     = 2000;   // спад, мс
const uint8_t  SOL_END_DUTY    = 50;     // заполнение в конце спада, % (100 = без спада)
const uint16_t SOL_PWM_HZ      = 1000;   // частота ШИМ (XY-MOS: не выше 1-2 кГц)

// Отсчёт, пока замок открыт: в конце каждой секунды один короткий тон -
// высокий, средний, низкий. После низкого соленоид выключается.
// По тонам слышно, на какой секунде отпала задвижка.
const uint16_t OPEN_TICK_MS    = 1000;   // период отсчёта, мс
const uint8_t  OPEN_TONE_MS    = 15;     // длина тона, мс (как щелчок клавиши)
const uint16_t OPEN_TONES_HZ[3] = {2500, 2000, 1500};   // 1-я, 2-я, 3-я секунда и далее

// Код
#define CODE_MAX_LEN 8                   // не больше 8 без правки раскладки EEPROM
const uint8_t  CODE_MIN_LEN    = 4;
const char     FACTORY_CODE[]  = "0000";

// Защита от перебора
const uint8_t  MAX_FAILS       = 3;               // ошибок до блокировки
const uint32_t LOCKOUT_MS      = 5UL * 60UL * 1000UL;
const uint16_t WRONG_DELAY_MS  = 1500;           // пауза перед сигналом "неверно"

// Напоминание выключить тумблер
const uint16_t REMIND_START_S  = 30;     // после последнего нажатия
const uint16_t REMIND_PERIOD_S = 30;     // период (точность WDT +-10%)

// Смена кода / сброс
const uint16_t PROG_TIMEOUT_S        = 30;
const uint8_t  FACTORY_RESET_HOLD_S  = 10;

// Задержка после включения до любой записи в EEPROM (дребезг тумблера)
const uint16_t POWERUP_DELAY_MS = 300;

// Пины
// Клавиатура 4x4: разъём 8 pin прямо на D2..D9 (вывод клавиатуры 1 -> D2).
// Если разъём надет наоборот (вывод 1 -> D9), поставьте KEYPAD_REVERSED 1.
#define KEYPAD_REVERSED 0
#define USE_ABCD        0      // 1 - клавиши A, B, C, D тоже можно использовать в коде
const uint8_t KP_ROWS = 4;
#if USE_ABCD
const uint8_t KP_COLS = 4;
#else
const uint8_t KP_COLS = 3;     // колонка A/B/C/D не опрашивается
#endif
#if KEYPAD_REVERSED
const uint8_t ROW_PINS[KP_ROWS] = {9, 8, 7, 6};     // выводы клавиатуры 1..4
const uint8_t COL_PINS[4]       = {5, 4, 3, 2};     // выводы клавиатуры 5..8
#else
const uint8_t ROW_PINS[KP_ROWS] = {2, 3, 4, 5};     // выводы клавиатуры 1..4
const uint8_t COL_PINS[4]       = {6, 7, 8, 9};     // выводы клавиатуры 5..8
#endif
const uint8_t PIN_KP_ABCD = COL_PINS[3];            // вывод 8 (колонка ABCD)
const uint8_t PIN_SOL  = 10;   // OC1B (аппаратный ШИМ Timer1) - не менять
const uint8_t PIN_BUZ  = 11;
const uint8_t PIN_PROG = 12;

const char KEYMAP[KP_ROWS][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// ============================================================================


// ===== STORAGE-BEGIN ========================================================
// ВНИМАНИЕ: типы ниже должны идти ДО первой функции в файле - Arduino IDE
// вставляет автоматические объявления функций перед первой функцией.
// Надёжное хранение в EEPROM. Этот блок также проверяется тестом на ПК
// (имитация обрыва питания на каждом байте записи).

const uint8_t COPIES      = 3;
const uint8_t MAGIC_CODE  = 0xC7;
const uint8_t MAGIC_STATE = 0x5E;

// Копии разнесены по EEPROM (1 КБ)
const int ADDR_CODE[COPIES]  = {0,   128, 256};
const int ADDR_STATE[COPIES] = {384, 512, 640};

enum Mode : uint8_t { MODE_NORMAL = 0, MODE_LOCKED = 1, MODE_ONE_TRY = 2 };
enum LoadResult : uint8_t { LOAD_OK, LOAD_VIRGIN, LOAD_CORRUPT };

struct CodeData  { uint8_t len; char code[CODE_MAX_LEN]; } __attribute__((packed));
struct StateData { uint8_t mode; uint8_t fails; }          __attribute__((packed));

template <typename T>
struct Rec {
  uint8_t  magic;   // пишется один раз и больше не меняется
  uint32_t seq;     // номер записи, растёт с каждой записью
  T        d;
  uint16_t crc;     // CRC16-CCITT по всем предыдущим байтам
} __attribute__((packed));

CodeData  gCode;
StateData gState;

bool keyAllowed(char k) {
  return (k >= '0' && k <= '9') || (k >= 'A' && k <= 'D');
}

bool dataValid(const CodeData &d) {
  if (d.len < CODE_MIN_LEN || d.len > CODE_MAX_LEN) return false;
  for (uint8_t i = 0; i < CODE_MAX_LEN; i++) {
    if (i < d.len ? !keyAllowed(d.code[i]) : d.code[i] != 0) return false;
  }
  return true;
}

bool dataValid(const StateData &d) {
  return d.mode <= MODE_ONE_TRY && d.fails <= MAX_FAILS;
}

template <typename T>
uint16_t recCrc(const Rec<T> &r) {
  const uint8_t *p = (const uint8_t *)&r;
  uint16_t c = 0xFFFF;
  for (uint8_t i = 0; i < sizeof(Rec<T>) - 2; i++) c = _crc_ccitt_update(c, p[i]);
  return c;
}

template <typename T>
bool recValid(const Rec<T> &r, uint8_t magic) {
  return r.magic == magic && r.crc == recCrc(r) && dataValid(r.d);
}

// Чтение: берётся целая копия с наибольшим номером, остальные чинятся.
// Лучшая копия при починке никогда не перезаписывается.
template <typename T>
LoadResult loadRec(const int *addr, uint8_t magic, T &out) {
  Rec<T> r[COPIES];
  bool v[COPIES];
  bool anyMagic = false;
  int8_t best = -1;
  for (uint8_t i = 0; i < COPIES; i++) {
    EEPROM.get(addr[i], r[i]);
    v[i] = recValid(r[i], magic);
    if (r[i].magic == magic) anyMagic = true;
    if (v[i] && (best < 0 || r[i].seq > r[best].seq)) best = i;
  }
  if (best < 0) return anyMagic ? LOAD_CORRUPT : LOAD_VIRGIN;
  for (uint8_t i = 0; i < COPIES; i++) {
    if (i != best && (!v[i] || memcmp(&r[i], &r[best], sizeof(Rec<T>)) != 0)) {
      EEPROM.put(addr[i], r[best]);
    }
  }
  out = r[best].d;
  return LOAD_OK;
}

// Запись: новый номер = максимальный целый + 1; копии пишутся по очереди,
// каждая проверяется чтением (при несовпадении - повтор).
// Успех, если записано не меньше 2 копий из 3.
template <typename T>
bool saveRec(const int *addr, uint8_t magic, const T &d) {
  Rec<T> r;
  uint32_t maxSeq = 0;
  bool found = false;
  for (uint8_t i = 0; i < COPIES; i++) {
    EEPROM.get(addr[i], r);
    if (recValid(r, magic) && (!found || r.seq > maxSeq)) { maxSeq = r.seq; found = true; }
  }
  Rec<T> n;
  memset(&n, 0, sizeof(n));
  n.magic = magic;
  n.seq = found ? maxSeq + 1 : 1;
  n.d = d;
  n.crc = recCrc(n);

  uint8_t ok = 0;
  for (uint8_t i = 0; i < COPIES; i++) {
    for (uint8_t attempt = 0; attempt < 2; attempt++) {
      EEPROM.put(addr[i], n);
      EEPROM.get(addr[i], r);
      if (memcmp(&r, &n, sizeof(n)) == 0) { ok++; break; }
    }
  }
  return ok >= 2;
}

bool saveCode(const char *c, uint8_t len) {
  CodeData d;
  memset(&d, 0, sizeof(d));
  d.len = len;
  memcpy(d.code, c, len);
  gCode = d;
  return saveRec(ADDR_CODE, MAGIC_CODE, d);
}

// gState в памяти всегда получает новое значение (даже если запись не
// удалась) - в памяти оно не мягче того, что в EEPROM.
bool saveState(uint8_t mode, uint8_t fails) {
  StateData d;
  d.mode = mode;
  d.fails = fails > MAX_FAILS ? MAX_FAILS : fails;
  gState = d;
  return saveRec(ADDR_STATE, MAGIC_STATE, d);
}

bool factoryDefaults() {
  bool ok = saveState(MODE_NORMAL, 0);          // сначала состояние,
  ok = saveCode(FACTORY_CODE, strlen(FACTORY_CODE)) && ok;   // потом код
  return ok;
}

// true - всё загружено; false - код в памяти повреждён (все 3 копии)
bool loadAll() {
  LoadResult rc = loadRec(ADDR_CODE, MAGIC_CODE, gCode);
  if (rc == LOAD_CORRUPT) return false;
  if (rc == LOAD_VIRGIN) {                      // новый чип / чужие данные
    factoryDefaults();
    return loadRec(ADDR_CODE, MAGIC_CODE, gCode) == LOAD_OK &&
           loadRec(ADDR_STATE, MAGIC_STATE, gState) == LOAD_OK;
  }
  if (loadRec(ADDR_STATE, MAGIC_STATE, gState) != LOAD_OK) {
    saveState(MODE_LOCKED, MAX_FAILS);          // состояние потеряно -> блокировка
  }
  return true;
}
// ===== STORAGE-END ==========================================================

// Сброс флагов сторожевого таймера как можно раньше (до setup)
void wdtEarlyInit(void) __attribute__((naked, used, section(".init3")));
void wdtEarlyInit(void) {
  MCUSR = 0;
  wdt_disable();
}

// ---------------------------- Сторожевой таймер -----------------------------

volatile bool gWdtFired = false;
ISR(WDT_vect) { gWdtFired = true; }

void wdtActiveMode() {
#if USE_WATCHDOG
  wdt_enable(WDTO_4S);
#else
  wdt_disable();
#endif
}

void wdtSleepMode1s() {           // прерывание раз в ~1 с (для сна)
  cli();
  wdt_reset();
  MCUSR &= ~_BV(WDRF);
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | _BV(WDP2) | _BV(WDP1);
  sei();
}

void delayWdt(uint32_t ms) {
  while (ms) {
    uint16_t d = ms > 50 ? 50 : ms;
    delay(d);
    ms -= d;
    wdt_reset();
  }
}

bool bodEnabled() {
  cli();
  uint8_t ext = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
  sei();
  return (ext & 0x07) != 0x07;    // BODLEVEL = 111 - BOD выключен
}

// ---------------------------- Соленоид --------------------------------------

void solOff() {
  TCCR1A = 0;
  TCCR1B = 0;
  digitalWrite(PIN_SOL, LOW);
}

// ---------------------------- Буззер и сигналы ------------------------------

void beep(uint16_t freq, uint16_t ms) {
#if BUZZER_PASSIVE
  tone(PIN_BUZ, freq);
  delayWdt(ms);
  noTone(PIN_BUZ);
  digitalWrite(PIN_BUZ, LOW);
#else
  (void)freq;
  digitalWrite(PIN_BUZ, HIGH);
  delayWdt(ms);
  digitalWrite(PIN_BUZ, LOW);
#endif
}

void gap(uint16_t ms) { delayWdt(ms); }

void sigClick()     { beep(2500, 15); }
void sigReady()     { beep(1000, 80); gap(60); beep(1500, 80); }
void sigOpen()      { beep(1000, 90); gap(30); beep(1300, 90); gap(30); beep(1700, 150); }
void sigWrong()     { beep(300, 700); }
void sigError()     { beep(400, 120); gap(100); beep(400, 120); }
void sigLockStart() { for (uint8_t i = 0; i < 3; i++) { beep(250, 300); gap(150); } }
void sigWait()      { beep(250, 60); }
void sigRemind()    { beep(2000, 60); gap(80); beep(2000, 60); }
void sigProgEnter() { beep(1500, 300); gap(100); beep(2000, 300); }
void sigStep()      { beep(1800, 100); }
void sigSuccess()   { beep(1000, 120); gap(40); beep(1500, 120); gap(40); beep(2000, 250); }
void sigTick()      { beep(1200, 40); }
void sigFault()     { beep(400, 500); gap(100); beep(400, 100); gap(100); beep(400, 500); }
void sigBodWarn()   { for (uint8_t i = 0; i < 5; i++) { beep(600, 50); gap(50); } }

// ---------------------------- Открытие --------------------------------------

void buzzStart(uint16_t freq) {
#if BUZZER_PASSIVE
  tone(PIN_BUZ, freq);
#else
  (void)freq;
  digitalWrite(PIN_BUZ, HIGH);
#endif
}

void buzzStop() {
#if BUZZER_PASSIVE
  noTone(PIN_BUZ);
#endif
  digitalWrite(PIN_BUZ, LOW);
}

// Полный ток SOL_FULL_MS, затем линейный спад заполнения ШИМ 100% -> SOL_END_DUTY%
// за SOL_RAMP_MS. В конце каждой секунды (OPEN_TICK_MS) - короткий тон:
// высокий, средний, низкий. Соленоид выключается после последнего тона.
void openLock() {
  const uint16_t top = (uint16_t)(F_CPU / SOL_PWM_HZ) - 1;
  const uint8_t  endDuty = SOL_END_DUTY > 100 ? 100 : SOL_END_DUTY;
  const uint32_t total = (uint32_t)SOL_FULL_MS + SOL_RAMP_MS;
  const uint8_t  nTones = sizeof(OPEN_TONES_HZ) / sizeof(OPEN_TONES_HZ[0]);

  // Fast PWM, TOP = ICR1. OCR1B = TOP даёт постоянную 1 (100%).
  // OCR1B буферизуется и обновляется в начале периода - без рывков.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  ICR1 = top;
  OCR1B = top;
  TCCR1A = _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  bool toneOn = false;
  uint32_t start = millis();
  uint32_t t;
  while ((t = millis() - start) < total + OPEN_TONE_MS) {
    wdt_reset();

    uint8_t duty = 100;
    if (t > SOL_FULL_MS) {
      uint32_t r = t - SOL_FULL_MS;
      duty = (SOL_RAMP_MS == 0 || r >= SOL_RAMP_MS)
             ? endDuty
             : 100 - (uint8_t)((uint32_t)(100 - endDuty) * r / SOL_RAMP_MS);
    }
    OCR1B = (uint16_t)((uint32_t)top * duty / 100);

    // тон k (k = 1, 2, 3...) звучит с k*OPEN_TICK_MS в течение OPEN_TONE_MS
    uint32_t k = t / OPEN_TICK_MS;
    bool want = k >= 1 && (t % OPEN_TICK_MS) < OPEN_TONE_MS;
    if (want && !toneOn) {
      buzzStart(OPEN_TONES_HZ[k - 1 < nTones ? k - 1 : nTones - 1]);
      toneOn = true;
    } else if (!want && toneOn) {
      buzzStop();
      toneOn = false;
    }
  }
  solOff();
  buzzStop();
}

// ---------------------------- Клавиатура ------------------------------------

void kpIdle() {
  for (uint8_t r = 0; r < KP_ROWS; r++) { digitalWrite(ROW_PINS[r], LOW); pinMode(ROW_PINS[r], INPUT); }
  for (uint8_t c = 0; c < KP_COLS; c++) pinMode(COL_PINS[c], INPUT_PULLUP);
  pinMode(PIN_KP_ABCD, INPUT_PULLUP);     // колонка ABCD всегда вход (опрашивается при USE_ABCD 1)
}

char kpScanRaw() {
  for (uint8_t r = 0; r < KP_ROWS; r++) {
    digitalWrite(ROW_PINS[r], LOW);
    pinMode(ROW_PINS[r], OUTPUT);
    delayMicroseconds(30);
    for (uint8_t c = 0; c < KP_COLS; c++) {
      if (digitalRead(COL_PINS[c]) == LOW) {
        pinMode(ROW_PINS[r], INPUT);
        return KEYMAP[r][c];
      }
    }
    pinMode(ROW_PINS[r], INPUT);
  }
  return 0;
}

// Символ один раз на нажатие (антидребезг 30 мс), иначе 0
char getKey() {
  static char last = 0;
  static uint32_t since = 0;
  static bool reported = false;
  char k = kpScanRaw();
  if (k != last) {
    last = k;
    since = millis();
    reported = false;
    return 0;
  }
  if (k && !reported && millis() - since >= 30) {
    reported = true;
    return k;
  }
  return 0;
}

bool progPressed() {
  if (digitalRead(PIN_PROG) != LOW) return false;
  delayWdt(50);
  return digitalRead(PIN_PROG) == LOW;
}

void waitProgRelease() {
  while (digitalRead(PIN_PROG) == LOW) delayWdt(10);
  delayWdt(50);
}

// ---------------------------- Сон -------------------------------------------

EMPTY_INTERRUPT(PCINT0_vect);
EMPTY_INTERRUPT(PCINT1_vect);
EMPTY_INTERRUPT(PCINT2_vect);

void pcintEnable(uint8_t pin) {
  *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
  PCIFR |= _BV(digitalPinToPCICRbit(pin));
  PCICR |= _BV(digitalPinToPCICRbit(pin));
}

void pcintDisableAll() {
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;
}

bool wakeInputActive() {
  for (uint8_t c = 0; c < KP_COLS; c++) if (digitalRead(COL_PINS[c]) == LOW) return true;
  return digitalRead(PIN_PROG) == LOW;
}

// Спать seconds секунд. true - разбудило нажатие, false - время вышло.
bool sleepSeconds(uint16_t seconds) {
  for (uint8_t r = 0; r < KP_ROWS; r++) { digitalWrite(ROW_PINS[r], LOW); pinMode(ROW_PINS[r], OUTPUT); }
  for (uint8_t c = 0; c < KP_COLS; c++) { pinMode(COL_PINS[c], INPUT_PULLUP); pcintEnable(COL_PINS[c]); }
  pcintEnable(PIN_PROG);
  delayMicroseconds(50);

  bool byKey = false;
  uint16_t ticks = 0;
  wdtSleepMode1s();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  while (ticks < seconds) {
    cli();
    if (wakeInputActive()) { sei(); byKey = true; break; }
    gWdtFired = false;
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
    if (gWdtFired) ticks++;
    else { byKey = true; break; }
  }

  pcintDisableAll();
  wdtActiveMode();
  kpIdle();
  return byKey;
}

// ---------------------------- Логика замка ----------------------------------

char     gBuf[CODE_MAX_LEN];
uint8_t  gLen = 0;
bool     gOverflow = false;
bool     gAuthorized = false;          // сейф открыт в этой сессии питания
uint32_t gLastActivity = 0;

void clearBuf() {
  memset(gBuf, 0, sizeof(gBuf));
  gLen = 0;
  gOverflow = false;
}

// Сравнение без раннего выхода
bool codeMatches(const char *buf, uint8_t len, bool overflow) {
  uint8_t diff = (uint8_t)(len ^ gCode.len) | (overflow ? 1 : 0);
  for (uint8_t i = 0; i < CODE_MAX_LEN; i++) {
    char a = i < len ? buf[i] : 0;
    char b = i < gCode.len ? gCode.code[i] : 0;
    diff |= (uint8_t)(a ^ b);
  }
  return diff == 0;
}

void doLockout() {
  while (true) {
    sigLockStart();
    clearBuf();
    uint32_t start = millis();
    while (millis() - start < LOCKOUT_MS) {
      wdt_reset();
      if (getKey()) sigWait();
    }
    if (saveState(MODE_ONE_TRY, gState.fails)) break;
    sigFault();                       // не записалось - ещё одна блокировка
  }
  sigReady();
  gLastActivity = millis();
}

void submitCode() {
  if (!gOverflow && gLen < CODE_MIN_LEN) {   // слишком короткий - не попытка
    sigError();
    return;
  }

  // Сначала записываем попытку как неудачную, потом проверяем.
  bool saved;
  if (gState.mode == MODE_ONE_TRY) {
    saved = saveState(MODE_LOCKED, gState.fails);
  } else {
    uint8_t f = gState.fails + 1;
    saved = saveState(f >= MAX_FAILS ? MODE_LOCKED : MODE_NORMAL, f);
  }
  if (!saved) {                               // память не пишется - код не проверяем
    clearBuf();
    sigFault();
    return;
  }

  if (codeMatches(gBuf, gLen, gOverflow)) {
    saveState(MODE_NORMAL, 0);
    clearBuf();
    sigOpen();
    openLock();
    gAuthorized = true;
    return;
  }

  clearBuf();
  delayWdt(WRONG_DELAY_MS);
  sigWrong();
  if (gState.mode == MODE_LOCKED) doLockout();
}

void handleKey(char k) {
  sigClick();
  if (k == '*') { clearBuf(); return; }
  if (k == '#') {
    if (gLen == 0 && !gOverflow) return;
    submitCode();
    return;
  }
  if (gLen < CODE_MAX_LEN) gBuf[gLen++] = k;
  else gOverflow = true;
}

// Ввод нового кода в режиме программирования
bool readNewCode(char *out, uint8_t &outLen) {
  uint8_t len = 0;
  bool overflow = false;
  uint32_t t = millis();
  while (millis() - t < PROG_TIMEOUT_S * 1000UL) {
    wdt_reset();
    char k = getKey();
    if (!k) continue;
    t = millis();
    sigClick();
    if (k == '*') { len = 0; overflow = false; continue; }
    if (k == '#') {
      if (overflow || len < CODE_MIN_LEN) return false;
      outLen = len;
      return true;
    }
    if (len < CODE_MAX_LEN) out[len++] = k;
    else overflow = true;
  }
  return false;
}

void programMode() {
  waitProgRelease();
  sigProgEnter();

  char c1[CODE_MAX_LEN], c2[CODE_MAX_LEN];
  uint8_t l1 = 0, l2 = 0;

  if (!readNewCode(c1, l1)) { sigError(); return; }
  sigStep();
  if (!readNewCode(c2, l2)) { sigError(); return; }
  if (l1 != l2 || memcmp(c1, c2, l1) != 0) { sigError(); return; }

  if (saveCode(c1, l1)) sigSuccess();
  else sigFault();
}

// Кнопка зажата: держать FACTORY_RESET_HOLD_S секунд. true - сброс выполнен.
bool factoryResetHold() {
  for (uint8_t s = 0; s < FACTORY_RESET_HOLD_S; s++) {
    sigTick();
    for (uint8_t i = 0; i < 20; i++) {
      delayWdt(48);
      if (digitalRead(PIN_PROG) != LOW) return false;
    }
  }
  bool ok = factoryDefaults();
  if (ok) sigSuccess(); else sigFault();
  waitProgRelease();
  return ok;
}

// Все копии кода повреждены. Замок не открывается; раз в 30 с сигнал
// "неисправность". Выход - сброс на заводской код внутренней кнопкой.
void faultMode() {
  while (true) {
    sigFault();
    if (sleepSeconds(REMIND_PERIOD_S) && digitalRead(PIN_PROG) == LOW) {
      if (factoryResetHold()) return;
    }
  }
}

void idleReminder() {
  clearBuf();
  while (true) {
    sigRemind();
    if (sleepSeconds(REMIND_PERIOD_S)) break;
  }
  gLastActivity = millis();
}

// ---------------------------- setup / loop ----------------------------------

void setup() {
  digitalWrite(PIN_SOL, LOW);
  pinMode(PIN_SOL, OUTPUT);
  solOff();

  digitalWrite(PIN_BUZ, LOW);
  pinMode(PIN_BUZ, OUTPUT);
  pinMode(PIN_PROG, INPUT_PULLUP);

  ADCSRA &= ~_BV(ADEN);               // АЦП не используется
  kpIdle();

  wdtActiveMode();
  delayWdt(POWERUP_DELAY_MS);         // до первой записи в EEPROM

#if CHECK_BOD
  if (!bodEnabled()) sigBodWarn();
#endif

  if (digitalRead(PIN_PROG) == LOW) factoryResetHold();

  while (!loadAll()) faultMode();

  if (gState.mode == MODE_LOCKED) doLockout();   // блокировка пережила отключение

  sigReady();
  gLastActivity = millis();
}

void loop() {
  wdt_reset();

  char k = getKey();
  if (k) {
    handleKey(k);
    gLastActivity = millis();
  }

  if (gAuthorized && progPressed()) {
    programMode();
    clearBuf();
    gLastActivity = millis();
  }

  if (millis() - gLastActivity >= REMIND_START_S * 1000UL) idleReminder();
}
