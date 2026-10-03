// Тест модуля хранения EEPROM прошивки safe_lock на ПК.
//
// Берёт блок STORAGE-BEGIN..STORAGE-END из прошивки (файл storage.inc
// создаётся make или run_test.sh), подменяет EEPROM имитацией и проверяет:
//  - обрыв питания на каждом байте каждой записи (5 вариантов испорченного байта);
//  - двойной сбой: обрыв ещё и во время починки копий при следующем включении;
//  - порчу 1-2 копий из 3 (20 000 случайных случаев);
//  - поведение при порче всех копий, на чистом чипе и с чужими данными;
//  - износ ячеек за 5 лет.
// Ожидаемый результат: "checks: 51440, failures: 0".

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <functional>
struct PowerLoss {};
struct MockEEPROM {
  uint8_t mem[1024]; long budget = -1; int torn = 0; long writes = 0;
  void wb(int a, uint8_t v) {
    if (mem[a] == v) return;               // update(): одинаковые не пишутся
    if (budget == 0) {
      switch (torn) { case 0: break; case 1: mem[a] = v; break; case 2: mem[a] = 0xFF; break;
                      case 3: mem[a] = (uint8_t)(v & mem[a]); break; default: mem[a] = rand(); }
      throw PowerLoss();
    }
    if (budget > 0) budget--;
    mem[a] = v; writes++;
  }
  template<class T> T& get(int a, T& t) { memcpy(&t, mem + a, sizeof t); return t; }
  template<class T> const T& put(int a, const T& t) { auto p = (const uint8_t*)&t; for (size_t i = 0; i < sizeof t; i++) wb(a + i, p[i]); return t; }
} EEPROM;
static uint16_t _crc_ccitt_update(uint16_t crc, uint8_t data) { // как в avr-libc
  data ^= (uint8_t)(crc & 0xff); data ^= data << 4;
  return ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}
#define CODE_MAX_LEN 8
const uint8_t CODE_MIN_LEN = 4; const uint8_t MAX_FAILS = 3; const char FACTORY_CODE[] = "0000";
#include "storage.inc"

std::string codeStr() { return std::string(gCode.code, gCode.len); }
std::string snap() { char b[64]; snprintf(b, 64, "%s|%d|%d", codeStr().c_str(), gState.mode, gState.fails); return b; }
// Загрузка "после включения": очистить RAM, loadAll
bool boot(std::string &out) { memset(&gCode,0,sizeof gCode); memset(&gState,0,sizeof gState); bool ok = loadAll(); out = snap(); return ok; }
bool copiesConsistent() {
  for (auto base : {ADDR_CODE, ADDR_STATE}) {
    int sz = base == ADDR_CODE ? sizeof(Rec<CodeData>) : sizeof(Rec<StateData>);
    for (int i = 1; i < 3; i++) if (memcmp(EEPROM.mem + base[0], EEPROM.mem + base[i], sz)) return false;
  }
  return true;
}
long checks = 0, fails = 0;
void expect(bool c, const char* msg, const std::string& ctx) { checks++; if (!c) { fails++; if (fails < 20) printf("FAIL: %s  (%s)\n", msg, ctx.c_str()); } }

int main() {
  srand(1);
  // 1. Чистый чип и чужой мусор
  memset(EEPROM.mem, 0xFF, 1024); std::string s;
  expect(boot(s) && s == "0000|0|0", "virgin init", s);
  for (int t = 0; t < 200; t++) {
    for (auto &b : EEPROM.mem) b = rand();
    EEPROM.mem[0]=EEPROM.mem[128]=EEPROM.mem[256]=0; // без метки
    expect(boot(s) && s == "0000|0|0", "foreign garbage -> factory", s);
  }
  // 2. Обрыв питания на каждом байте каждой операции
  std::vector<std::pair<const char*, std::function<void()>>> ops = {
    {"st N1", []{ saveState(MODE_NORMAL,1); }}, {"st N2", []{ saveState(MODE_NORMAL,2); }},
    {"st L3", []{ saveState(MODE_LOCKED,3); }}, {"st T3", []{ saveState(MODE_ONE_TRY,3); }},
    {"st L3b",[]{ saveState(MODE_LOCKED,3); }}, {"code8", []{ saveCode("12345678",8); }},
    {"st N0", []{ saveState(MODE_NORMAL,0); }}, {"codeA", []{ saveCode("A1B2",4); }},
    {"factory",[]{ factoryDefaults(); }}, {"code5", []{ saveCode("98765",5); }},
  };
  memset(EEPROM.mem, 0xFF, 1024); boot(s);
  long cuts = 0;
  for (auto &op : ops) {
    uint8_t before[1024]; memcpy(before, EEPROM.mem, 1024);
    std::string oldV; boot(oldV);
    op.second(); std::string newV = snap();
    uint8_t after[1024]; memcpy(after, EEPROM.mem, 1024);
    for (long k = 0;; k++) {
      bool finished = false;
      for (int torn = 0; torn < 5; torn++) {
        memcpy(EEPROM.mem, before, 1024); boot(s);
        EEPROM.budget = k; EEPROM.torn = torn;
        try { op.second(); finished = true; } catch (PowerLoss&) {}
        EEPROM.budget = -1;
        if (finished) break;
        cuts++;
        std::string r; bool ok = boot(r);
        std::string ctx = std::string(op.first) + " cut@" + std::to_string(k) + " torn" + std::to_string(torn) + " -> " + r;
        expect(ok, "load ok", ctx);
        expect(r == oldV || r == newV, "old or new value", ctx + " old=" + oldV + " new=" + newV);
        expect(copiesConsistent(), "copies repaired", ctx);
        std::string r2; boot(r2); expect(r2 == r, "stable on 2nd boot", ctx);
        // двойной сбой: обрыв во время починки при следующем включении
        uint8_t mid[1024];
        memcpy(EEPROM.mem, before, 1024); boot(s); EEPROM.budget = k; EEPROM.torn = torn;
        try { op.second(); } catch (PowerLoss&) {} EEPROM.budget = -1;
        memcpy(mid, EEPROM.mem, 1024);
        for (long j = 0; j < 40; j++) {
          memcpy(EEPROM.mem, mid, 1024); EEPROM.budget = j; EEPROM.torn = rand() % 5;
          std::string x; bool rep = true; try { boot(x); } catch (PowerLoss&) { rep = false; } EEPROM.budget = -1;
          if (rep) break;
          std::string r3; bool ok3 = boot(r3);
          expect(ok3 && (r3 == oldV || r3 == newV), "double fault", ctx + " repcut@" + std::to_string(j) + " -> " + r3);
        }
      }
      if (finished) break;
    }
    memcpy(EEPROM.mem, after, 1024);
  }
  printf("power-cut scenarios: %ld\n", cuts);
  // 3. Порча копий (старение ячеек)
  memset(EEPROM.mem, 0xFF, 1024); boot(s); saveCode("4321",4); saveState(MODE_NORMAL,2); std::string good; boot(good);
  uint8_t base[1024]; memcpy(base, EEPROM.mem, 1024);
  for (int t = 0; t < 20000; t++) {
    memcpy(EEPROM.mem, base, 1024);
    int nbad = 1 + rand() % 2;   // 1 или 2 копии из 3
    for (int region = 0; region < 2; region++) {
      const int* a = region ? ADDR_STATE : ADDR_CODE; int sz = region ? sizeof(Rec<StateData>) : sizeof(Rec<CodeData>);
      int first = rand() % 3;
      for (int c = 0; c < nbad; c++) { int ci = (first + c) % 3; int nb = 1 + rand() % 3;
        for (int q = 0; q < nb; q++) EEPROM.mem[a[ci] + rand() % sz] ^= (1 << (rand() % 8)); }
    }
    std::string r; bool ok = boot(r);
    expect(ok && r == good, "survive corruption of 1-2 copies", r);
    expect(copiesConsistent(), "repaired after corruption", r);
  }
  // 4. Все три копии испорчены
  memcpy(EEPROM.mem, base, 1024); for (int i = 0; i < 3; i++) EEPROM.mem[ADDR_CODE[i] + 3] ^= 0x10;
  expect(!boot(s), "all code copies bad -> fault, not factory 0000", s);
  memcpy(EEPROM.mem, base, 1024); for (int i = 0; i < 3; i++) EEPROM.mem[ADDR_STATE[i] + 5] ^= 0x01;
  expect(boot(s) && s == "4321|1|3", "all state copies bad -> locked", s);
  // 5. Износ: записи на одну ячейку за 5 лет (2 открытия/день, 2 записи состояния на открытие)
  memset(EEPROM.mem, 0xFF, 1024); boot(s);
  static long cell[1024] = {0}; uint8_t prev[1024]; memcpy(prev, EEPROM.mem, 1024);
  for (int i = 0; i < 3650*2; i++) { saveState(MODE_NORMAL, 1); saveState(MODE_NORMAL, 0);
    for (int a = 0; a < 1024; a++) { if (EEPROM.mem[a] != prev[a]) cell[a] += 1; } memcpy(prev, EEPROM.mem, 1024); }
  long mx = 0; for (long c : cell) if (c > mx) mx = c;
  printf("max writes per cell over 5 years (2 openings/day): %ld (limit 100000)\n", mx);
  printf("checks: %ld, failures: %ld\n", checks, fails);
  return fails != 0;
}
