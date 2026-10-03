# Готовая прошивка

| Файл | Плата | Настройки |
|---|---|---|
| `safe_lock_v2.2_promini-3v3-8mhz.hex` | Arduino Pro Mini 3.3 V / 8 MHz (ATmega328P) | по умолчанию |

Собрано ядром Arduino AVR 1.8.8 из `firmware/safe_lock/safe_lock.ino` без изменений: 7818 байт флеша, 103 байта ОЗУ.

Настройки по умолчанию, как именно залить файл и как проверить контрольную сумму — в [docs/firmware.md](../../docs/firmware.md#готовая-прошивка-hex).

Проверка контрольной суммы:

```
sha256sum -c SHA256SUMS                                          # Linux / macOS / Git Bash
certutil -hashfile safe_lock_v2.2_promini-3v3-8mhz.hex SHA256    # Windows
```
