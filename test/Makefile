# Тест модуля хранения EEPROM на ПК (имитация обрыва питания)
# Использование:  make        - собрать и запустить
#                 make clean  - удалить сборку

FW  := ../firmware/safe_lock/safe_lock.ino
CXX ?= g++

.PHONY: run clean

run: storage_test
	./storage_test

storage.inc: $(FW)
	sed -n '/STORAGE-BEGIN/,/STORAGE-END/p' $(FW) > storage.inc

storage_test: storage_test.cpp storage.inc
	$(CXX) -std=c++17 -O2 -Wall -o storage_test storage_test.cpp

clean:
	rm -f storage_test storage_test.exe storage.inc
