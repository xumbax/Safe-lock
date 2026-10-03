#!/bin/sh
# Тест модуля хранения EEPROM (без make). Нужен g++ или clang++.
set -e
cd "$(dirname "$0")"
sed -n '/STORAGE-BEGIN/,/STORAGE-END/p' ../firmware/safe_lock/safe_lock.ino > storage.inc
${CXX:-g++} -std=c++17 -O2 -Wall -o storage_test storage_test.cpp
./storage_test
