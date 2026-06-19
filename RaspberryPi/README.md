# Raspberry Pi

Ten katalog jest przeznaczony na kod i konfiguracje dla modułu Raspberry Pi w projekcie Mecanosaurus.

## Struktura startera
- `src/main.py` - glowny punkt startowy modulu
- `src/hw_test.py` - szybki test UART + GPIO
- `config/settings.example.json` - przykladowa konfiguracja
- `scripts/setup.sh` - utworzenie `.venv` i instalacja zaleznosci
- `scripts/run.sh` - uruchomienie modulu
- `requirements.txt` - zaleznosci Pythona

## Szybki start
1. Przejdz do katalogu `RaspberryPi`.
2. Wykonaj `bash scripts/setup.sh`.
3. Uruchom test sprzetu: `python src/hw_test.py --uart /dev/ttyUSB0 --baud 115200`.
4. Uruchom modul: `bash scripts/run.sh`.

## Dalszy rozwoj
- Dodaj obsluge protokolu komunikacji z ESP32 (UART/Bluetooth/Wi-Fi).
- Rozszerz `main.py` o petle sterowania i watchdog.
- Dodaj logowanie do plikow i testy integracyjne.
