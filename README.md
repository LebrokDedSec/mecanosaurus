# Mecanosaurus

System sterowania robotem z kolami mecanum.

## Struktura
- app/ - aplikacja mobilna lub desktopowa
- releases/ - gotowe buildy aplikacji do pobrania (APK/IPA)
- ESP32/ - firmware głównego robota (PlatformIO)
- ESP32_LiDAR/ - firmware modułu LiDAR (PlatformIO)
- RaspberryPi/ - kod i konfiguracje dla modułu Raspberry Pi
- firmware/ - kod ESP32/Arduino
- simulation/ - symulacje (Webots, MATLAB, modele)
- hardware/ - CAD, schematy i PCB
- docs/ - dokumentacja projektu, raporty, obrazy
- data/ - logi, CSV i dane testowe

## Funkcje
- sterowanie joystickiem
- ruch holonomiczny
- komunikacja Bluetooth/USB

## Szybki start
1. Pobierz aplikację z folderu [releases/](./releases/) (plik `.apk` dla Androida).
2. Na telefonie Android włącz **„Instaluj z nieznanych źródeł"** i otwórz pobrany plik.
3. Połącz się przez Bluetooth z robotem i steruj joystickiem.

## Dla deweloperów
1. Utwórz branch funkcjonalny, np. `feature/joystick`.
2. Commituj małe, czytelne zmiany.
3. Wypychaj zmiany i scalaj do `main` po weryfikacji.
