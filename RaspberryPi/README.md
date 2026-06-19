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

## Test AprilTag + podazanie robota (ROS2)

Poniższy test pozwala:
- wygenerowac i wydrukowac znacznik AprilTag,
- wykryc znacznik z kamery USB,
- publikowac komendy jazdy na `/cmd_vel`, aby robot jechal w kierunku taga.

### 1. Instalacja zaleznosci Pythona

```bash
cd RaspberryPi
python3 -m pip install -r requirements.txt
```

### 2. Wygenerowanie taga do wydruku

```bash
cd RaspberryPi
python3 src/generate_apriltag.py --tag-id 0 --family 36h11 --size-px 1000 --out tag0.png
```

Wydrukuj `tag0.png` w skali 100% (bez dopasowania do strony).

### 3. Uruchomienie testu podazania

Załaduj ROS2 Jazzy i uruchom node:

```bash
source /opt/ros/jazzy/setup.bash
cd RaspberryPi
python3 src/apriltag_follow_test.py \
	--camera 0 \
	--tag-id 0 \
	--tag-size 0.12 \
	--fx 615 --fy 615 --cx 320 --cy 240 \
	--target-distance 0.50 \
	--topic /cmd_vel
```

Skrypt wypisuje w terminalu:
- estymowana odleglosc `z` do taga,
- przesuniecie boczne `x`,
- kat celu (bearing),
- wysylane komendy `cmd_v` i `cmd_w`.

Uwaga: aby test byl wiarygodny, podmien `fx/fy/cx/cy` na parametry z kalibracji Twojej kamerki.

## Sterowanie ESP32 napedu po BLE na podstawie AprilTag

Jezeli firmware napedu ESP32 przyjmuje komendy BLE w formacie:
- `DRIVE:x,y,omega`
- `STOP`

mozesz uruchomic gotowy mostek kamera -> BLE:

```bash
cd RaspberryPi
python3 src/apriltag_ble_follow.py \
	--camera 0 \
	--tag-id 0 \
	--tag-size 0.12 \
	--calib-file config/camera_calib.npz \
	--distance-scale 1.52 \
	--ble-name ESP32-S3-DEVKITC-1-N16R8V
```

Opcjonalnie (szybsze laczenie) podaj adres BLE:

```bash
python3 src/apriltag_ble_follow.py --ble-address AA:BB:CC:DD:EE:FF
```

Wazne parametry strojenia:
- `--target-distance` docelowa odleglosc od taga [m]
- `--kp-forward`, `--kp-strafe`, `--kp-turn` wzmocnienia regulatora
- `--max-x`, `--max-y`, `--max-omega` limity komendy wysylanej do ESP32
- `--command-timeout` po ilu sekundach bez taga wysylac `STOP`
- `--show-preview` podglad kamery z aktualna komenda

Tryb domyslny jest headless (bez okna podgladu), co jest wygodne podczas jazdy robota bez laptopa.
Skrypt czeka na wykrycie taga i do tego czasu wysyla `STOP`.
Dodatkowo po polaczeniu BLE skrypt czeka na sygnal gotowosci z ESP32:
- `ready` z charakterystyki control, albo
- pierwsza ramke telemetryczna.

Dopiero po tym handshake uruchamiana jest petla sterowania z kamerki.

Limity predkosci sa liczone z geometrii i RPM:
- `--max-wheel-rpm` (domyslnie 220)
- `--wheel-diameter` (domyslnie 0.10 m)
- `--wheelbase-length` (domyslnie 0.260 m)
- `--track-width` (domyslnie 0.486 m)

Na starcie skrypt wypisuje policzone maksima `vx`, `vy`, `omega` oraz efektywne limity po uwzglednieniu `--max-x/--max-y/--max-omega`.

Parametry handshake BLE:
- `--ble-telemetry-uuid` UUID charakterystyki telemetrycznej
- `--ble-ready-timeout` timeout oczekiwania na gotowosc ESP po polaczeniu

## Start automatyczny po wlaczeniu zasilania robota (headless)

Jesli Raspberry Pi ma jechac bez monitora/klawiatury/myszki, uzyj uslugi systemd.

Skrypt BLE follow ma domyslnie:
- oczekiwanie na wykrycie taga,
- wysylanie `STOP`, gdy tag nie jest widoczny,
- docelowy dystans `0.50 m`.

Założenia dynamiki (dodane do konfiguracji):
- masa robota okolo 4-5 kg,
- predkosc maksymalna okolo 1 m/s,
- osiagniecie predkosci w okolo 1 s.

W praktyce to oznacza, ze na start zastosowane sa bezpieczne limity znormalizowane:
- `MAX_X_NORM=0.35`
- `MAX_Y_NORM=0.40`
- `MAX_OMEGA_NORM=0.35`

### Jednorazowa instalacja autostartu

```bash
cd RaspberryPi
bash scripts/setup.sh
bash scripts/install_autostart_service.sh
```

To tworzy i aktywuje usluge:
- `mecanosaurus-apriltag.service`

oraz plik konfiguracji:
- `/etc/default/mecanosaurus-apriltag`

### Co ustawic przed pierwsza jazda

1. Wpisz adres BLE napedu ESP32 do:
	- `/etc/default/mecanosaurus-apriltag` -> `BLE_ADDRESS=AA:BB:CC:DD:EE:FF`
2. Sprawdz sciezke kalibracji:
	- `CALIB_FILE=config/camera_calib.npz`
3. Zostaw `SHOW_PREVIEW=0` dla pracy bez monitora.

### Obsluga uslugi

```bash
sudo systemctl start mecanosaurus-apriltag.service
sudo systemctl status mecanosaurus-apriltag.service
sudo journalctl -u mecanosaurus-apriltag.service -f
```

Po tym robot powinien po prostu wystartowac po wlaczeniu zasilania pokladowego,
polaczyc sie przez BLE z napedem i czekac na wykrycie AprilTaga.

## Kalibracja kamerki pod AprilTag (polecane)

Jesli sześcian "plywa" albo orientacja wyglada niestabilnie, zrob kalibracje intrinsics + dystorsji.

### 1. Przygotuj plansze chessboard

- Wydrukuj plansze 10x7 (inner corners), czyli 11x8 squares, na sztywniejszej kartce.
- Upewnij sie, ze kwadraty maja rowny rozmiar (np. 25 mm).

### 2. Uruchom kalibracje

```bash
cd RaspberryPi
python3 src/camera_calibrate.py --camera 0 --cols 10 --rows 7 --square-mm 25 --samples 25 --out config/camera_calib.npz
```

W oknie kalibracji:
- ustawiaj plansze pod roznymi katami i pozycjami,
- naciskaj `SPACE`, gdy narozniki sa dobrze wykryte,
- po zebraniu probek skrypt zapisze `config/camera_calib.npz`.

### 3. Uruchom viewer z kalibracja

```bash
cd RaspberryPi
python3 src/camera_viewer.py \
	--camera 0 \
	--tag-family 36h11 \
	--tag-id 0 \
	--tag-size 0.12 \
	--calib-file config/camera_calib.npz \
	--smooth-alpha 0.35
```

Wskazowki strojenia:
- `--smooth-alpha 0.5` mniej wygladzania, szybsza reakcja.
- `--smooth-alpha 0.2` mocniejsze wygladzanie, mniej drgan.
- dla slabego swiatla ustaw nizsze `--quad-decimate`, np. `0.8`.
