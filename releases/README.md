# Mecanosaurus – Pobieranie aplikacji

Tutaj znajdziesz gotowe buildy aplikacji mobilnej gotowe do zainstalowania bezpośrednio na telefonie.

---

## Android (APK)

> Wymaga zezwolenia na instalację z nieznanych źródeł.

1. Na telefonie Android otwórz ten link i pobierz najnowszy plik `.apk`:
   - [Mecanosaurus-v1.0.0.apk](./android/Mecanosaurus-v1.0.0.apk)
2. W ustawieniach telefonu włącz **„Instaluj z nieznanych źródeł"** (Ustawienia → Bezpieczeństwo lub Prywatność).
3. Otwórz pobrany plik i postępuj zgodnie z instrukcjami instalatora.

---

## iOS (IPA)

> Bezpośrednia instalacja na iOS wymaga podpisanego certyfikatu Enterprise lub dystrybucji przez TestFlight.

Aktualnie aplikacja iOS jest dostępna wyłącznie przez **TestFlight** lub wymaga własnego podpisania.
Skontaktuj się z administratorem projektu, żeby uzyskać dostęp.

---

## Jak dodać nowy build (dla deweloperów)

### Android
```bash
# W katalogu app/
flutter build apk --release
# Skopiuj plik do releases/android/
copy app\build\app\outputs\flutter-apk\app-release.apk releases\android\Mecanosaurus-vX.Y.Z.apk
```

### iOS
```bash
flutter build ipa --release
# Wyeksportuj .ipa z Xcode i wrzuć do releases/ios/
```

Zaktualizuj link w tym README po dodaniu nowego pliku.

---

## Historia wersji

| Wersja | Data | Platforma | Zmiany |
|--------|------|-----------|--------|
| v1.0.0 | 2026-06-17 | Android | Pierwszy oficjalny release |
