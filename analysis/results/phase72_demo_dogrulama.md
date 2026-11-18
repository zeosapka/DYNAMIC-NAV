# Faz 7.2 — Demo Hazırlığı Doğrulama Kaydı

**Üretilen Belgeler:**
- [phase72_demo_senaryosu.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase72_demo_senaryosu.md)
- [phase72_anlatim_metni.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase72_anlatim_metni.md)
- [phase72_ekran_kaydi_kontrol_listesi.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase72_ekran_kaydi_kontrol_listesi.md)
- [phase72_demo_dogrulama.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase72_demo_dogrulama.md)

---

## 1. Uygulama ve Senaryo Çalışma Doğrulaması

`SDL_VIDEODRIVER=dummy` ortamında `DynamicNavApp(DEFAULT_CONFIG, initial_view=AppView.HOME)` başlatılarak tüm görünümler (`HOME`, `SIMULATION`, `GUIDE`) ve 8 senaryo doğrudan test edilmiştir (`ALL_SCENARIOS_AND_GUI_OK`):

| Senaryo | A\* Planlama (`success`) | Yol Noktası Sayısı | Yol Maliyeti (m) | Dinamik Engel Sayısı |
|---|---|---|---|---|
| `HEAD-ON` | `True` | `81` | `16,00` | `1` |
| `CROSSING` | `True` | `81` | `20,14` | `2` |
| `CORRIDOR` | `True` | `81` | `16,00` | `1` |
| `STOP-AND-GO` | `True` | `81` | `16,00` | `1` |
| `NARROW-GATE` | `True` | `81` | `16,00` | `1` |
| `MULTIPLE-DYNAMIC-OBSTACLES` | `True` | `81` | `16,00` | `4` |
| `SUDDEN-STOP` | `True` | `81` | `16,00` | `1` |
| `SUDDEN-STOP-FINITE-DECEL` | `True` | `81` | `16,00` | `0` |

> **Not:** `interface/app.py` arayüzündeki hazır butonlar `HEAD-ON` (`KARŞIDAN GELEN`), `CROSSING` (`ÇAPRAZ GEÇİŞ`), `CORRIDOR` (`KORİDOR`) ve `STOP-AND-GO` (`DUR-KALK`) senaryolarını doğrudan açar; `NARROW-GATE` senaryosu ise kaynak kod değiştirilmeden `phase72_ekran_kaydi_kontrol_listesi.md` içinde verilen tek satırlık salt-okunur Python başlatıcısıyla canlı Pygame penceresinde açılmaktadır.

---

## 2. Video Kuralları ve Veri Tutarlılığı Kontrolü

- **Süre:** `04:45` (`3–5` dakika aralığında).
- **Dil ve Altyazı:** Tamamen Türkçe seslendirme metni ve senkron Türkçe altyazı blokları hazırlandı.
- **Gerçek Görüntü & Veri İlkesi:** Yalnızca gerçek Pygame uygulaması ve Faz 6.5.6 denetim sonuçları (`191/191` pytest, `175,52 s`; `8/8` temel senaryo, `min_swept = 0,152024 m`, ort. DWA `11,85 ms`; `14/14` `NARROW-GATE` / `42` koşum, `0` çarpışma, `0` pay ihlali, `min_swept = 0,150019 m`, `%100` determinizm, medyan `5,58 ms`, P99 `232,48 ms`) kullanıldı.
- **Ekip Bilgileri:** Elektrik-Elektronik Mühendisliği — Eren Çakmaz, Ahmet Gürlek, Arda Başaran.
- **Salt-Okunur Koruma:** Kaynak kod, konfigürasyon, testler ve tarihsel deneysel sonuçlar değiştirilmedi.
