# DYNAMIC-NAV: Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi

**DYNAMIC-NAV**, iki boyutlu sürekli uzayda diferansiyel sürüşlü bir mobil robotun statik ve hareketli engellerden çarpışmasız kaçınmasını sağlayan, Uluslararası Birimler Sistemi (SI) tabanlı, deterministik bir navigasyon ve simülasyon sistemidir.

---

## 1. Projenin Amacı ve Kapsamı

Sistem; küresel ızgara tabanlı **A\*** yol planlamasını, kinematik olarak uygulanabilir **Dinamik Pencere Yaklaşımı (DWA)** yerel kaçınmasını, proaktif koridor geçiş durum makinesini (`CorridorPassingManager`) ve zaman damgalı **nedensel gözlem hattını** (`ObservationPipeline`) bir araya getirir. Tüm hareketler ve güvenlik kontrolleri `dt_s = 0.05 s` adımıyla ve gizli gelecek engel zamanlayıcılarına erişmeden yürütülür.

## 2. Nihai Demo Videosu

- **Nihai Demo Videosu (Türkçe Seslendirme + Gömülü Altyazı, `229.73 s` / `03:49.73`)**: [`docs/Demo_Video.mp4`](docs/Demo_Video.mp4)

## 3. Nihai Akademik Teknik Rapor

- **Teknik Rapor (Markdown)**: [`docs/DYNAMIC-NAV_Teknik_Rapor.md`](docs/DYNAMIC-NAV_Teknik_Rapor.md)
- **Teknik Rapor (PDF, 5 sayfa A4)**: [`docs/DYNAMIC-NAV_Teknik_Rapor.pdf`](docs/DYNAMIC-NAV_Teknik_Rapor.pdf)

## 4. Sistem Mimarisi

```text
DYNAMIC-NAV/
├── .github/workflows/tests.yml   # GitHub Actions CI (Python 3.12, pytest -v)
├── config.py                     # Tüm SI parametrelerinin tek doğruluk kaynağı
├── main.py                       # Uygulama giriş noktası
├── simulation/                   # Pygame-bağımsız deterministik fizik ve senaryo çekirdeği
├── algorithms/                   # A*, DWA, CorridorPassingManager, ObservationPipeline, PathFollower
├── interface/                    # Pygame görselleştirme ve kullanıcı arayüzü
├── analysis/                     # Sürekli süpürme açıklığı denetimi, kıyaslama ve results/ kanıtları
├── tests/                        # 202 otomatik Pytest birim, nedensellik ve entegrasyon testi
├── docs/                         # Nihai akademik teslim belgeleri ve demo videosu
└── archive/                      # Ara demo sürümleri ve tarihsel raporlar
```

## 5. A\*, DWA ve `CorridorPassingManager`

- **A\* Küresel Yol Planlayıcı (`algorithms/astar.py`)**: `0.2 m` çözünürlüklü `100 x 75` doluluk ızgarası üzerinde `r_R + d_safe = 0.50 m` Öklid engel şişirme, oktil sezgisel fonksiyonu (`octile_heuristic_m`) ve segment düzleştirme uygular.
- **DWA Yerel Planlayıcı (`algorithms/dwa.py`)**: `7 x 17 = 119` hız penceresi adayını `T_p = 1.2 s` ufukta benzetir; tepki süresi (`0.10 s`) ve frenleme ivmesi (`5.0 m/s^2`) ile tam duruş mesafesini doğrular, gerektiğinde güvenli bekleme ve geri kaçış önceliklerini çalıştırır.
- **Proaktif Koridor Geçiş Yöneticisi (`algorithms/corridor_passing.py`)**: `IDLE`, `PASSING_ARMED`, `PASSING_ACTIVE` ve `ABORT_HOLD` durum makinesi ile kilitli geçiş ekseni boyunca çoklu dinamik engeli (`ActivePassingObstacle`) izler; yön-terslenmesi durumunda nedensel cooldown aşımı ve `keep_committed_side` yan-şerit bağlılığı sağlar.

## 6. Dinamik Engel Senaryoları

`simulation/scenarios.py` içinde 8 deterministik senaryo tanımlıdır:
- `HEAD-ON`, `CROSSING`, `CORRIDOR`, `STOP-AND-GO`, `NARROW-GATE`, `MULTIPLE-DYNAMIC-OBSTACLES`, `SUDDEN-STOP`, `SUDDEN-STOP-FINITE-DECEL`.

## 7. Güvenlik ve Çarpışma Kontrolü

`analysis/metrics.py`, ardışık zaman adımları arasındaki sürekli hareketi tarayarak hem **anlık açıklığı** (`c_inst`) hem de **süpürülmüş açıklığı** (`c_swept`) hesaplar:
- `c_swept <= 0.0 m`: Fiziksel çarpışma
- `0.0 m < c_swept < 0.15 m`: Güvenlik payı ihlali (`d_safe = 0.15 m`)

## 8. Kurulum ve Çalıştırma

**Python 3.12.10** ortamında [`requirements.txt`](requirements.txt) (`pygame>=2.5.0`, `numpy>=1.26.0`, `matplotlib>=3.8.0`, `pytest>=8.0.0`) ile doğrulanmıştır:

```powershell
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

Başsız (Headless) ortamlar için `$env:SDL_VIDEODRIVER = "dummy"` ayarlanarak çalıştırılabilir.

## 9. Test Komutları ve Güncel Test Sonucu

Tüm test paketini çalıştırmak için:

```powershell
python -m pytest -v
```

- **Yerel Tam Test Paketi**: **`202 passed`**, `0 failed`, `0 skipped` (`467.93 s`).
- **GitHub Actions CI ([`.github/workflows/tests.yml`](.github/workflows/tests.yml))**: **`202 passed`**, `0 failed`, `0 skipped` (`Python 3.12`, `ubuntu-latest`).

## 10. Deneysel Sonuçlar

Detaylı deneysel özet için [`docs/Deneysel_Sonuclar.md`](docs/Deneysel_Sonuclar.md) belgesine bakınız:
- **8 Temel Senaryo**: Tamamında `GOAL_REACHED`, `0` fiziksel çarpışma, `0` güvenlik marjı ihlali.
- **`CORRIDOR__L1-DO-01__02` (`obs_speed_scale = 1.15`)**: `goal_reached = True`, `0` fiziksel çarpışma, `0` güvenlik marjı ihlali, `min_swept_m = 0.165947 m`, `completion_time_s = 18.5 s`, `%100` deterministik.
- Ham JSON/CSV deneysel kanıtlar [`analysis/results/`](analysis/results/) altında korunmaktadır.

## 11. Sınırlılıklar

- **Gerçek-Zaman Duvar Saati Gecikmesi**: `NARROW-GATE` testlerinde medyan DWA süresi `5.58 ms` olmakla birlikte `P99` gecikmesi `232.48 ms` düzeyine çıkmakta ve çağrıların `%17.25`'i `50 ms` bütçesini aşmaktadır; bu nedenle sert gerçek-zaman garantisi iddia edilmemektedir.
- **Simülasyon Ortamı**: Tüm doğrulamalar 2B simülasyonda yapılmıştır; fiziksel donanım veya saha robotu testi gerçekleştirilmemiştir.

## 12. Teslim Belgeleri

- Teknik Rapor: [`docs/DYNAMIC-NAV_Teknik_Rapor.md`](docs/DYNAMIC-NAV_Teknik_Rapor.md) / [`docs/DYNAMIC-NAV_Teknik_Rapor.pdf`](docs/DYNAMIC-NAV_Teknik_Rapor.pdf)
- Nihai Demo Videosu: [`docs/Demo_Video.mp4`](docs/Demo_Video.mp4)
- Deneysel Sonuçlar: [`docs/Deneysel_Sonuclar.md`](docs/Deneysel_Sonuclar.md)
- Yapay Zekâ Kullanım Beyanı: [`docs/AI_Kullanim_Beyani.md`](docs/AI_Kullanim_Beyani.md) / [`docs/AI_Kullanim_Beyani.pdf`](docs/AI_Kullanim_Beyani.pdf)
- Ekip Katkı Tablosu: [`docs/Ekip_Katki_Tablosu.md`](docs/Ekip_Katki_Tablosu.md) / [`docs/Ekip_Katki_Tablosu.pdf`](docs/Ekip_Katki_Tablosu.pdf)
- Teslim Kontrol Listesi: [`docs/Teslim_Kontrol_Listesi.md`](docs/Teslim_Kontrol_Listesi.md)

## 13. Ekip Bilgileri

- **Kurum:** Sakarya Uygulamalı Bilimler Üniversitesi — Elektrik-Elektronik Mühendisliği
- **Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran
- **Danışman / Teslim Tarihi / İmzalar:** [`docs/Ekip_Katki_Tablosu.md`](docs/Ekip_Katki_Tablosu.md) içinde `[EKİP ONAYI GEREKLİ]` olarak insan onayı beklemektedir.
