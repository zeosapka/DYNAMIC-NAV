# DYNAMIC-NAV

**Dinamik Engelden Kaçınma — Otonom 2B Mobil Robot Navigasyon Simülatörü**

DYNAMIC-NAV, 2B otonom mobil robot navigasyonu ve dinamik engelden kaçınma için deterministik, SI birimleriyle çalışan bir Python mühendislik simülatörüdür. Üç kişilik bir üniversite mühendislik projesi kapsamında geliştirilmiştir.

---

## 1. Proje Hakkında

Bu proje, statik ve hareketli engellerin bulunduğu 2B ortamlarda diferansiyel sürüşlü bir mobil robotun başlangıç konumundan hedef konumuna güvenli biçimde ulaşmasını sağlayan hiyerarşik bir navigasyon mimarisi sunar. Sistem, gizli gelecek senaryo zamanlamalarına erişmeden yalnızca zaman damgalı gözlemler üzerinden çalışacak şekilde tasarlanmıştır.

## 2. Projenin Amacı

- Küresel yol planlama (Global Path Planning) ile yerel engelden kaçınma (Local Obstacle Avoidance) katmanlarını modüler bir mimaride birleştirmek.
- Fiziksel simülasyon ile kullanıcı arayüzünü kesin sınırlarla ayırmak (`simulation/` ve `algorithms/` paketleri `pygame` içermez).
- Tüm senaryolarda katı güvenlik marjını (`safety_margin_m = 0.15 m`) hem anlık hem de sürekli süpürme boyunca minimum açıklık (Swept Clearance) düzeyinde doğrulamak.
- Tekrarlanabilir, deterministik (`dt_s = 0.05 s`) deneysel karşılaştırma ve regresyon test altyapısı sağlamak.

## 3. Sistem Mimarisi

Sistem, katı sorumluluk ayrımı ilkesine göre beş ana pakete ayrılmıştır:

| Paket / Dosya | Sorumluluk |
| :--- | :--- |
| `algorithms/astar.py` | 4/8-bağlantılı A\* küresel yol planlayıcı, engel şişirme (Obstacle Inflation) ve parça-kare mesafe (`segment_to_square_distance_m`) doğrulaması. |
| `algorithms/dwa.py` | Dinamik Pencere Yaklaşımı (Dynamic Window Approach, DWA) yerel planlayıcı, tam frenleme yörüngesi doğrulaması, P1–P5 geri dönüş hakemliği (Fallback Arbitration) ve küresel yeniden planlama (`replan_global_path_around_obstacles()`). |
| `algorithms/path_follower.py` | Engelden kaçınma içermeyen temel izleme denetleyicisi (`WaypointFollower`, `ASTAR_FOLLOWER`). |
| `algorithms/corridor_passing.py` | Nedensel koridor ve açık alan sollama değerlendirme bileşeni (`CorridorPassingManager`). |
| `algorithms/observation.py` | Zaman damgalı nedensel gözlem hattı (Causal Observation Pipeline, `ObservationPipeline`) ve sonlu fark/EMA tabanlı hız kestiricisi (`ObstacleStateEstimator`). |
| `simulation/` | Çerçeve-bağımsız SI fizik motoru (`entities.py`, `world.py`, `dynamic_obstacles.py`, `objects.py`, `geometry.py`, `coordinates.py`, `safety.py`, `scenarios.py`). |
| `interface/` | Pygame tabanlı görselleştirme, etkileşimli kontroller ve uygulama döngüsü (`app.py`, `renderer.py`, `controls.py`, `views.py`). |
| `analysis/` | Tekrarlanabilir kıyaslama koşucusu (`benchmark.py`), metrik hesaplayıcı (`metrics.py`), adım adım tanılama (`diagnostics.py`) ve deney çıktıları (`results/`). |
| `tests/` | `191` adet Pytest birim, nedensellik ve entegrasyon testi. |

## 4. Kullanılan Algoritmalar

1. **A\* Küresel Yol Planlama (A\* Global Path Planning)**: `20.0 m × 15.0 m` dünyayı `0.2 m` çözünürlüklü doluluk ızgarasına (Occupancy Grid) ayırır; robot yarıçapı ve güvenlik marjı toplamı (`0.50 m`) kadar şişirme uygulayarak statik ve gözlenen engellerden güvenli bir küresel referans yolu üretir.
2. **Dinamik Pencere Yaklaşımı (Dynamic Window Approach, DWA)**: İvmelenme sınırları dahilinde erişilebilir `(v, omega)` hız adaylarını örnekler; orta nokta kinematik entegrasyonu ile `1.2 s` ileri kestirim (`predict_dt_s = 0.1 s`, ilk simülasyon adımı `0.05 s` ayrıca doğrulanır) ve tam frenleme yörüngesi üzerinde sürekli statik/dinamik süpürme açıklığını denetler.
3. **Proaktif Koridor ve Açık Alan Sollaması (`CorridorPassingManager`)**: Histerezis ve soğuma (Cooldown) mekanizmalarıyla aynı yönde ilerleyen daha yavaş bir öncü engel tespit edildiğinde, yalnızca nedensel sabit-hız kestirimi kullanarak sol/sağ/A\* aday yolları üretir ve her adayı tam DWA + frenleme kontrolünden geçirir.
4. **Nedensel Gözlem Hattı (Causal Observation Pipeline)**: Kontrolcüye yalnızca `ObservationFrame` üzerinden gözlenen konum, kestirilen hız, gözlem yaşı (`observation_age_s`) ve belirsizlik artışı aktarılır; `DynamicObstacle` sınıfının gizli senaryo programlarına erişim kesinlikle engellenmiştir.

## 5. Robotun Hareket Modeli

Tek tekerlekli (Unicycle) kinematik model `simulation/entities.py` (`Robot`) ve `config.py` (`RobotConfig`, `DWAConfig`) içinde SI birimleriyle tanımlanmıştır:

- **Robot yarıçapı**: `radius_m = 0.35 m`
- **Doğrusal ve açısal hız sınırları**: `max_linear_speed_mps = 3.0 m/s`, `max_angular_speed_radps = 2.8 rad/s`, `min_reverse_speed_mps = -0.6 m/s`
- **İvmelenme ve frenleme sınırları**: `max_linear_accel_mps2 = 5.0 m/s²`, `max_angular_accel_radps2 = 16.0 rad/s²`, `max_brake_decel_mps2 = 5.0 m/s²`, `reaction_time_s = 0.10 s`
- **Kinematik entegrasyon**: Her `dt_s = 0.05 s` adımında orta nokta yönelim integrasyonu (`mid_theta = theta + 0.5 * omega * dt_s`) uygulanır.

## 6. Dinamik Engel Senaryoları

`simulation/scenarios.py` içinde tanımlı 8 deterministik kıyaslama senaryosu:

| Senaryo | Davranış | Açıklama |
| :--- | :--- | :--- |
| `HEAD-ON` | `HEAD_ON` | Hareketli engel `y = 7.5 m` ekseni boyunca robota karşıdan yaklaşır. |
| `CROSSING` | `CROSSING` | İki dinamik engel robotun rotasına dik doğrultuda farklı `x` konumlarından geçiş yapar. |
| `CORRIDOR` | `CORRIDOR` | Dinamik engel `3.0 m` genişliğindeki dar koridor (`x ∈ [5.0, 15.0]`) içinde devriye gezer; proaktif yanal sollama gerektirir. |
| `STOP-AND-GO` | `STOP_AND_GO` | Dikey geçiş yapan engel belirli aralıklarla durur ve yeniden hareket eder. |
| `NARROW-GATE` | `CORRIDOR` | Dar geçit (`x ∈ [8.0, 10.0]`) devriye gezen bir engel tarafından işgal edilir; güvenli bekleme mesafesi ve geçit çıkışı ayrımı gerektirir. |
| `MULTIPLE-DYNAMIC-OBSTACLES` | `CROSSING` / `STOP_AND_GO` | Dört eşzamanlı dinamik engel (aynı yönlü öncü, iki dik geçiş ve bir dur-kalk engel). |
| `SUDDEN-STOP` | `STOP_AND_GO` | Önde ilerleyen dairesel engel aniden durur ve ardından geri yönde hareket eder. |
| `SUDDEN-STOP-FINITE-DECEL` | `SimObject` (`STOP_AND_GO`) | Sonlu yavaşlama ivmesine sahip `OrientedRect` öncü araç durur ve yeniden hızlanır. |

## 7. Güvenlik ve Çarpışma Kontrolü

- **Güvenlik marjı (Safety Margin)**: `safety_margin_m = 0.15 m` (yüzeyden yüzeye minimum açıklık).
- **Anlık ve Süpürme Boyunca Minimum Açıklık (Instantaneous & Swept Clearance)**: `analysis/metrics.py` her adımda dünya sınırları, statik AABB engeller (`segment_to_square_distance_m`), dinamik dairesel engeller (`_swept_circle_clearance_m`) ve `SimObject` geometrileri için hem uç nokta hem de sürekli parça açıklığını hesaplar.
- **Tam Frenleme Doğrulaması**: DWA, seçilen her aday yörünge için `1.2 s` kestirim ufkunun ardından tam duruşa kadar olan frenleme yörüngesini de sürekli statik ve dinamik süpürme açıklığı ile doğrular.
- **Sonlanma Sınıflandırması**: `GOAL_REACHED` (`goal_tolerance_m = 0.25 m`), `COLLISION_ABORT` (`clearance <= 0.0 m`), `BLOCKED` / `PLANNING_FAILED` veya `TIMEOUT` (`60.0 s`).

## 8. Deneysel Sonuçlar (Faz 6.5.6 Denetimi)

Aşağıdaki sonuçlar `analysis/results/phase656_final_audit_report.md` kapsamında belgelenmiş test takımı ve senaryolar üzerinde bağımsız olarak doğrulanmıştır (keyfi modellenmemiş koşullarda evrensel çarpışmasızlık garantisi anlamına gelmez):

- **Tam Pytest Test Takımı (`pytest -q`)**: **`191 / 191` test başarılı**, `0` başarısız, `0` atlanmış (`175.52 s`).
- **Temel Senaryolar (`8 / 8`)**: Tüm senaryolarda `GOAL_REACHED`, **`0` fiziksel çarpışma**, **`0` güvenlik marjı ihlali**, minimum süpürme açıklığı **`0.152024 m`** (`STOP-AND-GO`).
- **`NARROW-GATE` Seviye 1 Pertürbasyonları (`14` varyant × `3` tekrar = `42` koşum)**: **`14 / 14` `GOAL_REACHED`**, **`0` fiziksel çarpışma**, **`0` güvenlik marjı ihlali**, **`%100` tekrarlanabilir determinizm**, minimum süpürme açıklığı **`0.150019 m`** (`NARROW-GATE__L1-DO-01__01 >= 0.150000 m`).
- **Hesaplama Gecikmesi Uyarısı**: 8 temel senaryoda ortalama DWA süresi `11.85 ms` (`P95 = 36.98 ms`, `maksimum = 246.88 ms`) ölçülmüştür. `42` adet `NARROW-GATE` koşumunda (`19.014` DWA çağrısı) medyan süre `5.58 ms` olmakla birlikte, kapı önü bekleme ve geri dönüş değerlendirmeleri sırasında **`P99` gecikmesi `232.48 ms`** düzeyine ulaşmış ve kaydedilen çağrıların **`%17.25`'i (`3.280 / 19.014`) `50 ms` bütçesini aşmıştır**.

### Güncel Hedefli Doğrulama (Koridor Geçişi ve `ABORT_HOLD` Sertleştirmesi)

- **Mimari Güncelleme**: `CorridorPassingManager` içine `IDLE` / `PASSING_ARMED` / `PASSING_ACTIVE` / `ABORT_HOLD` durum makinesi, kilitli geçiş ekseni üzerinde destek fonksiyonu tabanlı çoklu engel takibi (`ActivePassingObstacle`), aktif geçiş sırasında histerezis ve yan-seçim bağlılığı (`keep_committed_side`), yön-terslenmesi durumunda hedefli cooldown aşımı ve `DWAPlanner` içinde izole `ABORT_HOLD` frenleme koruması eklenmiştir.
- **Tam Pytest Test Takımı (`python -m pytest -v`)**: **`202 / 202` test başarılı** (`0` başarısız, `467.93 s`).
- **Hedefli Doğrulama Özeti**:
  - `tests/test_corridor_passing_state_machine.py`: **5 passed**
  - `tests/test_phase62_safe_waiting_reverse.py` (`CORRIDOR-False` dahil): **17 passed**
  - `tests/test_phase654_corridor_passing.py`: **9 passed**
  - `tests/test_phase6553_narrow_gate_standoff.py`: **7 passed**
  - `tests/test_phase80_reproduction.py` (`CORRIDOR__L1-DO-01__02` dahil): **6 passed** (`DO-01`: `goal_reached=True`, `col=0`, `mviol=0`, `min_swept_m=0.165947 m`, `completion_time=18.5 s`, deterministik).

## 9. Kurulum

**Python 3.12.10** ortamında `requirements.txt` (`pygame>=2.5.0`, `numpy>=1.26.0`, `matplotlib>=3.8.0`, `pytest>=8.0.0`) ile doğrulanmıştır:

```powershell
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```

## 10. Uygulamanın Çalıştırılması

```powershell
python main.py
```

Başsız (Headless) ortamlar için:

```powershell
$env:SDL_VIDEODRIVER = "dummy"
python main.py
```

## 11. Testlerin Çalıştırılması

Tüm test takımını çalıştırmak için:

```powershell
pytest -v
```

Faz 6.5.x nedensellik, koridor sollama, zamanlayıcı ve süpürme açıklığı regresyon testlerini çalıştırmak için:

```powershell
pytest tests/test_phase652_causal_observation.py tests/test_phase654_corridor_passing.py tests/test_phase6551_timer_fix.py tests/test_phase6553_narrow_gate_standoff.py tests/test_phase6555_open_space_overtaking.py tests/test_phase6557_static_swept_clearance.py -v
```

Başarılı bir koşumda `0` başarısız ve `0` atlanmış test raporlanır.

## 12. Proje Klasör Yapısı

```text
DYNAMIC-NAV/
├── main.py                  # Uygulama giriş noktası
├── config.py                # Merkezi SI ve arayüz yapılandırma veri sınıfları
├── requirements.txt         # Python bağımlılıkları
├── README.md                # Proje dokümantasyonu
├── AGENTS.md                # Mühendislik ve doğrulama kuralları
├── algorithms/              # A*, DWA, PathFollower, CorridorPassingManager, ObservationPipeline
├── simulation/              # Çerçeve-bağımsız SI fizik motoru, engeller, geometriler, senaryolar
├── interface/               # Pygame görselleştirme, kontroller, görünümler ve uygulama döngüsü
├── analysis/                # Kıyaslama koşucusu, süpürme açıklığı metrikleri, tanılama ve results/
├── tests/                   # 191 Pytest birim, nedensellik ve entegrasyon testi
└── assets/                  # Statik arayüz varlıkları
```

## 13. Sistemin Sınırlılıkları

- **Kapı Önü Bekleme Gecikmesi**: `NARROW-GATE` bekleme ve yanal tutunma adımlarında en kötü durum DWA süresi `50 ms` bütçesini aşabilmektedir (`P99 = 232.48 ms`, `%17.25 > 50 ms`).
- **Kapsam Dışı Pertürbasyonlar**: Faz 6.5.5'te tespit edilen `NARROW-GATE` dışındaki bazı Seviye 1 pertürbasyonları (`SUDDEN-STOP` zamanlama kaymaları, `CORRIDOR` başlangıç yönelim sapmaları) ile Seviye 2/3 güçlü pertürbasyonlar Faz 6.5.5.7 sonrası yeniden doğrulanmamıştır.
- **Sensör Gürültüsü**: `ObservationPipeline` gecikme ve belirsizlik parametrelerini desteklemekle birlikte Faz 6.5.6 denetimi varsayılan sıfır-gürültü (`ObservationConfig`) ayarlarıyla yürütülmüştür.

## 14. Gelecek Çalışmalar

- Bekleme bölgesi ışın tarama ve statik ızgara şişirme işlemlerinin önbelleklenmesiyle DWA `P99` gecikmesinin `50 ms` altına indirilmesi.
- Seviye 2 (güçlü pertürbasyonlar) ve Seviye 3 (çoklu `SimObject` birleşimleri) dayanıklılık testlerinin tamamlanması.
- Yapay zekâ destekli geliştirme şeffaflığı: Geliştirme sürecinde kod yazımı, yeniden düzenleme ve tanılama analizlerinde yapay zekâ araçlarından destek alınmıştır; tüm mimari kararlar, testler ve doğrulamalar proje ekibi tarafından incelenmiş ve onaylanmıştır.
