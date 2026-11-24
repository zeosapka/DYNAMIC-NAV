# DYNAMIC-NAV: Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi

**Kurum:** Sakarya Uygulamalı Bilimler Üniversitesi — Elektrik-Elektronik Mühendisliği  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  

---

## Türkçe Özet

Otonom mobil robotların dinamik, dar ve kısmen öngörülemez ortamlarda güvenli biçimde gezinmesi; algılama kısıtları, kinematik sınırlar ve hesaplama zaman bütçesi nedeniyle önemli bir mühendislik problemidir. Bu çalışmada, iki boyutlu sürekli uzayda diferansiyel sürüşlü bir mobil robotun statik ve hareketli engellerden çarpışmasız kaçınmasını sağlayan **DYNAMIC-NAV** simülasyon ve navigasyon sistemi tasarlanmış, iyileştirilmiş ve doğrulanmıştır. Sistem; katı Uluslararası Birimler Sistemi (SI) disiplinine sahip deterministik bir fizik çekirdeği, kesin Öklid engel şişirmeli 8-yönlü A\* küresel yol planlayıcısı (`algorithms/astar.py`) ve sürekli süpürme açıklığı doğrulamalı Dinamik Pencere Yaklaşımı (DWA) yerel planlayıcısını (`algorithms/dwa.py`) bütünleştirmektedir.

Algoritma geliştirme süreci Faz 5.0 (`ASTAR_DWA` `3/5` başarı, `2` çarpışma) ve Faz 6.2 (`6/6` başarı, `0` çarpışma, ancak `SUDDEN-STOP` senaryosunda `0,1390 m` güvenlik payı ihlali) üzerinden Faz 6.5.5 (`120` varyant, `360` koşum, `%85,0` başlangıç başarısı) dayanıklılık taramasıyla ilerletilmiştir. Yapılan hedefli düzeltmeler (zamanlayıcı, güvenli bekleme, açık alan sollaması ve sürekli statik AABB süpürme hizalaması) sonrasında Faz 6.5.6 tarihsel doğrulamasında `191/191` `pytest` testi (`175,52 s`), `8/8` temel senaryo (`min_swept = 0,152024 m`) ve `14/14` `NARROW-GATE` Seviye 1 varyantı (`42` koşum, `min_swept = 0,150019 m`, `%100` determinizm) sıfır çarpışma ve sıfır güvenlik payı ihlaliyle (`>= 0,150000 m`) tamamlanmıştır. Devamındaki Faz 8.0 sertleştirmesi ve `CorridorPassingManager` durum makinesi (`PASSING_ARMED`, `PASSING_ACTIVE`, `ABORT_HOLD`, `keep_committed_side`) ile `main` (`e3b6ac3`) üzerinde hem yerel tam test paketi hem de GitHub Actions (`Pytest (Python 3.12)`) **`202/202`** başarıyla geçmiş; `CORRIDOR__L1-DO-01__02` senaryosu da `0` çarpışma, `0` pay ihlali ve `min_swept = 0,165947 m` ile tamamlanmıştır. Bununla birlikte `NARROW-GATE` testlerinde `19.014` DWA çağrısının `%17,25`'inin (`3.280` çağrı) `50 ms` adım süresini aşması (`P99 = 232,48 ms`) nedeniyle sert gerçek-zaman garantisi iddia edilmemektedir.

**Anahtar Kelimeler:** Mobil Robot Navigasyonu, Dinamik Engel Kaçınması, A\* Yol Planlama, Dinamik Pencere Yaklaşımı (DWA), Sürekli Süpürme Açıklığı, Deterministik Simülasyon.

---

## 1. Giriş

Otonom mobil robotlar; depo lojistiği, endüstriyel taşıma ve servis robotiği gibi uygulamalarda statik haritaların yanı sıra ani durabilen, yön değiştirebilen veya dar geçitlerden karşı yönden yaklaşabilen dinamik engellere karşı da güvenli hareket etmek zorundadır. Yalnızca küresel ızgara planlayıcılar statik çevre için kısa yollar üretse de hız/ivme sınırlarını ve hareketli engelleri dikkate almaz; yalnızca yerel reaktif yöntemler ise dar koridorlarda yerel minimuma takılma riski taşır. **DYNAMIC-NAV**, küresel planlama ile kinematik olarak uygulanabilir yerel kaçınmayı hiyerarşik bir mimaride birleştirirken, fiziksel simülasyonu görselleştirme arayüzünden tamamen izole eder.

## 2. Problem Tanımı ve Proje Amacı

İki boyutlu sınırlı bir çalışma uzayı $\mathcal{W} = [0, W] \times [0, H] \subset \mathbb{R}^2$ ($W = 20,0\ \text{m}$, $H = 15,0\ \text{m}$) içinde yarıçapı $r_R = 0,35\ \text{m}$ olan dairesel bir mobil robot ele alınmaktadır. Ortamda eksen-hizalı dikdörtgen (AABB) statik engeller ve hareketli dairesel dinamik engeller bulunmaktadır. Amaç; robotun başlangıç konumundan hedef konumuna $\epsilon_g = 0,25\ \text{m}$ tolerans içinde ulaşmasını sağlarken tüm simülasyon boyunca (i) fiziksel çarpışmasızlık ($c_{\mathrm{swept}} > 0\ \text{m}$), (ii) minimum güvenlik payı korunumu ($c_{\mathrm{swept}} \ge d_{\mathrm{safe}} = 0,15\ \text{m}$) ve (iii) gizli gelecek yörünge bilgisi olmadan nedensel gözlem ilkesinin sağlanmasıdır.

## 3. Sistem Mimarisi

DYNAMIC-NAV dört modüler katmandan oluşmaktadır: (1) tüm SI parametrelerinin tek doğruluk kaynağı `config.py`, (2) `pygame` bağımsızlığı bulunmayan deterministik fizik ve senaryo çekirdeği `simulation/`, (3) 8-yönlü A\* ve DWA planlayıcılarını içeren `algorithms/`, (4) Pygame arayüzü `interface/` ve bağımsız sürekli süpürme denetçisi `analysis/`.

## 4. Matematiksel Model ve Algoritmalar

### 4.1. Robot Kinematiği ve Orta Nokta İntegrasyonu

Diferansiyel sürüşlü robotun sürekli zamanlı hareket modeli (`simulation/entities.py::Robot.step`) denklem (1) ile tanımlanır:

$$\dot{x}(t) = v(t)\cos\theta(t), \qquad \dot{y}(t) = v(t)\sin\theta(t), \qquad \dot{\theta}(t) = \omega(t) \tag{1}$$

Burada $(x, y)$ konum ($\text{m}$), $\theta \in [-\pi, \pi]$ yönelim ($\text{rad}$), $v \in [-0,6,\ 3,0]\ \text{m/s}$ doğrusal hız ve $\omega \in [-2,8,\ 2,8]\ \text{rad/s}$ açısal hızdır. Ayrık adım $\Delta t = 0,05\ \text{s}$ (`ExperimentConfig.dt_s`) üzerinde yönelim gecikmesini azaltmak için **orta nokta yönelim integrasyonu** uygulanır:

$$\theta_{k+1} = \mathrm{wrap}(\theta_k + \omega_k \Delta t), \qquad \theta_{\mathrm{mid}} = \theta_k + \frac{1}{2}\omega_k \Delta t \tag{2}$$

$$x_{k+1} = x_k + v_k \cos(\theta_{\mathrm{mid}})\Delta t, \qquad y_{k+1} = y_k + v_k \sin(\theta_{\mathrm{mid}})\Delta t \tag{3}$$

### 4.2. Küresel Yol Planlama — A\*

`algorithms/astar.py`, dünyayı $h = 0,2\ \text{m}$ çözünürlüklü $100 \times 75$ doluluk ızgarasına böler. Statik engeller, robot yarıçapı ve güvenlik payı toplamı ($r_R + d_{\mathrm{safe}} = 0,50\ \text{m}$) kadar kesin Öklid hücre-merkezi mesafesiyle şişirilir (`inflate_occupancy_grid`):

$$d_{\min}(\Delta r, \Delta c) = \sqrt{\max\left(0,\ |\Delta r|h - \frac{h}{2}\right)^2 + \max\left(0,\ |\Delta c|h - \frac{h}{2}\right)^2} < r_R + d_{\mathrm{safe}} \tag{4}$$

Arama sırasında toplam tahmini maliyet $f(n)$ (5) ve 8-yönlü kabul edilebilir **oktil sezgisel fonksiyonu** (`octile_heuristic_m`) (6) kullanılır (ortogonal geçiş $1,0$, çapraz geçiş $\sqrt{2}$):

$$f(n) = g(n) + h_{\mathrm{oct}}(n) \tag{5}$$

$$h_{\mathrm{oct}}(\Delta r, \Delta c) = \left(\max(|\Delta r|, |\Delta c|) + (\sqrt{2} - 1)\min(|\Delta r|, |\Delta c|)\right) \cdot h \tag{6}$$

Bulunan yol `is_segment_collision_free` ile düzleştirilir.

### 4.3. Yerel Dinamik Engel Kaçınması — DWA

`algorithms/dwa.py`, anlık $(v, \omega)$ değerinden ivme sınırları ($a_{\mathrm{lin}} = 5,0\ \text{m/s}^2$, $a_{\mathrm{ang}} = 16,0\ \text{rad/s}^2$) içinde ulaşılabilen dinamik hız penceresini (`compute_dynamic_window`) (7) hesaplar ve $7 \times 17 = 119$ aday çifte örnekler:

$$V_d = \left[\max(v_{\mathrm{bound}},\, v - a_{\mathrm{lin}}\Delta t),\ \min(v_{\max},\, v + a_{\mathrm{lin}}\Delta t)\right] \times \left[\max(-\omega_{\max},\, \omega - a_{\mathrm{ang}}\Delta t),\ \min(\omega_{\max},\, \omega + a_{\mathrm{ang}}\Delta t)\right] \tag{7}$$

Her aday $T_p = 1,2\ \text{s}$ ufukta $\Delta t_p = 0,1\ \text{s}$ adımlarla ileri benzetilir ve `score_trajectory` (8) ile puanlanır ($w_h = 1,3$, $w_c = 0,8$, $w_v = 1,8$, $w_p = 1,0$):

$$J(v, \omega) = w_h S_{\mathrm{head}} + w_c S_{\mathrm{clear}} + w_v S_{\mathrm{vel}} + w_p S_{\mathrm{path}} + S_{\mathrm{prog}} + P_{\mathrm{standstill}} \tag{8}$$

Tepki süresi $t_{\mathrm{react}} = 0,10\ \text{s}$ ve frenleme ivmesi $a_{\mathrm{brake}} = 5,0\ \text{m/s}^2$ için duruş mesafesi (`compute_stopping_distance_m`) (9) ile doğrulanır:

$$d_{\mathrm{stop}}(v) = |v|\, t_{\mathrm{react}} + \frac{v^2}{2 a_{\mathrm{brake}}} \tag{9}$$

### 4.4. Çarpışma Kontrolü ve Güvenlik Açıklığı

`analysis/metrics.py`, tek zaman adımındaki **anlık açıklık** ($c_{\mathrm{inst}}$, `compute_instantaneous_clearance_m`) ile ardışık adımlar arasındaki doğrusal süpürme boyunca hesaplanan **süpürülmüş açıklığı** ($c_{\mathrm{swept}}$, `compute_swept_clearance_m`) kesin biçimde ayırır. Statik AABB engellere karşı `segment_to_square_distance_m`, hareketli engellere karşı `swept_circle_min_distance_m` kullanılır. Güvenlik sınırı $d_{\mathrm{safe}} = 0,150000\ \text{m}$'dir; $c_{\mathrm{swept}} \le 0$ fiziksel çarpışma, $0 < c_{\mathrm{swept}} < 0,15\ \text{m}$ ise güvenlik payı ihlali olarak sınıflandırılır.

## 5. Deneysel Yöntem ve Test Ortamı

Değerlendirme `simulation/scenarios.py` içinde tanımlı 8 deterministik senaryo (`HEAD-ON`, `CROSSING`, `CORRIDOR`, `STOP-AND-GO`, `NARROW-GATE`, `MULTIPLE-DYNAMIC-OBSTACLES`, `SUDDEN-STOP`, `SUDDEN-STOP-FINITE-DECEL`) ve `NARROW-GATE` Seviye 1 dayanıklılık varyantları üzerinde yürütülmüştür. Ölçüm adımı $\Delta t = 0,05\ \text{s}$'dir; tüm metrikler (`goal_reached`, çarpışma/ihlal sayıları, $c_{\mathrm{inst}}$, $c_{\mathrm{swept}}$, süre, durma sayısı, determinizm ve DWA süreleri) `analysis/metrics.py` (`RunMetrics`) tarafından kaydedilmiştir.

## 6. Deneysel Sonuçlar

### 6.1. Algoritma Geliştirme Karşılaştırması (Faz 5.0 vs. Faz 6.2)

**Tablo 1**, aynı senaryo ve aynı ölçüm yöntemi altında Faz 5.0 ile Faz 6.2 arasındaki gelişimi göstermektedir. `ASTAR_FOLLOWER` tüm senaryolarda `COLLISION_ABORT` ile sonlanmıştır. `ASTAR_DWA` ise Faz 5.0'da `3/5` başarı (`NARROW-GATE` ve `SUDDEN-STOP`'ta çarpışma) verirken, Faz 6.2'de `6/6` `GOAL_REACHED` (`0` çarpışma) düzeyine çıkmış; ancak `SUDDEN-STOP` (`0,1390 m < 0,15 m`) ve `CROSSING` (`1` pay ihlali) durumlarında güvenlik payı ihlalleri devam etmiştir.

| Senaryo | Faz 5.0 `ASTAR_DWA` (Sonuç / Çarp. / İhlal / Süp. m) | Faz 6.2 `ASTAR_DWA` (Sonuç / Çarp. / İhlal / Süp. m / Süre s / Yol m) |
|---|---|---|
| `HEAD-ON` | `GOAL_REACHED` / 0 / 0 / `0,2605` | `GOAL_REACHED` / 0 / 0 / `0,2605` / `15,60` / `23,95` |
| `CROSSING` | `GOAL_REACHED` / 0 / 1 / `0,1500` | `GOAL_REACHED` / 0 / 1 / `0,1500` / `13,80` / `19,28` |
| `NARROW-GATE` | `COLLISION_ABORT` / 1 / 1 / `-0,0232` | `GOAL_REACHED` / 0 / 0 / `0,3398` / `15,55` / `24,13` |
| `MULTIPLE-DYN` | `GOAL_REACHED` / 0 / 0 / `0,1502` | `GOAL_REACHED` / 0 / 0 / `0,1502` / `17,55` / `25,34` |
| `SUDDEN-STOP` | `COLLISION_ABORT` / 1 / 3 / `-0,0299` | `GOAL_REACHED` / 0 / 2 / `0,1390` / `23,45` / `23,41` |
| `FINITE-DECEL` | *(Faz 5.0 kapsamı dışı)* | `GOAL_REACHED` / 0 / 1 / `0,1500` / `18,95` / `22,54` |

**Tablo 1.** Faz 5.0 ve Faz 6.2 `ASTAR_DWA` karşılaştırması (`ASTAR_FOLLOWER` her iki fazda `0/5` ve `0/6` `COLLISION_ABORT`).

### 6.2. Faz 6.5.5 Dayanıklılık Taraması ve Faz 6.5.6 Final Doğrulaması

Faz 6.5.5 Seviye 1 taramasında (`120` varyant, `360` koşum) başlangıç başarısı `%85,0` (`102/120`), çarpışma oranı `%15,0` (`18/120`) ve pay ihlali `%20,83` (`25/120`) olarak saptanmıştır. Zamanlayıcı düzeltmesi, güvenli bekleme, açık alan sollaması ve statik AABB süpürme hizalamasının ardından **Faz 6.5.6** nihai denetiminde **`191/191`** `pytest` testi (`175,52 s`) geçmiş; 8 temel senaryonun tamamı `0` çarpışma ve `0` ihlal ile `GOAL_REACHED` olmuştur (**Tablo 2**, **Şekil 1**).

| Temel Senaryo | Sonuç | Min. Anlık Açıklık (m) | Min. Süpürme Açıklığı (m) | Ort. DWA (ms) |
|---|---|---|---|---|
| `HEAD-ON` | `GOAL_REACHED` | `0,258924` | `0,257926` | `7,273` |
| `CROSSING` | `GOAL_REACHED` | `0,153208` | `0,152699` | `7,251` |
| `CORRIDOR` | `GOAL_REACHED` | `0,155297` | `0,155123` | `3,437` |
| `STOP-AND-GO` | `GOAL_REACHED` | `0,153300` | `0,152024` | `6,428` |
| `NARROW-GATE` | `GOAL_REACHED` | `0,154440` | `0,154363` | `44,470` |
| `MULTIPLE-DYNAMIC-OBSTACLES` | `GOAL_REACHED` | `0,153590` | `0,152792` | `8,608` |
| `SUDDEN-STOP` | `GOAL_REACHED` | `0,298817` | `0,298483` | `6,448` |
| `SUDDEN-STOP-FINITE-DECEL` | `GOAL_REACHED` | `0,154328` | `0,153625` | `10,915` |

**Tablo 2.** Faz 6.5.6 8 temel senaryo final sonuçları (`min_swept = 0,152024 m`, `0` çarpışma, `0` ihlal).

Temel testlerden ayrı yürütülen **Tablo 3**'teki `NARROW-GATE` Seviye 1 dayanıklılık testlerinde (`14` varyant × `3` tekrar = `42` koşum) `14/14` varyant başarılı olmuş, `0` çarpışma, `0` pay ihlali, `%100` determinizm ve `0,150019 m` minimum süpürme açıklığı ölçülmüştür.

| Test Grubu | Varyant | Koşum | Başarı Oranı | Çarpışma | Pay İhlali | Min. Süpürme (m) | Determinizm |
|---|---|---|---|---|---|---|---|
| `NARROW-GATE` Seviye 1 (Faz 6.5.6) | `14` | `42` | `14/14` (`%100`) | `0` | `0` | `0,150019` | `%100` |

**Tablo 3.** Faz 6.5.6 `NARROW-GATE` Seviye 1 dayanıklılık testi özeti (`19.014` DWA çağrısında medyan `5,58 ms`).

### 6.3. Nihai Sertleştirme ve DO-01 Doğrulama Notu (`main` — `e3b6ac3`)

Faz 6.5.6 tarihsel doğrulamasının ardından yürütülen Faz 8.0 sertleştirme sürecinde `CorridorPassingManager` (`algorithms/corridor_passing.py`) içine `IDLE / PASSING_ARMED / PASSING_ACTIVE / ABORT_HOLD` durum makinesi, çoklu engel takibi (`ActivePassingObstacle`), yön-terslenmesi cooldown aşımı ve `DWAPlanner` içine izole `ABORT_HOLD` frenleme koruması eklenmiştir. `CORRIDOR__L1-DO-01__02` (`obs_speed_scale = 1,15`) senaryosunda aktif geçiş sırasında `"right"` şeridinden `"left"` şeridine erken taraf değiştirme kök nedeni `keep_committed_side` önceliği ile giderilmiştir. Güncel `main` (`e3b6ac3`) üzerinde **`202/202`** `pytest` testi (yerel `467,93 s` ve GitHub Actions `Pytest (Python 3.12)` `176,40 s`) sıfır hatayla geçmiş; `CORRIDOR__L1-DO-01__02` senaryosunda `goal_reached = True`, `0` çarpışma, `0` güvenlik payı ihlali, `min_swept = 0,165947 m`, `completion_time = 18,5 s` ve `%100` determinizm doğrulanmıştır.

**Şekil 1**, Faz 6.2 `SUDDEN-STOP` (`0,1390 m`) ihlalinden Faz 6.5.6 temizliğine (`>= 0,152024 m`) geçişle birlikte anlık ve süpürülmüş açıklık farkını; **Şekil 2**, Faz 6.2.1 (`2.821` çağrı: ort. `19,15 ms`, medyan `16,04 ms`, P95 `43,52 ms`, P99 `66,49 ms`, maks. `131,87 ms`) ile Faz 6.5.6 `NARROW-GATE` (`P99 = 232,48 ms`, `%17,25 > 50 ms`) zaman bütçesi davranışını; **Şekil 3** ise farklı deney kapsamlarını birleştirmeden fazlar arası gelişimi göstermektedir. *(Vektör grafikler PDF sürümünde yer almaktadır.)*

## 7. Tartışma ve Sınırlılıklar

1. **Gerçek Zamanlı Duvar Saati Performansı vs. Deterministik Adım:** Simülasyon sabit $\Delta t = 0,05\ \text{s}$ (`50 ms`) adımıyla deterministik çalışmaktadır; ancak duvar saati (wall-clock) hesaplama süresinde Faz 6.2.1'de `%3,2` (`90/2.821`), Faz 6.5.6 `NARROW-GATE` testlerinde ise `3.280/19.014` (`%17,25`) çağrı `50 ms` bütçesini aşmıştır (`P99 = 232,48 ms`). Bu nedenle sert gerçek-zaman garantisi iddia edilmemektedir.
2. **Geometri ve Kestirim Kabulleri:** 2B dairesel robot geometrisi ve kısa ufuklu sabit hızlı engel ekstrapolasyonu kullanılmaktadır.

## 8. Sonuç

DYNAMIC-NAV, Faz 5.0'daki çarpışmalardan ve Faz 6.5.5'teki sınır ihlallerinden Faz 6.5.6 tarihsel denetiminde `191/191` `pytest` testine (`8/8` temel senaryo ve `14/14` `NARROW-GATE` Seviye 1 varyantı), ardından Faz 8.0 nihai sertleştirmesiyle `main` (`e3b6ac3`) üzerinde **`202/202`** `pytest` testine (yerel ve GitHub Actions) ve `CORRIDOR__L1-DO-01__02` dâhil sıfır çarpışma, sıfır güvenlik payı ihlali ve `%100` determinizm düzeyine ulaştırılmıştır.

## 9. Kaynaklar

1. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.
2. Fox, D., Burgard, W., & Thrun, S. (1997). The Dynamic Window Approach to Collision Avoidance. *IEEE Robotics & Automation Magazine*, 4(1), 23–33.
3. Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
4. DYNAMIC-NAV Proje Deposu: `config.py`, `simulation/entities.py`, `algorithms/astar.py`, `algorithms/dwa.py`, `analysis/metrics.py`, `analysis/results/benchmark_raw.json`, `benchmark_phase62_raw.json`, `profile_phase63_baseline.json`, `phase655_level1_summary.json`, `phase656_final_audit_report.md`.
