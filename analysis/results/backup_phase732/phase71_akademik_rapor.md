# DYNAMIC-NAV: Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi

**Üniversite:** Sakarya Uygulamalı Bilimler Üniversitesi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  

---

## Özet

Otonom mobil robotların dinamik, dar ve kısmen öngörülemez ortamlarda güvenli biçimde gezinmesi; algılama kısıtları, kinematik sınırlar ve gerçek zamanlı hesaplama bütçesi nedeniyle kritik bir mühendislik problemidir. Bu çalışmada, iki boyutlu sürekli uzayda diferansiyel sürüşlü bir mobil robotun statik ve hareketli engellerden çarpışmasız kaçınmasını sağlayan **DYNAMIC-NAV** simülasyon ve navigasyon sistemi tasarlanmış ve doğrulanmıştır. Sistem; katı Uluslararası Birimler Sistemi (SI) disiplinine sahip deterministik bir fizik çekirdeği, Öklid engel şişirmeli 8-yönlü A\* küresel yol planlayıcısı (`algorithms/astar.py`) ve sürekli süpürme açıklığı (continuous swept-clearance) doğrulamalı Dinamik Pencere Yaklaşımı (Dynamic Window Approach — DWA) yerel planlayıcısını (`algorithms/dwa.py`) bütünleştirmektedir.

Yerel planlayıcı; aktif frenleme duruş mesafesi kontrolü, bekleme ufku doğrulaması, öncelikli geri çekilme (fallback) hiyerarşisi ve dar geçit ile açık alan sollamalarını yöneten geçit yönetimini tek bir deterministik kontrol döngüsünde birleştirmektedir. Sistem, 8 temel dinamik engel senaryosu ve 14 adet `NARROW-GATE` Seviye 1 dayanıklılık varyantı (toplam 42 tekrarlı koşum) üzerinde bağımsız sürekli-zamanlı süpürme açıklığı denetçisiyle (`analysis/metrics.py`) değerlendirilmiştir. Faz 6.5.6 doğrulama sonuçlarına göre `191/191` otomatik `pytest` testi (`175,52 s`) başarıyla geçmiş; 8 temel senaryonun tamamında (`min_swept = 0,152024 m`) ve 14 `NARROW-GATE` Seviye 1 varyantının 42 koşumunda (`min_swept = 0,150019 m`) sıfır çarpışma, sıfır güvenlik payı ihlali (`>= 0,1500 m`) ve `%100` fiziksel determinizm elde edilmiştir.

**Anahtar Kelimeler:** Mobil Robot Navigasyonu, Dinamik Engel Kaçınması, A\* Yol Planlama, Dinamik Pencere Yaklaşımı (DWA), Sürekli Süpürme Açıklığı, Deterministik Simülasyon.

---

## 1. Giriş

Otonom mobil robotlar; depo lojistiği, endüstriyel taşıma ve servis robotiği gibi uygulamalarda yalnızca statik haritalara göre değil, ani durabilen, yön değiştirebilen veya dar geçitlerden karşı yönden yaklaşabilen dinamik engellere karşı da güvenli hareket etmek zorundadır. Klasik ızgara tabanlı küresel planlayıcılar statik çevre için geometrik olarak en kısa yolu üretebilse de robotun hız ve ivme sınırlarını ya da hareketli engellerin anlık yörüngelerini hesaba katmaz. Buna karşılık yalnızca yerel reaktif yöntemlere dayanmak, dar koridorlarda veya uzun engellerin arkasında yerel minimuma takılma ya da güvenlik payı ihlali riskini doğurur.

Bu gereksinimleri karşılamak amacıyla geliştirilen **DYNAMIC-NAV** projesi; küresel yol planlama ile kinematik olarak uygulanabilir yerel yörünge seçimini hiyerarşik bir mimaride birleştirirken, fiziksel simülasyon ile görselleştirme katmanını birbirinden tamamen ayırır. Çalışmanın temel mühendislik katkıları şunlardır: (i) tüm algoritma ve simülasyon modüllerinde yalnızca SI birimlerinin kullanılması, (ii) sürekli zamanlı süpürme açıklığı kontrolü ile `0,15 m` güvenlik payının ayrık zamanlı örnekleme hatalarına karşı korunması, (iii) dar geçitlerde güvenli bekleme ve açık alanda proaktif sollama davranışlarının deterministik biçimde sağlanması.

---

## 2. Problem Tanımı

İki boyutlu sınırlı bir çalışma uzayı $\mathcal{W} = [0, W] \times [0, H] \subset \mathbb{R}^2$ ($W = 20,0\ \text{m}$, $H = 15,0\ \text{m}$) içerisinde yarıçapı $r_R = 0,35\ \text{m}$ olan dairesel bir mobil robot ele alınmaktadır. Ortamda eksen-hizalı dikdörtgen (AABB) statik engeller kümesi $\mathcal{O}_{\mathrm{stat}}$ ve zamanla konum değiştiren dairesel dinamik engeller kümesi $\mathcal{O}_{\mathrm{dyn}}(t)$ bulunmaktadır.

Problem; robotun başlangıç konumundan $\mathbf{p}_0 = (x_0, y_0)^T$ hedef konumuna $\mathbf{p}_g = (x_g, y_g)^T$ tolerans yarıçapı $\epsilon_g = 0,25\ \text{m}$ içinde ulaşmasını sağlarken, tüm simülasyon süresi $t \in [0, T_{\max}]$ boyunca aşağıdaki güvenlik ve kinematik kısıtların sağlanmasıdır:
- **Çarpışmasızlık:** Hiçbir statik veya dinamik engelle ya da dünya sınırıyla fiziksel temas olmaması (yüzey-yüzeye açıklık $c_{\mathrm{surf}}(t) > 0\ \text{m}$).
- **Güvenlik Payı Korunumu:** Planlanan ve izlenen hareket boyunca minimum yüzey açıklığının $d_{\mathrm{safe}} = 0,15\ \text{m}$ sınırının altında kalmaması.
- **Nedensel Gözlem İlkesi:** Kontrolcü kararlarının gizli gelecek engel zaman çizelgelerine erişmeden yalnızca anlık ve geçmiş gözlemlere dayanması.

---

## 3. Sistem Mimarisi

DYNAMIC-NAV, sorumlulukların katı biçimde ayrıldığı dört temel katmandan oluşmaktadır (**Şekil 1**):

1. **Merkezi Konfigürasyon (`config.py`):** Tüm dünya, robot, planlayıcı ve deney parametrelerinin tek doğruluk kaynağıdır (`WorldConfig`, `RobotConfig`, `DWAConfig`, `ExperimentConfig`).
2. **Simülasyon Çekirdeği (`simulation/`):** `entities.py`, `world.py` ve `scenarios.py` modüllerinden oluşur. `pygame` kütüphanesini hiçbir şekilde içe aktarmaz; robot kinematiğini ve engelleri $\Delta t = 0,05\ \text{s}$ zaman adımıyla ilerletir.
3. **Planlama Algoritmaları (`algorithms/`):** `astar.py` 8-yönlü A\* küresel yolunu üretir ve çizgi-parçası tabanlı düzleştirme uygular; `dwa.py` ise dinamik hız penceresini örnekler, frenleme ve süpürme açıklığı doğrulamasını yapar ve geri çekilme (fallback) kararlarını yönetir.
4. **Arayüz ve Analiz (`interface/`, `analysis/`):** `interface/` Pygame tabanlı görselleştirme ve kullanıcı etkileşimini sağlar; `analysis/` ise simülasyondan bağımsız olarak sürekli süpürme açıklığı (`compute_swept_clearance_m`), gecikme ve determinizm metriklerini hesaplar.

**Şekil 1.** DYNAMIC-NAV dört katmanlı sistem mimarisi ve modüller arası veri akışı. *(Vektör diyagram PDF sürümünde yer almaktadır.)*

---

## 4. Matematiksel Modelleme ve Robot Kinematiği

Robotun sürekli zamanlı tek-tekerlekli/diferansiyel sürüş (unicycle/differential-drive) hareket modeli aşağıdaki diferansiyel denklemlerle tanımlanır (`simulation/entities.py`):

$$\dot{x}(t) = v(t)\cos\theta(t), \qquad \dot{y}(t) = v(t)\sin\theta(t), \qquad \dot{\theta}(t) = \omega(t) \tag{1}$$

Burada $(x, y)$ robot merkezinin metre ($\text{m}$) cinsinden konumunu, $\theta \in [-\pi, \pi]$ radyan ($\text{rad}$) cinsinden yönelim açısını, $v \in [-0,6,\ 3,0]\ \text{m/s}$ doğrusal hızı ve $\omega \in [-2,8,\ 2,8]\ \text{rad/s}$ açısal hızı göstermektedir.

Ayrık simülasyon adımı $\Delta t = 0,05\ \text{s}$ (`ExperimentConfig.dt_s`) üzerinde `Robot.step` metodu, birinci dereceden Euler yaklaşımındaki yönelim gecikmesi hatasını azaltmak amacıyla **orta nokta yönelim integrasyonu** (midpoint heading integration) uygular:

$$\theta_{k+1} = \mathrm{wrap}\!\left(\theta_k + \omega_k \Delta t\right), \qquad \theta_{\mathrm{mid}} = \theta_k + \frac{1}{2}\omega_k \Delta t \tag{2}$$

$$x_{k+1} = x_k + v_k \cos(\theta_{\mathrm{mid}})\Delta t, \qquad y_{k+1} = y_k + v_k \sin(\theta_{\mathrm{mid}})\Delta t \tag{3}$$

Burada $\mathrm{wrap}(\cdot)$ açıyı $[-\pi, \pi]$ aralığına indirger; $\theta_{\mathrm{mid}}$ ise $[t_k, t_{k+1}]$ aralığındaki ortalama yönelimi temsil ederek dairesel yaylar boyunca konum sapmasını azaltır.

---

## 5. Küresel Yol Planlama — A\*

`algorithms/astar.py`, $20,0\ \text{m} \times 15,0\ \text{m}$ boyutlarındaki dünyayı $h = 0,2\ \text{m}$ çözünürlüklü bir doluluk ızgarasına böler ($100 \times 75$ hücre, `WorldConfig`). Statik engeller, robot yarıçapı ve güvenlik payı toplamı kadar ($r_R + d_{\mathrm{safe}} = 0,35 + 0,15 = 0,50\ \text{m}$) kesin Öklid hücre-merkezi mesafesiyle şişirilir (`inflate_occupancy_grid`):

$$d_{\min}(\Delta r, \Delta c) = \sqrt{\max\!\left(0,\ |\Delta r|h - \frac{h}{2}\right)^2 + \max\!\left(0,\ |\Delta c|h - \frac{h}{2}\right)^2} < r_R + d_{\mathrm{safe}} \tag{4}$$

Burada $\Delta r$ ve $\Delta c$ sırasıyla satır ve sütun farklarını, $h = 0,2\ \text{m}$ hücre boyutunu belirtir. Arama sırasında her $n$ düğümünün toplam tahmini maliyet fonksiyonu şu şekildedir:

$$f(n) = g(n) + h_{\mathrm{oct}}(n) \tag{5}$$

Burada $g(n)$ başlangıç hücresinden $n$ düğümüne kadar biriken gerçek ızgara maliyetini (ortogonal geçiş $1,0$, çapraz geçiş $\sqrt{2}$), $h_{\mathrm{oct}}(n)$ ise 8-yönlü hareket için kabul edilebilir (admissible) **oktil sezgisel fonksiyonunu** (`octile_heuristic_m`) ifade eder:

$$h_{\mathrm{oct}}(\Delta r, \Delta c) = \left(\max(|\Delta r|, |\Delta c|) + (\sqrt{2} - 1)\min(|\Delta r|, |\Delta c|)\right) \cdot h \tag{6}$$

8-yönlü ızgara üzerinde $h_{\mathrm{oct}}$ gerçek en kısa ızgara maliyetini asla aşmadığından bulunan yol ızgara çözünürlüğünde en kısadır; ardından `is_segment_collision_free` ve `segment_to_square_distance_m` ile gereksiz ara düğümlerden arındırılarak düzleştirilir.

---

## 6. Yerel Dinamik Engel Kaçınması — DWA

`algorithms/dwa.py`, kontrol döngüsünde robotun anlık $(v, \omega)$ hızından doğrusal ve açısal ivme sınırları ($a_{\mathrm{lin}} = 5,0\ \text{m/s}^2$, $a_{\mathrm{ang}} = 16,0\ \text{rad/s}^2$) içinde $\Delta t$ adımında ulaşabileceği dinamik hız penceresini hesaplar (`compute_dynamic_window`):

$$V_d = \left[\max(v_{\mathrm{bound}},\, v - a_{\mathrm{lin}}\Delta t),\ \min(v_{\max},\, v + a_{\mathrm{lin}}\Delta t)\right] \times \left[\max(-\omega_{\max},\, \omega - a_{\mathrm{ang}}\Delta t),\ \min(\omega_{\max},\, \omega + a_{\mathrm{ang}}\Delta t)\right] \tag{7}$$

Bu pencere $7 \times 17 = 119$ aday $(v, \omega)$ çiftine örneklenir (`linear_samples = 7`, `angular_samples = 17`) ve her aday $T_p = 1,2\ \text{s}$ öngörü ufku boyunca $\Delta t_p = 0,1\ \text{s}$ adımlarla ileri benzetilir. Kabul edilebilir aday yörüngeler `score_trajectory` fonksiyonuyla değerlendirilir (`DWAConfig`):

$$J(v, \omega) = w_h S_{\mathrm{head}} + w_c S_{\mathrm{clear}} + w_v S_{\mathrm{vel}} + w_p S_{\mathrm{path}} + S_{\mathrm{prog}} + P_{\mathrm{standstill}} \tag{8}$$

Burada ağırlıklar $w_h = 1,3$, $w_c = 0,8$, $w_v = 1,8$ ve $w_p = 1,0$'dır. Terimler sırasıyla yönelim uyumunu ($S_{\mathrm{head}} = 1 - |\mathrm{wrap}(\theta_{\mathrm{des}} - \theta_{\mathrm{end}})|/\pi$), üst sınırı $c_{\mathrm{cap}} = 0,65\ \text{m}$ olan açıklık puanını ($S_{\mathrm{clear}} = \min(\max(0, c_{\min}), c_{\mathrm{cap}})/c_{\mathrm{cap}}$), normalize doğrusal hızı ($S_{\mathrm{vel}} = v / v_{\max}$), küresel yola yakınlığı ($S_{\mathrm{path}} = 1 - \min(1, d_{\mathrm{path}} / 2,0)$), $[-1, 1]$ aralığına sıkıştırılmış hedefe ilerleme puanını ($S_{\mathrm{prog}}$) ve hedefe $0,35\ \text{m}$'den uzakken $v < 0,15\ \text{m/s}$ kalma durumunda uygulanan $P_{\mathrm{standstill}} = -1,0$ durağanlık cezasını temsil eder.

Ayrıca her adayın frenleme yapılabilirliğini doğrulamak için tepki süresi $t_{\mathrm{react}} = 0,10\ \text{s}$ ve frenleme ivmesi $a_{\mathrm{brake}} = 5,0\ \text{m/s}^2$ ile duruş mesafesi (`compute_stopping_distance_m`) hesaplanır:

$$d_{\mathrm{stop}}(v) = |v|\, t_{\mathrm{react}} + \frac{v^2}{2 a_{\mathrm{brake}}} \tag{9}$$

---

## 7. Çarpışma Algılama ve Güvenlik Kısıtları

Güvenlik değerlendirmesi (`analysis/metrics.py`, `algorithms/dwa.py`), **anlık açıklık** ($c_{\mathrm{inst}}$) ile **süpürülmüş açıklık** ($c_{\mathrm{swept}}$) arasında kesin ayrım yapar:
- **Anlık Açıklık (`compute_instantaneous_clearance_m`):** Robotun tek bir $t_k$ anındaki konumunda dünya sınırları, statik AABB engeller (`compute_static_obstacle_clearance`) ve dinamik engellerle olan en küçük yüzey-yüzeye Öklid mesafesidir (merkez mesafesinden robot ve engel yarıçapları çıkarılarak elde edilir).
- **Süpürülmüş Açıklık (`compute_swept_clearance_m`):** Ardışık $(t_k, t_{k+1})$ zaman adımları arasındaki doğrusal hareket parçası boyunca hesaplanan minimum yüzey-yüzeye açıklıktır; statik AABB'lere karşı `segment_to_square_distance_m` ile, hareketli dairesel engellere karşı ise `swept_circle_min_distance_m` ile sürekli zamanlı tünel geçme (tunneling) hatalarını önler.

Faz 6.5.5.7 düzenlemesiyle `evaluate_trajectory_with_braking()` içindeki statik AABB süpürme açıklığı hesabı `compute_swept_clearance_m()` ile geometrik olarak birebir hizalanmıştır. Güvenli aday bulunamadığında `SAFE_STANDSTILL_BRAKING` ($T_w = 2,5\ \text{s}$ bekleme ufku) ve `CorridorPassingManager` (dar geçit beklemesi ve açık alan proaktif sollaması) devreye girer.

---

## 8. Deneysel Senaryolar ve Değerlendirme Metrikleri

Sistem başarımı `simulation/scenarios.py` içinde tanımlanan 8 temel senaryo ve Faz 6.5.6 denetimindeki 14 `NARROW-GATE` Seviye 1 varyantı üzerinde değerlendirilmiştir (**Tablo 1**).

| Senaryo Adı | Test Edilen Fiziksel Etkileşim |
|---|---|
| `HEAD-ON` | Karşı yönden doğrudan yaklaşan dinamik engele karşı yanal kaçınma |
| `CROSSING` | Rotayı dik veya eğik açıyla kesen hareketli engellere karşı hız ve yönelim uyarlaması |
| `CORRIDOR` | Dar duvarlar arasında statik duvar güvenlik payını koruyarak dinamik engelle geçiş |
| `STOP-AND-GO` | Dur-kalk hareketi yapan engelin arkasında güvenli takip mesafesi korunumu |
| `NARROW-GATE` | Dar geçitte öncelik yönetimi, güvenli bekleme ve geçit çıkışı sonrası ilerleme |
| `MULTIPLE-DYNAMIC-OBSTACLES` | Aynı anda birden fazla dinamik engelin bulunduğu ortamda eşzamanlı kaçınma |
| `SUDDEN-STOP` | Ön engelin aniden durması durumunda tepki süresi ve frenleme yeterliliği |
| `SUDDEN-STOP-FINITE-DECEL` | Sonlu yavaşlama ivmesiyle duran engele karşı hız kestirimi ve pay korunumu |

**Tablo 1.** DYNAMIC-NAV temel doğrulama senaryoları ve test ettikleri etkileşim türleri.

`analysis/metrics.py` (`RunMetrics`) kapsamında ölçülen metrikler: hedefe ulaşma durumu (`goal_reached`), fiziksel çarpışma sayısı ($c_{\mathrm{swept}} \le 0$), güvenlik payı ihlal sayısı ($0 < c_{\mathrm{swept}} < 0,15\ \text{m}$), minimum anlık/süpürülmüş açıklık ($\text{m}$), görev süresi (`completion_time_s`, $\text{s}$), durma sayısı (`stop_count`), çoklu tekrarlı fiziksel determinizm ve DWA hesaplama süreleri ($\text{ms}$).

---

## 9. Deneysel Sonuçlar

Tüm deneysel veriler **Faz 6.5.6 — Nihai Sistem Bütünlüğü ve Regresyon Denetimi** kapsamında salt-okunur olarak doğrulanmıştır.

### 9.1. Otomatik Test Süiti ve Temel Senaryo Sonuçları

`pytest` test süitinde **`191/191`** test (`0` hata, `0` atlanan test) toplam `175,52 s` sürede geçmiştir. 8 temel senaryonun tamamı `GOAL_REACHED` durumuyla sonlanmış; hiçbir fiziksel çarpışma veya `0,1500 m` güvenlik payı ihlali gözlenmemiştir (**Tablo 2**, **Şekil 2**).

| Temel Senaryo | Sonuç | Min. Süpürme Açıklığı (m) | Ort. DWA Süresi (ms) |
|---|---|---|---|
| `HEAD-ON` | `GOAL_REACHED` | `0,257926` | `7,273` |
| `CROSSING` | `GOAL_REACHED` | `0,152699` | `7,251` |
| `CORRIDOR` | `GOAL_REACHED` | `0,155123` | `3,437` |
| `STOP-AND-GO` | `GOAL_REACHED` | `0,152024` | `6,428` |
| `NARROW-GATE` | `GOAL_REACHED` | `0,154363` | `44,470` |
| `MULTIPLE-DYNAMIC-OBSTACLES` | `GOAL_REACHED` | `0,152792` | `8,608` |
| `SUDDEN-STOP` | `GOAL_REACHED` | `0,298483` | `6,448` |
| `SUDDEN-STOP-FINITE-DECEL` | `GOAL_REACHED` | `0,153625` | `10,915` |

**Tablo 2.** Faz 6.5.6 temel senaryo sonuçları (ortalama DWA süresi ortalaması `11,85 ms`, ortalama P95 `36,98 ms`, maksimum `246,88 ms`).

**Şekil 2.** Temel senaryolarda minimum süpürme açıklığı ($\text{m}$) ve ortalama DWA hesaplama süresi ($\text{ms}$) karşılaştırması. *(Vektör grafik PDF sürümünde yer almaktadır.)*

### 9.2. NARROW-GATE Seviye 1 Dayanıklılık Testleri

Temel senaryo testlerinden ayrı olarak yürütülen `14` adet `NARROW-GATE` Seviye 1 varyantının her biri 3 bağımsız tekrar ile (toplam `42` koşum) sınanmıştır:
- **Tamamlanma ve Güvenlik:** `14/14` varyant (`42/42` koşum) `GOAL_REACHED` ile tamamlanmış; `0` fiziksel çarpışma ve `0` güvenlik payı ihlali kaydedilmiştir.
- **Kritik Açıklık ve Determinizm:** En kritik varyant olan `NARROW-GATE__L1-DO-01__01` koşumunda minimum süpürme açıklığı `0,150019 m` ölçülmüş; test edilen tüm varyantlarda `%100` fiziksel determinizm doğrulanmıştır.
- **Hesaplama Performansı:** `42` koşum boyunca yapılan `19.014` DWA çağrısında medyan süre `5,58 ms` olarak ölçülmüştür.

---

## 10. Tartışma ve Sınırlılıklar

1. **Hesaplama Gecikmesi Kuyruğu (Faz 6.5.6 Performans Uyarısı):** Medyan DWA süresi (`5,58 ms`) `50 ms` kontrol adımının oldukça altında olmakla birlikte, yoğun dar geçit etkileşimlerinde bekleme ufku ve sürekli süpürme doğrulamaları nedeniyle P99 gecikmesi `232,48 ms`'ye çıkmakta ve `19.014` çağrının `3.280`'i (`%17,25`) `50 ms` adım süresini aşmaktadır. Bu nedenle sistem şu hâliyle sert gerçek-zaman (hard real-time) garantisi iddiasında bulunmamaktadır; uzamsal indeksleme ve erken aday budaması ile kuyruk gecikmesi azaltılabilir.
2. **İki Boyutlu Dairesel Geometri Kabulu:** Robot modeli dairesel gövde ve düzlemsel kinematik üzerine kuruludur; farklı gövde biçimleri için çoklu daire yaklaşımı gerekmektedir.
3. **Kısa Ufuklu Sabit Hızlı Engel Kestirimi:** Yerel planlayıcı dinamik engellerin gelecek konumlarını sabit hız ve belirsizlik payı ile kestirmektedir; karmaşık insan/araç niyeti tahmini kapsam dışındadır.

---

## 11. Sonuç ve Ekip Görev Dağılımı

DYNAMIC-NAV projesi; katı SI birim mimarisi, orta nokta kinematiği, Öklid şişirmeli 8-yönlü A\* küresel planlayıcısı ve sürekli süpürme açıklığı doğrulamalı DWA yerel planlayıcısı ile dinamik ortamlarda güvenli mobil robot navigasyonu için tutarlı bir mühendislik çözümü sunmaktadır. Faz 6.5.6 denetiminde `191/191` `pytest` testi, `8/8` temel senaryo ve `14/14` `NARROW-GATE` Seviye 1 varyantı (`42` koşum) sıfır çarpışma, sıfır güvenlik payı ihlali ve `%100` determinizm ile doğrulanmıştır.

| Ekip Üyesi | Odak Alanı | İlgili Modüller / Çıktılar |
|---|---|---|
| **Eren Çakmaz** | Simülasyon Çekirdeği, Kinematik ve Küresel Planlama | `config.py`, `simulation/entities.py`, `simulation/world.py`, `algorithms/astar.py` |
| **Ahmet Gürlek** | Yerel Planlama (DWA), Güvenlik ve Arayüz | `algorithms/dwa.py`, `simulation/scenarios.py`, `interface/` |
| **Arda Başaran** | Deneysel Kıyaslama, Test Süiti ve Dokümantasyon | `analysis/`, `tests/`, `README.md`, Faz 6.5.6–7.3 raporları |

**Tablo 3.** Proje ekibi görev dağılımı (her üç üye ortak geliştirme, test ve değerlendirme süreçlerine katkı sağlamıştır).

---

## 12. Kaynakça

1. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.
2. Fox, D., Burgard, W., & Thrun, S. (1997). The Dynamic Window Approach to Collision Avoidance. *IEEE Robotics & Automation Magazine*, 4(1), 23–33.
3. Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
4. DYNAMIC-NAV Proje Deposu: `config.py`, `simulation/entities.py`, `algorithms/astar.py`, `algorithms/dwa.py`, `analysis/metrics.py`, `README.md` ve `analysis/results/phase656_final_audit_report.md`.
