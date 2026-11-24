# Faz 7.2 — DYNAMIC-NAV Türkçe Demo Video Senaryosu

**Proje:** DYNAMIC-NAV: Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Ekip:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  
**Hedef Süre:** `229.73 saniye` (`03:49.73`, 3–5 dakika kuralına tam uyumlu)  
**Dil ve Altyazı:** Tamamen Türkçe seslendirme + senkron Türkçe altyazı  

---

## Zaman Çizelgesi ve Sahne Planı

| Zaman Aralığı | Bölüm | Ekrandaki Görüntü (Gerçek Uygulama / Veri) | Gösterilecek Teknik İçerik |
|---|---|---|---|
| **0:00 – 0:30** | **1. Proje Tanıtımı** | `main.py` başlatılır; **ANA MENÜ** (`AppView.HOME`) ekranı görüntülenir. | Proje başlığı, Elektrik-Elektronik Mühendisliği ekip bilgisi, dinamik ortamlarda otonom mobil robot navigasyonu problemi ve SI birimli deterministik simülatör amacı. |
| **0:30 – 1:15** | **2. Sistem Mimarisi ve A\*** | Sağ karttaki `ALGORİTMA BİLGİSİ` (`AppView.GUIDE`) sekmesi kısaca gösterilir; ardından **SİMÜLASYON** ekranına geçilip `A* PLANLA` ve `8-YÖN` butonları vurgulanır. | 4 katmanlı mimari (`config.py`, `simulation/`, `algorithms/`, `interface/` + `analysis/`). **A\* küresel planlama** (`algorithms/astar.py`): `0,2 m` ızgara çözünürlüğü, `r_R + d_safe = 0,50 m` Öklid engel şişirme, `octile_heuristic_m` ve yol düzleştirme. |
| **1:15 – 2:15** | **3. DWA ve Karşıdan Gelen Engelden Kaçınma** | `KARŞIDAN GELEN` (`HEAD-ON`) ve `DUR-KALK` (`STOP-AND-GO`) senaryolarında `MOD: DWA` açıkken `BAŞLAT` ile canlı koşum; sağdaki **Güvenlik ve DWA Telemetri Paneli** yakın planı. | **DWA yerel planlama** (`algorithms/dwa.py`): `7 × 17 = 119` hız penceresi örneklemesi, `T_p = 1,2 s` ileri benzetim, karşıdan gelen engele karşı kavis çizerek yanal kaçınma, sürekli süpürme açıklığı (`0,15 m` güvenlik payı), `compute_stopping_distance_m` aktif frenleme ve `GÜVENLİ / DİKKAT / KRİTİK` göstergeleri. |
| **2:15 – 3:15** | **4. `CORRIDOR` ve `NARROW-GATE` Gösterimleri** | Önce `KORİDOR` (`CORRIDOR`) senaryosu canlı çalıştırılır; ardından `NARROW-GATE` senaryosu Pygame penceresinde canlı çalıştırılır. | **`CorridorPassingManager` durum makinesi** (`algorithms/corridor_passing.py`): A\* yolunu merkeze erken döndürmeden `PASSING_ARMED -> PASSING_ACTIVE -> ABORT_HOLD` geçişleri ve `keep_committed_side` yan-şerit bağlılığı.<br>`NARROW-GATE`: dar geçitte güvenli bekleme (`SAFE_STANDSTILL_BRAKING`, `T_w = 2,5 s`), engel yön değişimi sonrası geçit çıkışı ve proaktif yanal geçiş. |
| **3:15 – 3:50** | **5. Deneysel Sonuçlar ve Kapanış** | Terminalde/raporda tarihsel Faz 6.5.6 ve güncel `e3b6ac3` (`202/202`) doğrulama çıktıları ile ekip kapanış kartı gösterilir. | Tarihsel Faz 6.5.6: `191/191` pytest, `8/8` temel senaryo (`min_swept = 0,152024 m`), `14/14` `NARROW-GATE` Seviye 1 (`42` koşum, `min_swept = 0,150019 m`, medyan `5,58 ms`, `P99 = 232,48 ms`). Güncel `main` (`e3b6ac3`) doğrulaması: **`202/202` pytest** (yerel + GitHub Actions) ve `DO-01` (`min_swept = 0,165947 m`, `0` çarpışma). |
