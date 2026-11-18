# Faz 7.2 — DYNAMIC-NAV Türkçe Demo Video Senaryosu

**Proje:** DYNAMIC-NAV: Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Ekip:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  
**Hedef Süre:** `04:45` (3–5 dakika kuralına tam uyumlu)  
**Dil ve Altyazı:** Tamamen Türkçe seslendirme + senkron Türkçe altyazı  

---

## Zaman Çizelgesi ve Sahne Planı

| Zaman Aralığı | Bölüm | Ekrandaki Görüntü (Gerçek Uygulama / Veri) | Gösterilecek Teknik İçerik |
|---|---|---|---|
| **0:00 – 0:30** | **1. Proje Tanıtımı** | `main.py` başlatılır; **ANA MENÜ** (`AppView.HOME`) ekranı görüntülenir. | Proje başlığı, Elektrik-Elektronik Mühendisliği ekip bilgisi, dinamik ortamlarda otonom mobil robot navigasyonu problemi ve SI birimli deterministik simülatör amacı. |
| **0:30 – 1:15** | **2. Sistem Mimarisi ve A\*** | Sağ karttaki `ALGORİTMA BİLGİSİ` (`AppView.GUIDE`) sekmesi kısaca gösterilir; ardından **SİMÜLASYON** ekranına geçilip `A* PLANLA` ve `8-YÖN` butonları vurgulanır. | 4 katmanlı mimari (`config.py`, `simulation/`, `algorithms/`, `interface/` + `analysis/`), `0,2 m` ızgara çözünürlüğü, `r_R + d_safe = 0,50 m` Öklid engel şişirme, `octile_heuristic_m` ve yol düzleştirme. |
| **1:15 – 2:15** | **3. DWA ve Dinamik Engel Kaçınma** | `KARŞIDAN GELEN` (`HEAD-ON`) ve `DUR-KALK` (`STOP-AND-GO`) senaryolarında `MOD: DWA` açıkken `BAŞLAT` ile canlı koşum; sağdaki **Güvenlik ve DWA Telemetri Paneli** yakın planı. | `7 × 17 = 119` hız penceresi örneklemesi, `T_p = 1,2 s` ileri benzetim, sürekli süpürme açıklığı (`0,15 m` güvenlik payı), `compute_stopping_distance_m` aktif frenleme, `GÜVENLİ / DİKKAT / KRİTİK` durum göstergeleri. |
| **2:15 – 3:15** | **4. Senaryo Gösterimleri (`CORRIDOR` ve `NARROW-GATE`)** | Önce `KORİDOR` (`CORRIDOR`) senaryosu canlı çalıştırılır; ardından read-only başlatıcı ile `NARROW-GATE` senaryosu Pygame penceresinde canlı çalıştırılır. | `CORRIDOR`: dar duvarlar arasında statik pay + engel kaçınması.<br>`NARROW-GATE`: dar geçitte güvenli bekleme (`SAFE_STANDSTILL_BRAKING`, `T_w = 2,5 s`), yön değiştirme, geçit çıkışı sonrası `CorridorPassingManager` ile proaktif yanal sollama ve alternatif manevra seçimi. |
| **3:15 – 4:15** | **5. Deneysel Sonuçlar (Faz 6.5.6)** | Terminalde `phase656_final_audit_report.md` özet tablosu veya sunum slaydı üzerinde gerçek Faz 6.5.6 çıktıları gösterilir. | `191/191` pytest testi (`175,52 s`), `8/8` temel senaryo `GOAL_REACHED` (`min_swept = 0,152024 m`, ort. DWA `11,85 ms`), `14/14` `NARROW-GATE` Seviye 1 varyantı (`42` koşum, `0` çarpışma, `0` pay ihlali, `min_swept = 0,150019 m`, `%100` determinizm, medyan `5,58 ms`). |
| **4:15 – 4:45** | **6. Sonuç ve Ekip** | Uygulamanın `ANA MENÜ` ekranı ile kapanış kartı (proje adı, bölüm, 3 ekip üyesi ve önerilen sorumluluk paylaşımı). | Mühendislik katkılarının özeti, gerçek zamanlı P99 gecikme (`232,48 ms`) için gelecek iyileştirme notu ve ekip kapanışı. |
