# Faz 7.3.2 — Deneysel Kanıt Envanteri (`phase732_experimental_evidence_inventory.md`)

**Proje:** DYNAMIC-NAV — Sakarya Uygulamalı Bilimler Üniversitesi, Elektrik-Elektronik Mühendisliği  
**Tam İzlenebilirlik Tablosu:** [phase732_experimental_data_traceability.csv](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase732_experimental_data_traceability.csv) (`136` dosya)

---

## 1. İncelenen Deney Grupları ve Doğrulanan Bulgular

### 1.1. Faz 5.0 vs. Faz 6.2 Algoritma Geliştirme Karşılaştırması
- **Kaynaklar:** `analysis/results/benchmark_raw.json` (Faz 5.0, `10` koşum), `analysis/results/benchmark_phase62_raw.json` (Faz 6.2, `12` koşum).
- **Faz 5.0 (5 senaryo × 2 kontrolcü):**
  - `ASTAR_FOLLOWER`: `0/5` `GOAL_REACHED`, `5/5` `COLLISION_ABORT` (`col=1` her senaryoda).
  - `ASTAR_DWA`: `3/5` `GOAL_REACHED` (`HEAD-ON`, `CROSSING` 1 pay ihlaliyle, `MULTIPLE-DYNAMIC-OBSTACLES`); `2/5` `COLLISION_ABORT` (`NARROW-GATE` `swept = -0,0232 m`, `SUDDEN-STOP` `swept = -0,0299 m`).
- **Faz 6.2 (6 senaryo × 2 kontrolcü):**
  - `ASTAR_FOLLOWER`: `0/6` `GOAL_REACHED`, `6/6` `COLLISION_ABORT`.
  - `ASTAR_DWA`: `6/6` `GOAL_REACHED` (`0` fiziksel çarpışma); ancak `SUDDEN-STOP` (`swept = 0,1390 m < 0,1500 m`, `2` pay ihlali), `CROSSING` (`1` pay ihlali) ve `SUDDEN-STOP-FINITE-DECEL` (`1` pay ihlali) koşullarında güvenlik payı ihlalleri devam etmekteydi.

### 1.2. Güvenlik Metrikleri Ayrımı
- `min_instantaneous_clearance_m` (tek zaman adımı yüzey açıklığı) ile `min_swept_clearance_m` (ardışık zaman adımları arasındaki sürekli hareket süpürmesi açıklığı) farklı metriklerdir; güvenlik sınırı `0,150000 m`'dir. Önceki fazlarda `0,15 m` altına düşen değerler (ör. Faz 6.2 `SUDDEN-STOP` `0,1390 m`, Faz 6.5.5.6 `NARROW-GATE__L1-DO-01__01` `0,1498 m`) gizlenmeden kendi geliştirme fazlarına ait bulgular olarak korunmuştur.

### 1.3. Faz 6.2.1 / 6.3 Performans Profili (Önceki Denetim)
- **Kaynak:** `analysis/results/profile_phase63_baseline.json` (`8` senaryo, `2.821` DWA çağrısı).
- **Ölçümler:** Ortalama `19,15 ms`, medyan `16,04 ms`, P95 `43,52 ms`, P99 `66,49 ms`, maks. `131,87 ms`, `50 ms` üzeri `90` çağrı (`%3,2`). Zaman bütçesi aşımı özellikle `SUDDEN-STOP-FINITE-DECEL` (`101` çağrı, `%26,65`, maks. `154,10 ms`), `CROSSING` (`39` çağrı, `%14,13`, maks. `103,23 ms`) ve `MULTIPLE-DYNAMIC-OBSTACLES` (`36` çağrı, `%10,26`, maks. `99,37 ms`) senaryolarında gözlenmiştir. *(Not: Bu ölçümler Faz 6.2.1/6.3 profiline aittir; Faz 6.5.6 final ölçümleriyle karıştırılmamıştır.)*

### 1.4. Faz 6.5.5 Seviye 1 Dayanıklılık Taraması
- **Kaynak:** `analysis/results/phase655_level1_summary.json` (`120` varyant × `3` tekrar = `360` koşum).
- **Bulgular:** Başlangıç başarı oranı `%85,0` (`102/120` varyant), fiziksel çarpışma oranı `%15,0` (`18/120` varyant), güvenlik payı ihlal oranı `%20,83` (`25/120` varyant), fiziksel determinizm `%100,0` (`120/120`). Bu tarama Faz 6.5.5.1–6.5.5.7 düzeltmelerinin (zamanlayıcı, güvenli bekleme, açık alan sollaması ve sürekli statik AABB süpürme hizalaması) gerekçesini oluşturmuştur.

### 1.5. Faz 6.5.6 Nihai Sistem Bütünlüğü Doğrulaması
- **Kaynaklar:** `phase656_final_audit_report.md`, `phase656_scenario_regression.json`, `phase656_performance_summary.json`, `phase656_final_test_results.json`.
- **Bulgular:** `191/191` `pytest` (`175,52 s`), `8/8` temel senaryo `GOAL_REACHED` (`0` çarpışma, `0` pay ihlali, `min_swept = 0,152024 m`), `14/14` `NARROW-GATE` Seviye 1 varyantı (`42` tekrarlı koşum, `0` çarpışma, `0` pay ihlali, `%100` determinizm, `min_swept = 0,150019 m`, `19.014` DWA çağrısında medyan `5,58 ms`).
- **Performans Sınırlılığı:** `3.280 / 19.014` çağrı (`%17,25`) `50 ms` sınırını aşmış ve P99 gecikmesi `232,48 ms` ölçülmüştür; sert gerçek-zaman garantisi iddia edilmemektedir.
