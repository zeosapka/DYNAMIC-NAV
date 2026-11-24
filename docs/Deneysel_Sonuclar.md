# DYNAMIC-NAV — Deneysel Sonuçlar Özeti

**Doğrulanan Ana Dal Commit'i:** `e3b6ac3` (kod) / `11afc39` (dokümantasyon)  
**Test Altyapısı:** `Python 3.12.10` / `pytest 9.2.0` (yerel) ve `ubuntu-latest` GitHub Actions (`Python 3.12`, [`.github/workflows/tests.yml`](../.github/workflows/tests.yml))

---

## 1. Güncel Tam Test Paketi ve CI Doğrulaması (`202/202`)

- **Yerel Tam Paket (`python -m pytest -v`)**: **`202 passed`**, `0` failed, `0` skipped (`467,93 s`).
- **GitHub Actions CI (`Pytest (Python 3.12)`)**:
  - Commit `e3b6ac3` (Run `45395241408`): **`202 passed`**, `0` failed, `0` skipped (`176,40 s`).
  - Commit `11afc39` (Run `45397374119`): **`202 passed`**, `0` failed, `0` skipped (`218,45 s`).

---

## 2. 8 Deterministik Temel Senaryo Sonuçları (Faz 6.5.6 Kayıtları — Hedefe Ulaşma Oranı: `8 / 8`, `%100`)

Ham kanıt: [`analysis/results/phase656_scenario_regression.json`](../analysis/results/phase656_scenario_regression.json) ve [`analysis/results/phase656_final_audit_report.md`](../analysis/results/phase656_final_audit_report.md).

| Senaryo | Hedefe Ulaşma | Çarpışma | Min. Süpürme Açıklığı (m) | Görev Süresi (s) | Durma Sayısı | Güvenlik İhlali | Yeniden Planlama (Başarılı/Deneme) |
|---|---|---|---|---|---|---|---|
| `HEAD-ON` | `GOAL_REACHED` (`True`) | 0 | `0.257926` | `15.85` | 1 | 0 | `0 / 0` |
| `CROSSING` | `GOAL_REACHED` (`True`) | 0 | `0.152699` | `13.80` | 0 | 0 | `0 / 0` |
| `CORRIDOR` | `GOAL_REACHED` (`True`) | 0 | `0.155123` | `19.45` | 1 | 0 | `3 / 3` |
| `STOP-AND-GO` | `GOAL_REACHED` (`True`) | 0 | `0.152024` | `11.00` | 0 | 0 | `0 / 0` |
| `NARROW-GATE` | `GOAL_REACHED` (`True`) | 0 | `0.154363` | `22.50` | 2 | 0 | `1 / 1` |
| `MULTIPLE-DYNAMIC-OBSTACLES` | `GOAL_REACHED` (`True`) | 0 | `0.152792` | `17.60` | 1 | 0 | `0 / 0` |
| `SUDDEN-STOP` | `GOAL_REACHED` (`True`) | 0 | `0.298483` | `12.95` | 0 | 0 | `1 / 1` |
| `SUDDEN-STOP-FINITE-DECEL` | `GOAL_REACHED` (`True`) | 0 | `0.153625` | `16.70` | 1 | 0 | `0 / 0` |

## 2b. Görülmemiş Özel JSON Senaryoları Doğrulaması (`2 / 2`, `%100`)

`examples/custom_scenario_example.json` ve `tests/test_generalizability_and_status.py` ile tanımlanan iki yeni, daha önce görülmemiş başlangıç/hedef ve dinamik engel düzeni:

| Özel Senaryo | Hedefe Ulaşma | Çarpışma | Min. Süpürme Açıklığı (m) | Görev Süresi (s) |
|---|---|---|---|---|
| `CUSTOM-UNSEEN-DIAGONAL` (`examples/custom_scenario_example.json`) | `True` | 0 | `0.156585` | `14.05` |
| `CUSTOM-UNSEEN-REVERSE-LANE` | `True` | 0 | `0.150010` | `12.85` |

---

## 3. `CORRIDOR__L1-DO-01__02` Nihai Doğrulaması (`obs_speed_scale = 1.15`)

Tarihsel `cf348a3` tabanında `-0.036039 m` (ve ara sertleştirmede `-0.003664 m` / `-0.040569 m`) ile başarısız olan `CORRIDOR__L1-DO-01__02` varyantı, `CorridorPassingManager` durum makinesi ve `keep_committed_side` yan-şerit bağlılığı sonrasında sıfır hatayla çözülmüştür:

- **`goal_reached`**: `True`
- **Fiziksel çarpışma (`physical_collision_episode_count`)**: `0`
- **Güvenlik marjı ihlali (`safety_margin_violation_episode_count`)**: `0`
- **`min_swept_m`**: `0.165947 m` (`>= 0.150000 m`)
- **`min_clearance_m`**: `0.165947 m`
- **`stop_count`**: `1`
- **`proactive_replans`**: `4` (`replan_attempts = 0`, `replan_successes = 0`)
- **`completion_time_s`**: `18.5 s`
- **Determinizm**: Doğrulandı (`True`, iki bağımsız koşum özdeş)

---

## 4. Tarihsel Sonuçlar vs. Nihai Sonuçlar ve Performans Sınırlılıkları

- **Tarihsel Kayıtlar**: Faz 5.0 (`benchmark_raw.json`), Faz 6.2 (`benchmark_phase62_raw.json`), Faz 6.5.5 (`phase655_level1_raw.json`) ve Faz 8.0 ön-düzeltme (`phase80_pre_fix_360_baseline.json`) dosyaları `analysis/results/` altında değiştirilmeden korunmaktadır ve güncel sonuçlarla karıştırılmamalıdır.
- **Hesaplama Süresi Sınırlılığı**: `NARROW-GATE` Seviye 1 testlerinde (`19.014` DWA çağrısı) medyan DWA süresi `5.58 ms` olmakla birlikte `P99 = 232.48 ms` ölçülmüştür (`%17.25 > 50 ms`). Bu nedenle sert gerçek-zaman duvar saati garantisi iddia edilmemektedir.
