# Faz 7.1 — Türkçe Akademik Proje Raporu Doğrulama Kaydı

**Hedef Çıktılar:**
- [phase71_akademik_rapor.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase71_akademik_rapor.md)
- [phase71_akademik_rapor.pdf](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase71_akademik_rapor.pdf)
- [phase71_ekip_gorev_dagilimi.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase71_ekip_gorev_dagilimi.md)
- [phase71_rapor_dogrulama.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase71_rapor_dogrulama.md)

---

## 1. Teknik ve Kaynak Kod Doğrulaması

| Kontrol Kalemi | Doğrulanan Kaynak | Durum |
|---|---|---|
| Dünya ve Zaman Parametreleri (`20,0 m x 15,0 m`, `h = 0,2 m`, `dt_s = 0,05 s`, `max_sim_time_s = 60,0 s`) | `config.py` (`WorldConfig`, `ExperimentConfig`) | DOĞRULANDI |
| Robot Kinematik Parametreleri (`radius_m = 0,35 m`, `v ∈ [-0,6, 3,0] m/s`, `ω ∈ [-2,8, 2,8] rad/s`, orta nokta integrasyonu) | `config.py` (`RobotConfig`, `DWAConfig`), `simulation/entities.py` (`Robot.step`) | DOĞRULANDI |
| A* Şişirme, Maliyetler ve Oktil Sezgisel (`1,0`, `√2`, `octile_heuristic_m`, `is_segment_collision_free`) | `algorithms/astar.py` | DOĞRULANDI |
| DWA Örnekleme, Puanlama ve Geri Çekilme (`7 x 17 = 119`, `T_p = 1,2 s`, `Δt_p = 0,1 s`, `w_h=1,3`, `w_c=0,8`, `w_v=1,8`, `w_p=1,0`, `d_stop`, `waiting_horizon_s = 2,5 s`, `SAFE_STANDSTILL_BRAKING`, `CorridorPassingManager`) | `config.py` (`DWAConfig`), `algorithms/dwa.py` | DOĞRULANDI |
| Bağımsız Sürekli Süpürme Açıklığı Denetimi (`compute_swept_clearance_m`) | `analysis/metrics.py` | DOĞRULANDI |

---

## 2. Deneysel Sonuç Doğrulaması (Faz 6.5.6)

- **Test Süiti:** `191 / 191` pytest testi geçti (`0` hata, `0` atlanan, `175,52 s`).
- **Temel Senaryolar (`8 / 8`):** Tamamı `GOAL_REACHED`, `0` çarpışma, `0` güvenlik payı ihlali (`≥ 0,1500 m`), minimum süpürme açıklığı `0,152024 m` (`STOP-AND-GO`), ortalama DWA süresi ortalaması `11,85 ms`, ortalama P95 `36,98 ms`, maksimum `246,88 ms`.
- **`NARROW-GATE` Seviye 1 (`14 / 14` varyant, `42` koşum):** Tamamı `GOAL_REACHED`, `0` çarpışma, `0` güvenlik payı ihlali, `%100` 3-koşumlu determinizm, minimum süpürme açıklığı `0,150019 m` (`NARROW-GATE__L1-DO-01__01`), `19.014` DWA çağrısında medyan `5,58 ms`, P99 `232,48 ms`, `3.280 / 19.014` (`%17,25`) `> 50 ms`.

---

## 3. Biçim, Dil, Ekip Bilgileri ve PDF Sayfa Sayısı Kontrolü

- **Dil ve Terminoloji:** Raporun ana dili tamamen akademik Türkçedir; teknik terimlerin ilk kullanımında İngilizce karşılıkları parantez içinde verilmiş, kod içi sınıf/fonksiyon/dosya/değişken adları korunmuş ve `README.md` terminolojisiyle tam uyum sağlanmıştır.
- **Ekip ve Kurum Bilgileri:** Proje ekibi **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran** olarak yazılmış; görev dağılımı *önerilen sorumluluk paylaşımı* olarak sunulmuş ve ortak çalışma vurgulanmıştır. Üniversite, danışman ve teslim tarihi alanları uydurulmadan doldurulabilir boşluk (`[...]`) olarak bırakılmıştır.
- **Kaynakça:** Yalnızca gerçek, doğrulanabilir akademik kaynaklar (Hart, Nilsson & Raphael 1968; Fox, Burgard & Thrun 1997; Siegwart, Nourbakhsh & Scaramuzza 2011) ve proje deposu belgeleri kullanılmıştır.
- **Şekiller ve Tablolar:** Numaralandırılmış `Şekil 1` (sistem mimarisi diyagramı) ve `Tablo 1`, `Tablo 2` mevcuttur.
- **PDF Sayfa Sayısı:** `phase71_akademik_rapor.pdf` (`281.906` bayt) A4 boyutunda **4 sayfa** olarak üretilmiştir (kapak bloğu ve kaynakça dahil; hedeflenen `4–6` sayfa aralığındadır).

---

## 4. Salt-Okunur Koruma Kontrolü

Bu aşamada hiçbir Python kaynak dosyası, konfigürasyon (`config.py`), test (`tests/`) veya tarihsel kıyaslama (Faz 6.5.4–6.5.6) çıktısı değiştirilmemiştir.
