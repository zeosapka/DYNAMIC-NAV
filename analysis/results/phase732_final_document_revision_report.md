# Faz 7.3.2 / 7.3.3 — Nihai Akademik Doküman Revizyonu ve Teslim Raporu

**Kurum:** Sakarya Uygulamalı Bilimler Üniversitesi — Elektrik-Elektronik Mühendisliği  
**Proje:** DYNAMIC-NAV — Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Ekip:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  

---

## 1. Yapılan Değişiklikler ve Korunan Veriler

- **Yedekleme:** Önceki sürümler `analysis/results/backup_phase732/` altına yedeklendi.
- **Aşama A (Deneysel Kanıt Envanteri):** `analysis/results/` içindeki `136` dosya tarandı; [phase732_experimental_evidence_inventory.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase732_experimental_evidence_inventory.md) ve [phase732_experimental_data_traceability.csv](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase732_experimental_data_traceability.csv) oluşturuldu. Faz 5.0 (`benchmark_raw.json`), Faz 6.2 (`benchmark_phase62_raw.json`), Faz 6.2.1/6.3 (`profile_phase63_baseline.json`), Faz 6.5.5 (`phase655_level1_summary.json`) ve Faz 6.5.6 (`phase656_*`) verileri kapsamları karıştırılmadan ayrıştırıldı; eski pay ihlalleri (`0,1390 m` vb.) gizlenmeden tarihsel gelişim kanıtı olarak korundu.
- **Aşama B (Bilimsel Yayın Tipografisi ve Revizyon):** Tüm sunum tarzı koyu mavi şeritler kaldırıldı; beyaz zemin, siyah/koyu gri metin, **Times New Roman** (`11,5 pt` gövde), **Cambria Math** (MathML denklemler `(1)`–`(9)`) ve **Consolas** fontları gömüldü.
- **Koruma:** Kod tabanı, algoritmalar, testler ve kabul edilmiş demo videosu (`phase725_dynamic_nav_final_demo.mp4`) değiştirilmedi.

---

## 2. Üretilen PDF'ler, Sayfa Sayıları ve Font Doğrulaması

| Dosya | Sayfa | A4 (`595×842 pt`) | Gömülü Fontlar |
|---|---|---|---|
| [phase71_akademik_rapor.pdf](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase71_akademik_rapor.pdf) | **4** (`4–6` hedefi) | Evet | `TimesNewRomanPSMT`, `TimesNewRomanPS-BoldMT`, `TimesNewRomanPS-ItalicMT`, `CambriaMath`, `Consolas` |
| [phase73_ekip_katki_tablosu.pdf](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase73_ekip_katki_tablosu.pdf) | **1** | Evet | `TimesNewRomanPSMT`, `TimesNewRomanPS-BoldMT`, `Consolas` |
| [phase73_yapay_zeka_kullanim_beyani.pdf](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase73_yapay_zeka_kullanim_beyani.pdf) | **1** (`<=1` hedefi) | Evet | `TimesNewRomanPSMT`, `TimesNewRomanPS-BoldMT`, `Consolas` |

---

## 3. Denklem, Teslim ve Sınırlılık Özeti

- **Denklem Doğrulaması ([phase732_equation_verification.json](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase732_equation_verification.json)):** Denklemler (1)–(9) `simulation/entities.py`, `algorithms/astar.py`, `algorithms/dwa.py`, `analysis/metrics.py` ve `config.py` ile birebir doğrulandı; tamamı **Cambria Math** ile dizildi.
- **Teslim Kontrol Listesi ([phase732_submission_compliance_matrix.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase732_submission_compliance_matrix.md)):** 10/10 madde **PASS**.
- **Bilinen Sınırlılıklar:** (i) Faz 6.5.6 `NARROW-GATE` testlerinde `3.280 / 19.014` (`%17,25`) çağrının `50 ms` adım bütçesini aşması (`P99 = 232,48 ms`) nedeniyle sert gerçek-zaman garantisi iddia edilmemektedir; (ii) 2B dairesel gövde ve kısa ufuklu sabit hızlı engel kestirimi kullanılmaktadır; (iii) kişi bazlı git geçmişi olmadığından görev dağılımı modüler öneri olarak sunulmuştur.
- **Eksik Kalan İş:** Teslim aşamasında yalnızca paket dışı geçici klasörlerin (`.venv`, `__pycache__`, `.pytest_cache`, `demo_assets/`) dışarıda bırakılarak arşivin oluşturulması kalmıştır.
