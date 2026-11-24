# DYNAMIC-NAV — Ekip Katkı ve Görev Dağılımı Tablosu

**Kurum:** Sakarya Uygulamalı Bilimler Üniversitesi — Elektrik-Elektronik Mühendisliği  
**Proje:** DYNAMIC-NAV — Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  
**Danışman:** `[DANIŞMAN UNVANI VE ADI — İNSAN ONAYI BEKLENİYOR]`  
**Teslim Tarihi:** `[TESLİM TARİHİ — İNSAN ONAYI BEKLENİYOR]`

---

## 1. Kapsam ve Ortak Çalışma Çerçevesi

DYNAMIC-NAV projesi **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran** tarafından ortak bir mühendislik çalışması olarak geliştirilmiştir. Tüm ekip üyeleri gereksinim analizi, testlerin yürütülmesi, hata ayıklama ve deneysel değerlendirme süreçlerine ortak katkı sağlamıştır. Depoda bireysel sürüm kontrolü (`git`) kişi bazlı ayrışmadığı için aşağıdaki tablo, proje modüllerine göre **önerilen görev dağılımını** göstermektedir ve her üyenin yazılı onayını beklemektedir.

---

## 2. Önerilen Proje Görev Dağılımı

| Ekip Üyesi | Odak Alanı | İlgili Modüller ve Çıktılar | Onay Durumu |
|---|---|---|---|
| **Eren Çakmaz** | Algoritma ve Planlama | `algorithms/astar.py`, `algorithms/dwa.py`, `algorithms/corridor_passing.py`, küresel/yerel yol planlama ve `ABORT_HOLD` güvenliği | `[EKİP ONAYI GEREKLİ]` |
| **Ahmet Gürlek** | Simülasyon ve Test | `config.py`, `simulation/`, `tests/`, kinematik model, dinamik engel senaryoları ve `202/202` `pytest` doğrulaması | `[EKİP ONAYI GEREKLİ]` |
| **Arda Başaran** | Arayüz, Entegrasyon ve Dokümantasyon | `interface/`, `analysis/`, `README.md`, Türkçe arayüz entegrasyonu, demo videosu (`229.73 s`) ve akademik raporlama | `[EKİP ONAYI GEREKLİ]` |
