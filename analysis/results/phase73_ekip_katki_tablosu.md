# DYNAMIC-NAV — Ekip Katkı ve Görev Dağılımı Tablosu

**Kurum:** Sakarya Uygulamalı Bilimler Üniversitesi — Elektrik-Elektronik Mühendisliği  
**Proje:** DYNAMIC-NAV — Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  

---

## 1. Kapsam ve Ortak Çalışma Çerçevesi

DYNAMIC-NAV projesi **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran** tarafından ortak bir mühendislik çalışması olarak geliştirilmiştir. Tüm ekip üyeleri gereksinim analizi, testlerin yürütülmesi, hata ayıklama ve deneysel değerlendirme süreçlerine ortak katkı sağlamıştır. Aşağıdaki tablo, bireysel üstünlük veya tekil sahiplik iddiası taşımaksızın, proje modüllerine göre oluşturulmuş **önerilen görev dağılımını** göstermektedir.

---

## 2. Önerilen Proje Görev Dağılımı

| Ekip Üyesi | Odak Alanı | İlgili Modüller ve Çıktılar |
|---|---|---|
| **Eren Çakmaz** | Algoritma ve Planlama | `algorithms/astar.py`, `algorithms/dwa.py`, küresel ve yerel yol planlama, engel şişirme ve yörünge puanlama |
| **Ahmet Gürlek** | Simülasyon ve Test | `config.py`, `simulation/`, `tests/`, kinematik model, dinamik engel senaryoları ve `191` `pytest` regresyon doğrulaması |
| **Arda Başaran** | Arayüz, Entegrasyon ve Dokümantasyon | `interface/`, `analysis/`, `README.md`, Türkçe arayüz entegrasyonu, demo videosu hazırlığı ve akademik raporlama |
