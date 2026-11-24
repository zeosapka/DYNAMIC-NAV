# DYNAMIC-NAV — Ekip Katkı ve Görev Dağılımı Tablosu

**Kurum:** Sakarya Uygulamalı Bilimler Üniversitesi — Elektrik-Elektronik Mühendisliği  
**Proje:** DYNAMIC-NAV — Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  
**Danışman:** Doç. Dr. Ali Furkan Kamanlı  
**Teslim Tarihi:** 24 Kasım 2026

---

## 1. Kapsam ve Ortak Çalışma Çerçevesi

DYNAMIC-NAV projesi **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran** tarafından ortak bir mühendislik çalışması olarak geliştirilmiştir. Tüm ekip üyeleri gereksinim analizi, testlerin yürütülmesi, hata ayıklama ve deneysel değerlendirme süreçlerine ortak katkı sağlamıştır. Aşağıdaki tablo, proje modüllerine göre görev dağılımını ve ekip onay durumunu göstermektedir.

---

## 2. Proje Görev Dağılımı

| Ekip Üyesi | Odak Alanı | İlgili Modüller ve Çıktılar | Onay Durumu |
|---|---|---|---|
| **Eren Çakmaz** | Algoritma ve Planlama | `algorithms/astar.py`, `algorithms/dwa.py`, `algorithms/corridor_passing.py`, küresel/yerel yol planlama ve `ABORT_HOLD` güvenliği | Onaylandı |
| **Ahmet Gürlek** | Simülasyon ve Test | `config.py`, `simulation/`, `tests/`, kinematik model, dinamik engel senaryoları ve `206/206` `pytest` doğrulaması | Onaylandı |
| **Arda Başaran** | Arayüz, Entegrasyon ve Dokümantasyon | `interface/`, `analysis/`, `README.md`, Türkçe arayüz entegrasyonu, demo videosu (`229.73 s`) ve akademik raporlama | Onaylandı |
