# DYNAMIC-NAV — Ekip Katkı ve Görev Dağılımı Tablosu

**Proje Başlığı:** DYNAMIC-NAV — P2 Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Üniversite:** Sakarya Uygulamalı Bilimler Üniversitesi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  

---

## 1. Kapsam ve Metodoloji

Bu belge, DYNAMIC-NAV projesinin geliştirme, test, deneysel değerlendirme ve dokümantasyon süreçlerindeki görev dağılımını özetlemektedir. Çalışma **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran** tarafından ortak mühendislik projesi olarak yürütülmüş; her üç ekip üyesi de yazılım geliştirme, doğrulama testleri ve akademik raporlama aşamalarına ortak katkı sağlamıştır.

---

## 2. Proje Görev Dağılımı Tablosu

| Ekip Üyesi | Odak Alanı | İlgili Modüller / Çıktılar | Doğrulanan Proje Çıktıları |
|---|---|---|---|
| **Eren Çakmaz** | Simülasyon Çekirdeği, Kinematik Model ve Küresel Yol Planlama | `config.py`, `simulation/entities.py`, `simulation/world.py`, `algorithms/astar.py` | Tam SI birim disiplini, orta nokta yönelim integrasyonu, 8-yönlü A\* planlayıcısı ve Öklid engel şişirme modülleri |
| **Ahmet Gürlek** | Yerel Planlama (DWA), Güvenlik Doğrulaması, Senaryolar ve Arayüz | `algorithms/dwa.py`, `simulation/scenarios.py`, `interface/` | DWA hız penceresi, aktif frenleme doğrulaması, geri çekilme hiyerarşisi, 8 dinamik engel senaryosu ve Pygame arayüzü |
| **Arda Başaran** | Deneysel Kıyaslama, Test Süiti ve Akademik Dokümantasyon | `analysis/`, `tests/`, `README.md`, Faz 6.5.6–7.3 belgeleri | `191/191` `pytest` regresyon doğrulaması, `8/8` temel ve `42` `NARROW-GATE` koşum analizleri, Türkçe dokümantasyon ve raporlama |
