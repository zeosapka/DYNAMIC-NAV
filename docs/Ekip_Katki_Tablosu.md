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

| Ekip Üyesi | Odak Alanı ve Çıktılar | Karşılaşılan Teknik Sorun ve Çözüm | Onay Durumu |
|---|---|---|---|
| **Eren Çakmaz** | Algoritma ve Planlama: `algorithms/astar.py`, `algorithms/dwa.py`, `algorithms/corridor_passing.py`; küresel/yerel planlama ve `ABORT_HOLD` güvenliği | Izgara yolu engel köşelerinde gerekli açıklığı tek başına garanti etmiyordu; engel şişirme ve sürekli segment açıklığı kontrolleri eklendi. Dinamik engellerin koridor geçişini bozduğu durumlar için geçiş durum makinesi ve şerit bağlılığı geliştirildi. | Onaylandı |
| **Ahmet Gürlek** | Simülasyon ve Test: `config.py`, `simulation/`, `tests/`; kinematik model ve dinamik engel senaryoları | Zaman adımı sınırlarında dinamik engel dur/kalk geçişlerinde zamanlama ve tekrar üretilebilirlik sorunları ele alındı; `dt_s` tabanlı geçiş hesabı ve regresyon testleri kullanıldı. Senaryo ve hız/geometri sınırları için girdi doğrulaması eklendi. | Onaylandı |
| **Arda Başaran** | Arayüz, Entegrasyon ve Dokümantasyon: `interface/`, `analysis/`, `README.md`; demo (`229.73 s`) ve teknik raporlama | Anlık açıklık tek başına adımlar arasındaki olası çarpışmayı göstermiyordu; deney analizine süpürülmüş açıklık ve çarpışma/güvenlik payı ayrımı eklendi. Test ve deney sonuçları raporlanabilir metriklerle dışa aktarıldı. | Onaylandı |

Sorun ve çözüm örnekleri ekip görev alanları ile proje test/analiz kayıtlarından derlenmiştir; tabloda kişisel hata sahipliği iddia edilmemektedir. Ekip üyeleri teslim öncesinde kendi katkı anlatımlarını doğrulamalıdır.
