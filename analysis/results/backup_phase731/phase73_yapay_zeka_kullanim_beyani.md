# DYNAMIC-NAV — Yapay Zekâ Kullanım Beyanı

**Proje Başlığı:** DYNAMIC-NAV — P2 Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  
**Üniversite / Danışman / Tarih:** [Üniversite Adı Giriniz] | [Danışman Adı Giriniz] | [GG.AA.YYYY]  

---

## 1. Beyanın Amacı ve Kapsamı

Bu belge, DYNAMIC-NAV projesinin geliştirme, test, dokümantasyon ve sunum süreçlerinde yapay zekâ destekli araçların kullanımını akademik şeffaflık ilkesi doğrultusunda beyan etmek amacıyla hazırlanmıştır. Yapay zekâ araçları projeyi bağımsız olarak geliştirmemiş; mühendislik geliştirme sürecinde yardımcı araç olarak kullanılmıştır. Tüm mimari kararlar, gereksinim doğrulamaları ve nihai teslim sorumluluğu proje ekibine aittir.

## 2. Yapay Zekâ Destekli Çalışma Alanları

- **Yazılım Geliştirme ve Hata Ayıklama Desteği:** Python/Pygame tabanlı arayüz (`interface/`), simülasyon yardımcıları ve kod düzenlemelerinde hata ayıklama, sözdizimi iyileştirme ve kod inceleme desteği alınmıştır. Çekirdek SI birim disiplini (`config.py`), kinematik model ve modül ayrımı (`simulation/` içinde `pygame` bağımsızlığı) ekip tarafından belirlenen mühendislik kurallarına göre korunmuştur.
- **Algoritma Tasarım Desteği ve Teknik Açıklamalar:** 8-yönlü A\* küresel yol planlama (`algorithms/astar.py`), Dinamik Pencere Yaklaşımı (DWA, `algorithms/dwa.py`), sürekli süpürme açıklığı (swept-clearance) ve geri çekilme (fallback) davranışlarının matematiksel formülasyonu ile teknik dokümantasyon anlatımlarında yapılandırma desteği alınmıştır.
- **Test Oluşturma ve Sonuç Yorumlama:** Regresyon ve senaryo testlerinin (`tests/`) yazımı, `191/191` `pytest` çıktısının ve Faz 6.5.6 bağımsız denetim metriklerinin (`8/8` temel senaryo, `14/14` `NARROW-GATE` Seviye 1 / `42` koşum) yorumlanması süreçlerinde destek sağlanmıştır.
- **Dokümantasyon ve Raporlama:** Türkçe `README.md`, Faz 7.1 akademik proje raporu ve Faz 7.3 idari teslim belgelerinin dil tutarlılığı ve biçimlendirilmesinde yapay zekâ desteği kullanılmıştır.
- **Video ve Sunum Hazırlığı:** Demo videosu (`phase725_dynamic_nav_final_demo.mp4`) gerçek Pygame simülasyon ekranlarından yakalanmış; Türkçe seslendirme için `edge-tts` (`tr-TR-AhmetNeural`) yapay zekâ tabanlı konuşma sentezi, senkronize altyazı gömme ve teknik doğrulama için `FFmpeg`/`FFprobe` kullanılmıştır.

## 3. İnsan Doğrulaması ve Nihai Sorumluluk

A\*, DWA, simülasyon fiziği, çarpışma ve sürekli süpürme açıklığı güvenlik mekanizmaları (`0,15 m` güvenlik payı) proje gereksinimlerine karşı doğrudan çalıştırılarak (`191/191` `pytest`, bağımsız `analysis/metrics.py` denetimi ve canlı Pygame pencere testleri) doğrulanmıştır. Deneysel veriler uydurulmamış; gerçek koşum ve test çıktıları esas alınmıştır. Projeye ilişkin tüm nihai teknik kararlar, doğrulama onayları ve akademik teslim sorumluluğu **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran**'a aittir.

**Ekip Onayı / İmza:** Eren Çakmaz `[........]` | Ahmet Gürlek `[........]` | Arda Başaran `[........]`
