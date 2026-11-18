# DYNAMIC-NAV — Yapay Zekâ Kullanım Beyanı

**Proje Başlığı:** DYNAMIC-NAV — P2 Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Üniversite:** Sakarya Uygulamalı Bilimler Üniversitesi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  

---

## 1. Beyanın Amacı ve Kapsamı

Bu belge, DYNAMIC-NAV projesinin yazılım geliştirme, doğrulama, dokümantasyon ve sunum aşamalarında yapay zekâ destekli araçların kullanımını akademik şeffaflık ilkesi çerçevesinde açıklamak üzere hazırlanmıştır. Yapay zekâ araçları projeyi bağımsız olarak geliştirmemiş; mühendislik sürecinde yardımcı araç olarak kullanılmıştır. Tüm mimari kararlar, algoritma doğrulamaları ve teslim sorumluluğu proje ekibine aittir.

## 2. Yapay Zekâ Destekli Çalışma Alanları

- **Yazılım Geliştirme ve Hata Ayıklama Desteği:** Python/Pygame arayüzü (`interface/`), görselleştirme bileşenleri ve yardımcı betiklerde kod düzenleme ve hata ayıklama desteği alınmıştır. Çekirdek SI birim disiplini (`config.py`), kinematik model ve modül ayrımı (`simulation/` içinde `pygame` bağımsızlığı) proje kurallarına göre korunmuştur.
- **Algoritma Tasarım Desteği ve Teknik Açıklamalar:** 8-yönlü A\* küresel yol planlama (`algorithms/astar.py`), Dinamik Pencere Yaklaşımı (DWA, `algorithms/dwa.py`), sürekli süpürme açıklığı ve geri çekilme mekanizmalarının teknik açıklamaları ile matematiksel gösterimlerinin düzenlenmesinde destek alınmıştır.
- **Test Oluşturma ve Sonuç Yorumlama:** Regresyon testlerinin (`tests/`) yapılandırılması, `191/191` `pytest` çıktısının ve Faz 6.5.6 bağımsız denetim metriklerinin (`8/8` temel senaryo, `14/14` `NARROW-GATE` Seviye 1 / `42` koşum) analizinde destek sağlanmıştır.
- **Dokümantasyon ve Raporlama:** Türkçe `README.md`, akademik proje raporu ve idari teslim belgelerinin akademik Türkçe dil tutarlılığı ve biçimlendirmesinde yapay zekâ desteği kullanılmıştır.
- **Video ve Sunum Hazırlığı:** Demo videosu (`phase725_dynamic_nav_final_demo.mp4`) gerçek Pygame simülasyon çıktılarından kaydedilmiş; Türkçe seslendirme için `edge-tts` (`tr-TR-AhmetNeural`) konuşma sentezi, senkronize altyazı gömme ve teknik doğrulama için `FFmpeg`/`FFprobe` kullanılmıştır.

## 3. İnsan Doğrulaması ve Nihai Sorumluluk

A\*, DWA, simülasyon fiziği, çarpışma ve sürekli süpürme açıklığı güvenlik mekanizmaları (`0,15 m` güvenlik payı) proje gereksinimlerine karşı doğrudan çalıştırılarak (`191/191` `pytest`, bağımsız `analysis/metrics.py` denetimi ve canlı Pygame pencere testleri) doğrulanmıştır. Deneysel veriler gerçek koşum ve test çıktılarından elde edilmiştir. Projeye ilişkin tüm nihai teknik kararlar, doğrulama sonuçları ve akademik teslim sorumluluğu **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran**'a aittir.
