# DYNAMIC-NAV — Yapay Zekâ Kullanım Beyanı

**Kurum:** Sakarya Uygulamalı Bilimler Üniversitesi — Elektrik-Elektronik Mühendisliği  
**Proje:** DYNAMIC-NAV — Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  

---

## 1. Beyanın Amacı

Bu belge, DYNAMIC-NAV projesinin geliştirme, dokümantasyon, test değerlendirme ve sunum hazırlama süreçlerinde yapay zekâ destekli araçların kullanımını akademik şeffaflık ilkesiyle beyan etmektedir. Yapay zekâ araçları projeyi bağımsız olarak geliştirmemiş; yalnızca mühendislik sürecinde yardımcı araç olarak kullanılmıştır.

## 2. Yapay Zekâ Destekli Çalışma Alanları

- **Kod Geliştirme ve Hata Ayıklama Desteği:** Python/Pygame arayüzü (`interface/`), görselleştirme bileşenleri ve yardımcı betiklerde kod inceleme ve hata ayıklama desteği alınmıştır.
- **Algoritma Dokümantasyonu:** A\* küresel yol planlama (`algorithms/astar.py`), DWA yerel kaçınma (`algorithms/dwa.py`) ve sürekli süpürme açıklığı mekanizmalarının teknik açıklamalarının düzenlenmesinde destek alınmıştır.
- **Deney Sonuçlarının Yorumlanmasına Destek:** `191/191` `pytest` çıktıları ile Faz 5.0–6.5.6 arası kıyaslama ve dayanıklılık kayıtlarının tablo/grafik özetlerinin hazırlanmasında destek sağlanmıştır.
- **Akademik Rapor Biçimlendirme:** `README.md`, akademik proje raporu ve idari belgelerin akademik Türkçe dil tutarlılığı ve sayfa düzeninde destek alınmıştır.
- **Demo Videosu Araç Desteği:** Gerçek Pygame uygulamasından yakalanan görüntülere Türkçe seslendirme eklemek için `edge-tts` (`tr-TR-AhmetNeural`) ve altyazı gömme/miksaj için `FFmpeg`/`FFprobe` kullanılmıştır.

## 3. İnsan Doğrulaması ve Sorumluluk

Tüm algoritmalar, simülasyon fiziği ve güvenlik kısıtları proje ekibi tarafından çalıştırılarak doğrulanmıştır. Projeye ilişkin tüm teknik kararlar, test onayları ve nihai teslim sorumluluğu **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran**'a aittir.
