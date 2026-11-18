# DYNAMIC-NAV — Proje Ekibi ve Önerilen Sorumluluk Paylaşımı

**Proje Başlığı:** DYNAMIC-NAV: Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Üniversite:** [Üniversite Adı Giriniz]  
**Danışman:** [Danışman Unvanı ve Adı Giriniz]  
**Teslim Tarihi:** [GG.AA.YYYY]  

---

## Ortak Çalışma Beyanı

Bu proje **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran** tarafından ortak mühendislik çalışması olarak geliştirilmiştir. Aşağıda sunulan görev dağılımı, sunum ve teslim düzeni amacıyla hazırlanmış **önerilen sorumluluk paylaşımı** niteliğindedir; her üç ekip üyesi de yazılım geliştirme, birim ve senaryo testlerinin yürütülmesi, hata ayıklama, güvenlik doğrulaması ve deneysel değerlendirme süreçlerinin tümüne ortak katkı sağlamıştır.

---

## Önerilen Sorumluluk Paylaşımı Tablosu

| Ekip Üyesi | Önerilen Odak Alanı | Sorumlu Olduğu Modüller / Bileşenler | Somut Katkılar ve Çıktılar |
|---|---|---|---|
| **Eren Çakmaz** | Simülasyon Çekirdeği, Kinematik Model ve Global Yol Planlama | `config.py`, `simulation/entities.py`, `simulation/world.py`, `algorithms/astar.py` | - SI birim sistemine dayalı merkezi konfigürasyon yapısının (`WorldConfig`, `RobotConfig`, `DWAConfig`, `ExperimentConfig`) tasarlanması<br>- Orta nokta yönelim integrasyonu (midpoint heading integration) ile diferansiyel sürüş kinematik modelinin ve sürekli zamanlı çarpışma/mesafe hesaplarının geliştirilmesi<br>- 8-yönlü A\* global yol planlayıcısı, Öklid engel şişirme (inflation) ve çizgi-parçası tabanlı yol düzleştirme mekanizmasının uygulanması |
| **Ahmet Gürlek** | Yerel Planlama (DWA), Güvenlik Doğrulaması ve Geri Çekilme (Fallback) Yönetimi | `algorithms/dwa.py`, `simulation/scenarios.py`, `interface/` | - Dinamik Pencere Yaklaşımı (DWA) hız penceresi örneklemesi ve çok terimli yörünge puanlama fonksiyonunun geliştirilmesi<br>- Sürekli süpürme mesafesi (swept-clearance) kontrolü, aktif frenleme duruş doğrulaması ve güvenli bekleme/geri çekilme hiyerarşisinin tasarlanması<br>- 8 dinamik engel senaryosunun ve Pygame görselleştirme arayüzünün oluşturulması |
| **Arda Başaran** | Deneysel Kıyaslama (Benchmark), Test Süiti, Güvenlik Denetimi ve Dokümantasyon | `analysis/`, `tests/`, `README.md`, Faz 6.5.6–7.1 raporları | - Bağımsız çarpışma ve süpürme açıklığı metriklerinin (`compute_swept_clearance_m`) ve otomatik test süitinin (`191` pytest testi) geliştirilmesi<br>- Temel senaryolar, `NARROW-GATE` Seviye 1 varyantları (42 koşum) ve DWA gecikme profillerinin ölçülmesi ve determinizm doğrulaması<br>- Sistem bütünlük denetimi, Türkçe teknik dokümantasyon (`README.md`) ve akademik proje raporunun hazırlanması |
