# DYNAMIC-NAV — Proje Ekibi Görev Dağılımı

**Proje Başlığı:** DYNAMIC-NAV: Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Üniversite:** Sakarya Uygulamalı Bilimler Üniversitesi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Proje Ekibi:** Eren Çakmaz, Ahmet Gürlek, Arda Başaran  

---

## Ortak Çalışma Çerçevesi

Bu proje **Eren Çakmaz**, **Ahmet Gürlek** ve **Arda Başaran** tarafından ortak mühendislik çalışması olarak geliştirilmiştir. Aşağıdaki tablo projenin modüler yapısına göre oluşturulmuş **proje görev dağılımını** özetlemektedir; her üç ekip üyesi de yazılım geliştirme, testlerin yürütülmesi, hata ayıklama, güvenlik doğrulaması ve deneysel değerlendirme süreçlerine ortak katkı sağlamıştır.

---

## Proje Görev Dağılımı Tablosu

| Ekip Üyesi | Odak Alanı | İlgili Modüller ve Bileşenler | Sorumluluk Kapsamı |
|---|---|---|---|
| **Eren Çakmaz** | Simülasyon Çekirdeği, Kinematik Model ve Küresel Yol Planlama | `config.py`, `simulation/entities.py`, `simulation/world.py`, `algorithms/astar.py` | - SI birim sistemine dayalı merkezi konfigürasyon yapısının (`WorldConfig`, `RobotConfig`, `DWAConfig`, `ExperimentConfig`) düzenlenmesi<br>- Orta nokta yönelim integrasyonu ile diferansiyel sürüş kinematik modelinin ve sürekli zamanlı çarpışma/mesafe hesaplarının uygulanması<br>- 8-yönlü A\* küresel yol planlayıcısı, Öklid engel şişirme ve çizgi-parçası tabanlı yol düzleştirme mekanizmasının uygulanması |
| **Ahmet Gürlek** | Yerel Planlama (DWA), Güvenlik Doğrulaması ve Arayüz | `algorithms/dwa.py`, `simulation/scenarios.py`, `interface/` | - Dinamik Pencere Yaklaşımı (DWA) hız penceresi örneklemesi ve çok terimli yörünge puanlama fonksiyonunun geliştirilmesi<br>- Sürekli süpürme açıklığı kontrolü, aktif frenleme duruş doğrulaması ve güvenli bekleme/geri çekilme hiyerarşisinin uygulanması<br>- 8 dinamik engel senaryosunun ve Pygame görselleştirme arayüzünün geliştirilmesi |
| **Arda Başaran** | Deneysel Kıyaslama, Test Süiti, Güvenlik Denetimi ve Dokümantasyon | `analysis/`, `tests/`, `README.md`, Faz 6.5.6–7.3 belgeleri | - Bağımsız çarpışma ve süpürme açıklığı metriklerinin (`compute_swept_clearance_m`) ve otomatik test süitinin (`191` `pytest` testi) yürütülmesi<br>- Temel senaryolar, `NARROW-GATE` Seviye 1 varyantları (`42` koşum) ve DWA gecikme profillerinin ölçülmesi ve determinizm doğrulaması<br>- Sistem bütünlük denetimi, Türkçe teknik dokümantasyon (`README.md`) ve akademik proje raporunun hazırlanması |
