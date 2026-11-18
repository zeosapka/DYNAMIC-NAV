# DYNAMIC-NAV — Ekip Katkı ve Doğrulama Tablosu

**Proje Başlığı:** DYNAMIC-NAV — P2 Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi  
**Bölüm:** Elektrik-Elektronik Mühendisliği  
**Referans Belge:** [phase71_ekip_gorev_dagilimi.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase71_ekip_gorev_dagilimi.md)  
**Üniversite / Danışman / Tarih:** [Üniversite Adı Giriniz] | [Danışman Adı Giriniz] | [GG.AA.YYYY]  

---

## 1. Doğrulama Notu (Kayıt İncelemesi)

Mevcut proje dizininde bireysel geliştirici bazlı sürüm kontrolü (`git`) geçmişi bulunmadığından, dosya ve modül sahipliği depo kayıtlarından tekil kişi düzeyinde bağımsız olarak ayrılamamaktadır. Bu nedenle aşağıda `phase71_ekip_gorev_dagilimi.md` belgesindeki **önerilen sorumluluk paylaşımı** korunmuş; bireysel katkı maddeleri **ekip onayı gerektiren kalemler (`[EKİP ONAYI GEREKLİ]`)** olarak açıkça işaretlenmiştir. Her üç üye de ortak geliştirme, test ve değerlendirme süreçlerine katkı sağlamıştır.

---

## 2. Ekip Katkı Tablosu

| Ekip Üyesi | Önerilen Odak Alanı | İlgili Modüller / Çıktılar | Depo Kayıt Durumu | Onay Durumu |
|---|---|---|---|---|
| **Eren Çakmaz** | Simülasyon Çekirdeği, Kinematik Model ve Global Yol Planlama | `config.py`, `simulation/entities.py`, `simulation/world.py`, `algorithms/astar.py` | Modüller mevcut ve `191/191` pytest kapsamında doğrulanmış; kişi bazlı git kaydı yok. | `[EKİP ONAYI GEREKLİ]` |
| **Ahmet Gürlek** | Yerel Planlama (DWA), Güvenlik Doğrulaması, Senaryolar ve Arayüz | `algorithms/dwa.py`, `simulation/scenarios.py`, `interface/` | Modüller mevcut, 8/8 senaryo ve UI butonları test edilmiş; kişi bazlı git kaydı yok. | `[EKİP ONAYI GEREKLİ]` |
| **Arda Başaran** | Deneysel Kıyaslama, Test Süiti, Güvenlik Denetimi ve Dokümantasyon | `analysis/`, `tests/`, `README.md`, Faz 6.5.6–7.3 raporları | Tüm test/denetim kayıtları ve Türkçe raporlar mevcut; kişi bazlı git kaydı yok. | `[EKİP ONAYI GEREKLİ]` |

---

## 3. Ortak Ekip Onayı

Aşağıdaki imzalar ile proje ekibi, yukarıdaki önerilen sorumluluk paylaşımını ve ortak çalışma beyanını onaylar:

- **Eren Çakmaz:** `[İmza / Tarih]`
- **Ahmet Gürlek:** `[İmza / Tarih]`
- **Arda Başaran:** `[İmza / Tarih]`
