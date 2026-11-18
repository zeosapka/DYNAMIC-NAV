# Faz 7.2 — Ekran Kaydı Kontrol Listesi

## 1. Kayıt Öncesi Hazırlık

- [ ] **Sanal Ortam ve Uygulama Başlatma:** Proje kök dizininde (`C:\Users\Legolas\Desktop\DYNAMIC-NAV`) uygulamayı başlat:
  ```powershell
  .\.venv\Scripts\python.exe main.py
  ```
- [ ] **Pencere ve Ekran Kayıt Ayarları (OBS Studio / Ekran Kaydedici):**
  - Çözünürlük: `1920x1080` veya `1440x900` (pencere tam görünür olmalı, sağdaki güvenlik paneli kesilmemeli).
  - Kare hızı (FPS): `30` veya `60 FPS`.
  - Ses girişi: Net mikrofon kaydı (arka plan gürültüsü kapalı), Türkçe altyazı katmanı hazır.
- [ ] **Gösterilecek Senaryoların Hazırlığı:**
  - Arayüz sol menü ve **ANA MENÜ** butonları üzerinden doğrudan açılanlar: `KARŞIDAN GELEN` (`HEAD-ON`), `ÇAPRAZ GEÇİŞ` (`CROSSING`), `KORİDOR` (`CORRIDOR`), `DUR-KALK` (`STOP-AND-GO`).
  - `NARROW-GATE` senaryosunu canlı Pygame penceresinde tek komutla (kaynak kod değiştirmeden) açmak için salt-okunur başlatıcı:
    ```powershell
    .\.venv\Scripts\python.exe -c "from config import DEFAULT_CONFIG; from interface.app import DynamicNavApp; from interface.views import AppView; app = DynamicNavApp(DEFAULT_CONFIG, initial_view=AppView.SIMULATION); app.load_scenario('NARROW-GATE'); app.plan_path(); app.run()"
    ```
- [ ] **Deneysel Sonuç Ekranı:** `analysis/results/phase656_final_audit_report.md` dosyasını veya Faz 6.5.6 sonuç tablosunu ayrı bir pencerede hazır tut.

---

## 2. Çekim Sırası Kontrol Listesi (0:00 – 4:45)

- [ ] **0:00 – 0:30 (Proje Tanıtımı):** `ANA MENÜ` ekranını (`DYNAMIC NAV`) tam göster.
- [ ] **0:30 – 1:15 (Sistem Mimarisi ve A\*):**
  - `ALGORİTMA BİLGİSİ` (`KILAVUZ`) sekmesini 5–8 sn göster.
  - Simülasyon ekranına geç; `8-YÖN` butonunun aktif olduğunu ve `A* PLANLA` ile şişirilmiş ızgara üzerindeki düzleştirilmiş A\* rotasını göster.
- [ ] **1:15 – 2:15 (DWA ve Dinamik Engel Kaçınma):**
  - Üst çubukta `MOD: DWA` aktif olsun.
  - `KARŞIDAN GELEN` (`HEAD-ON`) ve `DUR-KALK` (`STOP-AND-GO`) senaryolarında `BAŞLAT` butonuna bas.
  - Sağdaki **Güvenlik ve Telemetri Paneli**'nin (`GÜVENLİ / DİKKAT / KRİTİK`, mesafe, hız) net okunduğundan emin ol.
- [ ] **2:15 – 3:15 (Senaryo Gösterimleri: `CORRIDOR` ve `NARROW-GATE`):**
  - Sol menüden `KORİDOR` (`CORRIDOR`) seçip `BAŞLAT` ile duvar payı + dinamik kaçınmayı kaydet.
  - `NARROW-GATE` penceresinde `BAŞLAT` ile robotun dar geçit önünde duruşunu, beklemesini, yön değiştirmesini ve geçit çıkışı sonrası yanal sollama manevrasını kaydet.
- [ ] **3:15 – 4:15 (Deneysel Sonuçlar):**
  - `191/191` pytest (`175,52 s`), `8/8` temel senaryo (`min_swept = 0,152024 m`, ort. DWA `11,85 ms`) ve `14/14` `NARROW-GATE` (`42` koşum, `0` çarpışma, `0` pay ihlali, `min_swept = 0,150019 m`, `%100` determinizm, medyan `5,58 ms`) verilerini ekranda göster.
- [ ] **4:15 – 4:45 (Sonuç ve Ekip):**
  - Elektrik-Elektronik Mühendisliği ekip kartını (Eren Çakmaz, Ahmet Gürlek, Arda Başaran) ve kapanış özetini göster.

---

## 3. Kayıt Sonrası Kalite Kontrolü

- [ ] Toplam video süresi `03:00` – `05:00` aralığında (`04:45` hedef).
- [ ] Tüm görüntüler gerçek uygulama kaydı ve gerçek Faz 6.5.6 verilerinden oluşuyor (sahte/üretilmiş ekran yok).
- [ ] Türkçe seslendirme ile Türkçe altyazılar birebir senkronize ve yazım hatası yok.
- [ ] Hiçbir kaynak kod, konfigürasyon veya test dosyası değiştirilmedi.
