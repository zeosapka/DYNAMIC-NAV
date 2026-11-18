# Faz 7.2.1 — DYNAMIC-NAV Gerçek Demo Videosu Üretim Raporu

## 1. Üretilen Nihai Çıktılar

- **Demo Videosu (MP4):** [phase72_dynamic_nav_demo.mp4](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase72_dynamic_nav_demo.mp4)
- **Türkçe Altyazı Dosyası (SRT):** [phase72_altyazilar.srt](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase72_altyazilar.srt)
- **Üretim ve Doğrulama Raporu:** [phase72_video_uretim_raporu.md](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase72_video_uretim_raporu.md)
- **Geçici Kayıt ve Yakalama Betiği:** `demo_assets/build_demo_video.py`, `demo_assets/raw_video.mp4`

---

## 2. FFprobe Teknik Doğrulama Sonucu

Nihai MP4 dosyası üzerinde `ffprobe` ile doğrulanan teknik özellikler:

| Özellik | Doğrulanan Değer |
|---|---|
| Dosya Yolu | `C:\Users\Legolas\Desktop\DYNAMIC-NAV\analysis\results\phase72_dynamic_nav_demo.mp4` |
| Kapsayıcı (Container) | `mov,mp4,m4a,3gp,3g2,mj2` (`+faststart`) |
| Toplam Süre | `275.000000 s` (`04:35` — `04:00–05:00` hedef aralığında) |
| Dosya Boyutu | `8.156.538` bayt (`~7,78 MiB`) |
| Video Akışı (`index=0`) | `h264` (`libx264`, `yuv420p`), `1920x1080`, `30/1 FPS` |
| Ses Akışı (`index=1`) | `aac`, `44100 Hz`, `2` kanal (stereo) |
| Altyazı | UTF-8 `phase72_altyazilar.srt` videoya gömülü (burned-in, Türkçe karakterler doğrulandı) |

---

## 3. Gerçek Uygulama Sahne Akışı (04:35)

Tüm simülasyon ve arayüz görüntüleri doğrudan `DynamicNavApp` (`interface/app.py`) Pygame yüzeyinden yakalanmıştır:

1. **`00:00 – 00:25` (`HOME`):** `AppView.HOME` Ana Menü ve proje tanıtımı.
2. **`00:25 – 01:00` (`GUIDE`):** `AppView.GUIDE` (`ALGORİTMA BİLGİSİ` ve `GÜVENLİK`) sekmeleri.
3. **`01:00 – 01:40` (`HEAD-ON`):** `499` canlı simülasyon adımı ile karşıdan gelen engele karşı DWA yanal kaçınması.
4. **`01:40 – 02:20` (`CROSSING`):** `453` canlı simülasyon adımı ile çapraz hareket eden engeller karşısında hız ve yönelim uyarlaması.
5. **`02:20 – 03:00` (`CORRIDOR`):** `626` canlı simülasyon adımı ile dar koridorda `0,15 m` duvar güvenlik payı korunarak geçiş.
6. **`03:00 – 03:45` (`NARROW-GATE`):** `720` canlı simülasyon adımı ile dar geçit önünde güvenli bekleme (`SAFE_STANDSTILL_BRAKING`), yön yeniden ayarlama ve çıkış sonrası hedefe ulaşım.
7. **`03:45 – 04:05` (Telemetri & Güvenlik Paneli):** Gerçek koşumlardan alınan sağ panel yakın planları (`6` canlı anlık görüntü).
8. **`04:05 – 04:35` (Faz 6.5.6 Doğrulanmış Sonuçlar & Ekip):** `191/191` `pytest`, `8/8` temel senaryo, `14/14` `NARROW-GATE` Seviye 1 (`42` koşum), `0` çarpışma, `0` pay ihlali, `%100` determinizm (yalnızca Faz 6.5.6 test kapsamına ait olduğu açıkça belirtilmiştir) ve Elektrik-Elektronik Mühendisliği ekip bilgisi (Eren Çakmaz, Ahmet Gürlek, Arda Başaran).

---

## 4. Türkçe Seslendirme Durumu ve Ekleme Adımı

- **Mevcut Durum (Şeffaf Bildirim):** Bu sistemdeki Windows `System.Speech` sentezleyicisinde yalnızca `en-US` (`Microsoft Zira Desktop`) sesi yüklü olup yerleşik Türkçe (`tr-TR`) TTS sesi bulunmamaktadır. Bu nedenle oluşturulan MP4 dosyası uyumlu bir `44100 Hz` stereo sessiz AAC ses akışı ve videoya gömülü senkron Türkçe altyazı içermektedir.
- **Türkçe Seslendirme Ekleme Adımı:** `analysis/results/phase72_anlatim_metni.md` metni mikrofonla (`turkce_anlatim.wav` veya `.mp3` olarak) kaydedildikten sonra aşağıdaki komutla videoya doğrudan eklenebilir:
  ```powershell
  ffmpeg -y -i analysis\results\phase72_dynamic_nav_demo.mp4 -i turkce_anlatim.wav -map 0:v:0 -map 1:a:0 -c:v copy -c:a aac -b:a 192k -shortest analysis\results\phase72_dynamic_nav_demo_sesli.mp4
  ```

---

## 5. Koruma ve Güvenlik Doğrulaması

- Hiçbir Python kaynak dosyası, `config.py`, test dosyası veya tarihsel Faz 6.5.6 deneysel sonucu değiştirilmemiştir.
- Tüm simülasyon sahneleri doğrudan `DynamicNavApp` çalıştırılarak elde edilmiştir; yapay veya sahte metrik/animasyon kullanılmamıştır.
