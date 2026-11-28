# DYNAMIC-NAV — Öğrenci Kontrol Listesi ve Proje Seçim Bildirimi

## 1. Proje Seçim Bildirimi

| Alan | Değer |
|---|---|
| **Seçilen Proje Kodu** | `P2` |
| **Ekip Adı** | `DYNAMIC-NAV` |
| **Üye 1** | Eren Çakmaz |
| **Üye 2** | Ahmet Gürlek |
| **Üye 3** | Arda Başaran |
| **Üye 4** | *(Boş)* |
| **Danışman** | Doç. Dr. Ali Furkan Kamanlı |
| **Tercih Edilen Geliştirme Ortamı** | Python 3.12.10, Pygame (`pygame 2.6.1`) |
| **Teslim Tarihi** | 24 Kasım 2026 |

---

## 2. Teslim Öncesi Kontrol Listesi

| # | Teslim Maddesi | Dosya / Kanıt | Durum |
|---|---|---|---|
| 1 | **Kaynak Kod** | `config.py`, `simulation/`, `algorithms/`, `interface/`, `analysis/`, `main.py` | **TAMAMLANDI** |
| 2 | **README (Kurulum, Çalıştırma, Bağımlılıklar)** | [`README.md`](../README.md) (`Windows` / `Linux` / `macOS` yönergeleri, sorun giderme) | **TAMAMLANDI** |
| 3 | **Tek Nihai Teknik Rapor (MD + PDF, 4–6 sayfa)** | [`DYNAMIC-NAV_Teknik_Rapor.md`](DYNAMIC-NAV_Teknik_Rapor.md), [`DYNAMIC-NAV_Teknik_Rapor.pdf`](DYNAMIC-NAV_Teknik_Rapor.pdf) (**6 sayfa** A4) | **TAMAMLANDI** |
| 4 | **Deneysel Sonuçlar Özeti** | [`Deneysel_Sonuclar.md`](Deneysel_Sonuclar.md) + `analysis/results/` ham JSON/CSV kanıtları | **TAMAMLANDI** |
| 5 | **Genellenebilirlik & Örnek Senaryo JSON** | [`examples/custom_scenario_example.json`](../examples/custom_scenario_example.json) + `main.py --scenario-json` | **TAMAMLANDI** |
| 6 | **Nihai Demo Videosu (3–5 dk)** | [`Demo_Video.mp4`](Demo_Video.mp4) (`229.73 s` = `03:49.73`, `1920x1080`, Türkçe ses + altyazı) | **TAMAMLANDI** |
| 7 | **Otomatik Testler & CI** | `tests/` — mevcut çalışma kopyasında **`206` yerel `pytest` testi geçti**; tarihsel GitHub Actions kayıtlarında **`202` test geçti** (`Python 3.12`) | **TAMAMLANDI** |
| 8 | **AI Kullanım Beyanı (MD + PDF, 1 sayfa)** | [`AI_Kullanim_Beyani.md`](AI_Kullanim_Beyani.md), [`AI_Kullanim_Beyani.pdf`](AI_Kullanim_Beyani.pdf) (`1` sayfa A4) | **TAMAMLANDI** |
| 9 | **Ekip Katkı Tablosu (MD + PDF, 1 sayfa)** | [`Ekip_Katki_Tablosu.md`](Ekip_Katki_Tablosu.md), [`Ekip_Katki_Tablosu.pdf`](Ekip_Katki_Tablosu.pdf) (`1` sayfa A4, tüm ekip onayları tamamlandı) | **TAMAMLANDI** |
| 10 | **Arşivlenmiş Ara Sürümler** | `archive/demo_versions/` (3 ara video), `archive/historical_reports/` (`phase71_akademik_rapor.*`, `phase73_rapor.md`) | **TAMAMLANDI** |

---

## 3. GitHub Deposu ve Teslim Paketi Erişimi

- **GitHub Deposu:** [https://github.com/zeosapka/DYNAMIC-NAV](https://github.com/zeosapka/DYNAMIC-NAV)

Teslim edilen `DYNAMIC-NAV_delivery.zip` arşivi ile yukarıdaki GitHub deposu birbirini tamamlayan iki erişim noktasıdır: ZIP paketi çevrimdışı inceleme ve arşivleme için tam teslim içeriğini sunarken, GitHub deposu kaynak kodlara, geliştirme geçmişine, dokümantasyona ve güncel sürüme çevrimiçi erişim sağlar.
