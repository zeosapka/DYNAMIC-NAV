# Faz 7.3.2 — Tipografi ve Bilimsel Yayın Düzeni Denetimi

**Denetlenen PDF'ler:** `phase71_akademik_rapor.pdf`, `phase73_ekip_katki_tablosu.pdf`, `phase73_yapay_zeka_kullanim_beyani.pdf`

---

## 1. Font Gömme Doğrulaması (`pypdf` `/BaseFont` İncelemesi)

| Belge | Gömülü Fontlar | Times New Roman | Cambria Math | Consolas |
|---|---|---|---|---|
| `phase71_akademik_rapor.pdf` | `TimesNewRomanPSMT`, `TimesNewRomanPS-BoldMT`, `TimesNewRomanPS-ItalicMT`, `CambriaMath`, `Consolas` | DOĞRULANDI | DOĞRULANDI | DOĞRULANDI |
| `phase73_ekip_katki_tablosu.pdf` | `TimesNewRomanPSMT`, `TimesNewRomanPS-BoldMT`, `Consolas` | DOĞRULANDI | N/A | DOĞRULANDI |
| `phase73_yapay_zeka_kullanim_beyani.pdf` | `TimesNewRomanPSMT`, `TimesNewRomanPS-BoldMT`, `Consolas` | DOĞRULANDI | N/A | DOĞRULANDI |

## 2. Sayfa Düzeni ve Bilimsel Estetik

- **Sayfa Boyutu ve Kenar Boşlukları:** Tüm belgeler A4 (`595 × 842 pt`), `20 mm / 19 mm` dengeli kenar boşluklu ve alt ortada sayfa numaralıdır (`Sayfa X / Y`).
- **Renk ve Tablo Tasarımı:** Koyu mavi sunum şeritleri ve yoğun mavi tablo başlıkları kaldırılmış; beyaz zemin, siyah/koyu gri metin, ince gri tablo çizgileri ve baskıda okunabilir bilimsel grafikler (`Şekil 1–3`) uygulanmıştır.
- **Paragraf ve Türkçe Karakter:** Gövde metni `11,5 pt` (`1,48` satır aralığı), iki yana yaslı ve tutarlı ilk satır girintilidir (`1,2 em`); Türkçe karakterler (`ğ, ü, ş, ı, ö, ç, İ`) tam doğrulanmıştır.
