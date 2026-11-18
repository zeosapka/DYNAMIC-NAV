# Faz 7.2 — DYNAMIC-NAV Türkçe Anlatım Metni ve Altyazılar

**Toplam Süre:** `04:45`  
**Dil:** Türkçe seslendirme ve senkronize Türkçe altyazı  

---

## 0:00 – 0:30 | Proje Tanıtımı

**Seslendirme Metni:**  
"Merhaba. Biz Elektrik-Elektronik Mühendisliği öğrencileri Eren Çakmaz, Ahmet Gürlek ve Arda Başaran. Bu videoda, dinamik ve dar ortamlarda otonom mobil robot navigasyonu için geliştirdiğimiz **DYNAMIC-NAV** simülasyon sistemini tanıtacağız. Gerçek dünyada mobil robotlar yalnızca sabit duvarlarla değil, aniden durabilen, yön değiştiren veya dar geçitlere giren hareketli engellerle de karşılaşır. DYNAMIC-NAV, tüm fiziksel hesaplamaları katı SI birimleriyle yürüten, çarpışmasız ve deterministik bir navigasyon mimarisi sunar."

**Altyazı Blokları:**
- `[00:00 - 00:15]` Elektrik-Elektronik Mühendisliği — DYNAMIC-NAV: Dinamik Engellerden Kaçınan Mobil Robot Simülasyon Sistemi.
- `[00:15 - 00:30]` Katı SI birimleri, deterministik fizik motoru ve gerçek zamanlı güvenlik doğrulaması.

---

## 0:30 – 1:15 | Sistem Mimarisi ve A\* Küresel Planlama

**Seslendirme Metni:**  
"Sistemimiz dört katmanlı bir mimariye sahiptir: `config.py` tüm parametrelerin tek doğruluk kaynağıdır; `simulation` katmanı `pygame` bağımlılığı olmadan kinematik ve engelleri `0,05 saniye` adımlarla ilerletir; `algorithms` katmanı A\* ve DWA planlayıcılarını içerir; `interface` ve `analysis` katmanları ise görselleştirme ile bağımsız güvenlik denetimini yürütür.  
Şimdi simülasyon ekranında **A\* PLANLA** komutuna bakalım. `algorithms/astar.py`, `20 m × 15 m` dünyayı `0,2 m` çözünürlüklü bir ızgaraya böler. Statik engeller, robot yarıçapı `0,35 m` ve `0,15 m` güvenlik payı toplamı olan `0,50 m` kadar kesin Öklid mesafesiyle şişirilir. **8-YÖN** modunda oktil sezgisel fonksiyonu kullanılarak en kısa yol bulunur ve çizgi-parçası kontrolüyle düzleştirilir."

**Altyazı Blokları:**
- `[00:30 - 00:50]` 4 katmanlı mimari: `config.py`, `simulation/`, `algorithms/`, `interface/` ve `analysis/`.
- `[00:50 - 01:15]` 8-yönlü A*: `0,2 m` ızgara, `0,50 m` Öklid engel şişirme, oktil sezgisel ve yol düzleştirme.

---

## 1:15 – 2:15 | DWA ve Dinamik Engel Kaçınma

**Seslendirme Metni:**  
"Küresel yol statik haritayı çözer; ancak hareketli engeller için yerel planlayıcıya ihtiyacımız vardır. **MOD: DWA** aktifken `KARŞIDAN GELEN` ve `DUR-KALK` senaryolarını çalıştıralım. `algorithms/dwa.py`, robotun doğrusal ve açısal ivme sınırları içinde ulaşabileceği hız penceresini `7 × 17`, yani `119` aday yörüngeye örnekler ve her adayı `1,2 saniye` ufuk boyunca ileri benzetir.  
Sağ taraftaki **Güvenlik ve Telemetri Paneli**'nde robotun anlık hızını, en yakın engel mesafesini ve güvenlik durumunu (`GÜVENLİ`, `DİKKAT`, `KRİTİK`) canlı izleyebiliyoruz. Robot yalnızca ayrık adımlarda değil, sürekli süpürme açıklığı boyunca `0,15 m` güvenlik payını korur ve `compute_stopping_distance_m` ile aktif frenleme mesafesini her adımda doğrular. Önündeki engel aniden yavaşlasa veya dursa bile robot güvenli takip ve frenleme mesafesini koruyarak çarpışmadan kaçar."

**Altyazı Blokları:**
- `[01:15 - 01:38]` DWA Yerel Planlama: `7 × 17 = 119` hız örneği ve `1,2 s` ileri yörünge benzetimi.
- `[01:38 - 02:15]` Sağ panel: Anlık hız, `0,15 m` sürekli süpürme payı ve aktif frenleme güvenliği.

---

## 2:15 – 3:15 | Senaryo Gösterimleri (`CORRIDOR` ve `NARROW-GATE`)

**Seslendirme Metni:**  
"Şimdi sistemin en zorlu iki senaryosuna geçelim. İlk olarak **KORİDOR** (`CORRIDOR`) senaryosunda robot, dar duvarların `0,15 m` güvenlik sınırını ihlal etmeden karşıdan veya önden gelen dinamik engelle geçiş manevrasını başarıyla tamamlar.  
İkinci olarak **NARROW-GATE** senaryosunu canlı izleyelim. Robot dar geçit önünde karşıdan gelen engeli algıladığında doğrudan geçide girmek yerine öncelikli geri çekilme hiyerarşisini devreye alır; `SAFE_STANDSTILL_BRAKING` ile `2,5 saniyelik` bekleme ufku boyunca güvenli bir bekleme noktasında durur. Engel geçidi terk ettikten sonra robot yönünü yeniden ayarlar ve geçit çıkışında yavaş bir engelle karşılaşırsa `CorridorPassingManager` sayesinde açık alanda proaktif yanal sollama yaparak alternatif güvenli rotadan hedefine ulaşır."

**Altyazı Blokları:**
- `[02:15 - 02:40]` `CORRIDOR`: Dar duvarlar arasında statik güvenlik payı ve dinamik engel kaçınması.
- `[02:40 - 03:15]` `NARROW-GATE`: `SAFE_STANDSTILL_BRAKING` (`2,5 s` bekleme), yön değiştirme ve proaktif sollama.

---

## 3:15 – 4:15 | Deneysel Sonuçlar (Faz 6.5.6)

**Seslendirme Metni:**  
"Sonuçlarımız Faz 6.5.6 Nihai Sistem Bütünlüğü ve Regresyon Denetimi kapsamında DWA'dan tamamen bağımsız `analysis/metrics.py` denetçisiyle doğrulanmıştır.  
Tüm birim ve regresyon testlerini kapsayan `191/191` `pytest` testi `175,52 saniyede` sıfır hatayla geçmiştir.  
8 temel dinamik engel senaryosunun tamamı `GOAL_REACHED` ile sonuçlanmış; sıfır çarpışma ve sıfır güvenlik payı ihlali ile en düşük süpürme açıklığı `0,152024 metre` olarak ölçülmüştür.  
En zorlu `14` `NARROW-GATE` Seviye 1 varyantının 3'er tekrarlı toplam `42` koşumunda ise `0` çarpışma, `0` güvenlik payı ihlali, `0,150019 metre` minimum süpürme açıklığı ve `%100` fiziksel determinizm elde edilmiştir. `19.014` DWA çağrısında medyan hesaplama süresi `5,58 milisaniye`dir."

**Altyazı Blokları:**
- `[03:15 - 03:35]` Faz 6.5.6: `191/191` pytest testi geçti (`175,52 s`, `0` hata).
- `[03:35 - 03:55]` `8/8` temel senaryo `GOAL_REACHED`, `0` çarpışma, min. süpürme `0,152024 m`.
- `[03:55 - 04:15]` `14/14` `NARROW-GATE` varyantı (`42` koşum): `0` ihlal, `%100` determinizm, medyan `5,58 ms`.

---

## 4:15 – 4:45 | Sonuç ve Ekip

**Seslendirme Metni:**  
"Özetle DYNAMIC-NAV; SI birim bütünlüğü, orta nokta kinematiği, Öklid şişirmeli A\* ve sürekli süpürme doğrulamalı DWA mimarisiyle güvenli otonom navigasyon sağlar. Gelecek çalışmalarda yoğun dar geçitlerdeki P99 kuyruk gecikmesini (`232,48 ms`) erken yörünge budamasıyla daha da düşürmeyi hedefliyoruz. Bu proje Eren Çakmaz, Ahmet Gürlek ve Arda Başaran'ın ortak mühendislik çalışmasıyla geliştirilmiştir. Dinlediğiniz için teşekkür ederiz."

**Altyazı Blokları:**
- `[04:15 - 04:32]` SI birimleri + A* + sürekli süpürme doğrulamalı DWA = Sıfır çarpışma, `%100` determinizm.
- `[04:32 - 04:45]` Elektrik-Elektronik Mühendisliği | Eren Çakmaz, Ahmet Gürlek, Arda Başaran.
