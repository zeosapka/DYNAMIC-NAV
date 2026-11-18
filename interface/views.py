"""Home screen (ANA MENÜ) and Interactive User Guide (KULLANIM KILAVUZU) views."""

from enum import Enum, auto
import pygame

from config import ProjectConfig
from interface.controls import Button


class AppView(Enum):
    """Top-level application views."""

    HOME = auto()
    SIMULATION = auto()
    GUIDE = auto()


class GuideTab(Enum):
    """Tabs inside the interactive Turkish user guide."""

    TUTORIAL = auto()
    ALGORITHMS = auto()
    SAFETY = auto()


class HomeViewRenderer:
    """Renders the Turkish ANA MENÜ (Home Screen) in dark engineering style."""

    def __init__(self, config: ProjectConfig) -> None:
        self._config = config
        self._theme = config.theme
        self._font_hero = pygame.font.SysFont("segoeui", 40, bold=True)
        self._font_sub = pygame.font.SysFont("segoeui", 16)
        self._font_h2 = pygame.font.SysFont("segoeui", 15, bold=True)
        self._font_body = pygame.font.SysFont("segoeui", 13)
        self._font_small = pygame.font.SysFont("segoeui", 12)

    def render(
        self,
        surface: pygame.Surface,
        buttons: list[Button],
        has_active_sim: bool,
    ) -> None:
        """Render the complete Home screen."""
        surface.fill(self._theme.background)
        w = self._config.window.width

        # Top hero banner
        hero_rect = pygame.Rect(120, 48, w - 240, 128)
        pygame.draw.rect(surface, self._theme.sidebar_bg, hero_rect, border_radius=10)
        pygame.draw.rect(surface, self._theme.border, hero_rect, width=1, border_radius=10)

        title = self._font_hero.render("DYNAMIC NAV", True, self._theme.accent)
        surface.blit(title, (156, 68))
        sub = self._font_sub.render(
            "Dinamik Engellerden Kaçınma — Otonom Mobil Robot Navigasyon Simülatörü",
            True,
            self._theme.text_primary,
        )
        surface.blit(sub, (156, 120))
        desc = self._font_small.render(
            "SI birimli 2B fizik motoru, A* küresel rota planlama, geometrik güvenlik analizi ve senaryo simülasyonu.",
            True,
            self._theme.text_secondary,
        )
        surface.blit(desc, (156, 146))

        # Left column card: Simülasyon & Senaryolar
        left_card = pygame.Rect(120, 200, 620, 430)
        pygame.draw.rect(surface, self._theme.sidebar_bg, left_card, border_radius=10)
        pygame.draw.rect(surface, self._theme.border, left_card, width=1, border_radius=10)
        surface.blit(
            self._font_h2.render("SİMÜLASYON BAŞLAT VE SENARYO SEÇİMİ", True, self._theme.accent),
            (148, 220),
        )
        hint = (
            "Aktif bir simülasyon oturumu mevcut; kaldığınız yerden devam edebilirsiniz."
            if has_active_sim
            else "Yeni bir simülasyon başlatın veya hazır test senaryolarından birini seçin."
        )
        surface.blit(
            self._font_small.render(hint, True, self._theme.text_secondary),
            (148, 246),
        )
        surface.blit(
            self._font_h2.render("HAZIR TEST SENARYOLARI", True, self._theme.text_secondary),
            (148, 356),
        )

        # Right column card: Kılavuz & Proje Bilgisi
        right_card = pygame.Rect(764, 200, 556, 430)
        pygame.draw.rect(surface, self._theme.sidebar_bg, right_card, border_radius=10)
        pygame.draw.rect(surface, self._theme.border, right_card, width=1, border_radius=10)
        surface.blit(
            self._font_h2.render("REHBER VE ALGORİTMA BİLGİSİ", True, self._theme.accent),
            (792, 220),
        )

        # Project & Team Info panel inside right card
        info_box = pygame.Rect(792, 376, 500, 224)
        pygame.draw.rect(surface, self._theme.panel_bg, info_box, border_radius=8)
        pygame.draw.rect(surface, self._theme.border, info_box, width=1, border_radius=8)
        surface.blit(
            self._font_h2.render("PROJE VE EKİP BİLGİSİ", True, self._theme.text_primary),
            (812, 392),
        )
        info_lines = [
            "Proje: DYNAMIC NAV — Üniversite Mühendislik Projesi",
            "Mimari: Modüler Python 3.12 + Pygame + NumPy + Pytest",
            "Birim Sistemi: Tam SI (metre, saniye, radyan, m/s, rad/s)",
            "Mevcut Faz: Faz 3.2 (A*, Dinamik Engeller, Güvenlik Paneli)",
            "Planlanan Faz 4: DWA (Dynamic Window Approach) Yerel Kaçınma",
            "Geliştirme Ekibi: DYNAMIC NAV Mühendislik Ekibi",
        ]
        iy = 426
        for line in info_lines:
            surface.blit(
                self._font_body.render(line, True, self._theme.text_secondary),
                (812, iy),
            )
            iy += 26

        # Draw buttons
        for btn in buttons:
            btn.draw(surface, self._font_body, self._theme, active=False)

        # Bottom footer
        footer = self._font_small.render(
            "İpucu: Simülasyon sırasında istediğiniz zaman 'ANA MENÜ' butonuyla bu ekrana dönebilirsiniz.",
            True,
            self._theme.text_secondary,
        )
        surface.blit(footer, (120, 660))


class GuideViewRenderer:
    """Renders the interactive Turkish KULLANIM KILAVUZU view."""

    def __init__(self, config: ProjectConfig) -> None:
        self._config = config
        self._theme = config.theme
        self._font_title = pygame.font.SysFont("segoeui", 26, bold=True)
        self._font_h2 = pygame.font.SysFont("segoeui", 16, bold=True)
        self._font_body = pygame.font.SysFont("segoeui", 14)
        self._font_small = pygame.font.SysFont("segoeui", 12)

    def render(
        self,
        surface: pygame.Surface,
        buttons: list[Button],
        active_tab: GuideTab,
    ) -> None:
        """Render the Turkish User Guide with tabbed content."""
        surface.fill(self._theme.background)
        w = self._config.window.width

        # Header bar
        header_rect = pygame.Rect(60, 28, w - 120, 64)
        pygame.draw.rect(surface, self._theme.sidebar_bg, header_rect, border_radius=8)
        pygame.draw.rect(surface, self._theme.border, header_rect, width=1, border_radius=8)
        surface.blit(
            self._font_title.render("KULLANIM KILAVUZU VE SİSTEM REHBERİ", True, self._theme.accent),
            (84, 42),
        )

        # Tab buttons & navigation buttons
        tab_labels = {
            GuideTab.TUTORIAL: "ADIM ADIM REHBER",
            GuideTab.ALGORITHMS: "ALGORİTMALAR",
            GuideTab.SAFETY: "GÜVENLİK GÖSTERGELERİ",
        }
        active_tab_label = tab_labels[active_tab]
        for btn in buttons:
            btn.draw(
                surface,
                self._font_body,
                self._theme,
                active=(btn.label == active_tab_label),
            )

        # Content panel
        content_rect = pygame.Rect(60, 166, w - 120, 690)
        pygame.draw.rect(surface, self._theme.sidebar_bg, content_rect, border_radius=10)
        pygame.draw.rect(surface, self._theme.border, content_rect, width=1, border_radius=10)

        if active_tab == GuideTab.TUTORIAL:
            self._draw_tutorial_tab(surface)
        elif active_tab == GuideTab.ALGORITHMS:
            self._draw_algorithms_tab(surface)
        else:
            self._draw_safety_tab(surface)

    def _draw_tutorial_tab(self, surface: pygame.Surface) -> None:
        surface.blit(
            self._font_h2.render("İLK ÇALIŞTIRMA VE ADIM ADIM KULLANIM REHBERİ", True, self._theme.accent),
            (92, 190),
        )
        steps = [
            ("1. Senaryo Seçimi:", "Sol paneldeki KARŞIDAN GELEN, ÇAPRAZ GEÇİŞ, KORİDOR veya DUR-KALK butonlarıyla hazır senaryo yükleyin."),
            ("2. Başlangıç Konumu:", "'BAŞLANGIÇ NOKTASI' modunu seçip haritada istediğiniz güvenli bir noktaya sol tıklayın."),
            ("3. Hedef Konumu:", "'HEDEF NOKTASI' modunu seçip haritada ulaşmak istediğiniz hedefe sol tıklayın."),
            ("4. Sabit Engel Ekleme/Silme:", "'ENGEL EKLE' modunda sol tık ile 1x1 m engel ekleyin; herhangi bir modda sağ tık ile kaldırın."),
            ("5. A* Rota Planlama:", "'A* ROTA PLANLA' butonuna basın. '8-YÖN' (Octile) ve '4-YÖN' (Manhattan) arasında geçiş yapabilirsiniz."),
            ("6. Otonom Hareketi Başlatma:", "'BAŞLAT' butonuna (veya Boşluk tuşuna) basarak robotun A* rotası boyunca ilerlemesini başlatın."),
            ("7. Duraklatma ve Devam Etme:", "'DURAKLAT' robotu mevcut konumunda durdurur; tekrar 'BAŞLAT' kalan rotadan devam ettirir."),
            ("8. Senaryoyu Sıfırlama:", "'SIFIRLA' (veya R tuşu) süreyi, robotu, dinamik engelleri ve güvenlik istatistiklerini başa alır."),
            ("9. Güvenlik Göstergeleri:", "Sağdaki güvenlik panelinden gerçek yüzey açıklığını (m), TTC süresini ve olay günlüğünü izleyin."),
            ("10. Klavye ile Manuel Sürüş:", "W/S veya Yukarı/Aşağı ile ileri-geri, A/D veya Sol/Sağ oklarla robotu manuel döndürebilirsiniz."),
        ]
        y = 232
        for title, body in steps:
            t_surf = self._font_body.render(title, True, self._theme.nav_blue)
            b_surf = self._font_body.render(body, True, self._theme.text_primary)
            surface.blit(t_surf, (92, y))
            surface.blit(b_surf, (320, y))
            y += 42

        # Quick first-run box
        box = pygame.Rect(92, 668, 1256, 152)
        pygame.draw.rect(surface, self._theme.panel_bg, box, border_radius=8)
        pygame.draw.rect(surface, self._theme.safety_safe, box, width=1, border_radius=8)
        surface.blit(
            self._font_h2.render("HIZLI BAŞLANGIÇ ÖZETİ (İLK KEZ KULLANANLAR İÇİN)", True, self._theme.safety_safe),
            (114, 684),
        )
        quick_lines = [
            "Adım 1: Ana Menü'den 'KARŞIDAN GELEN' veya 'YENİ SİMÜLASYON BAŞLAT' seçin.",
            "Adım 2: Harita üzerinde sarı A* küresel rotasını ve turuncu dinamik engelleri gözlemleyin.",
            "Adım 3: 'BAŞLAT' butonuna tıklayın — robot rotayı izler, engel yaklaşınca güvenli şekilde durur.",
        ]
        qy = 718
        for ql in quick_lines:
            surface.blit(self._font_body.render(ql, True, self._theme.text_primary), (114, qy))
            qy += 30

    def _draw_algorithms_tab(self, surface: pygame.Surface) -> None:
        surface.blit(
            self._font_h2.render("A*, ROTA TAKİBİ VE DWA ARASINDAKİ FARKLAR", True, self._theme.accent),
            (92, 190),
        )
        cards = [
            (
                "1. A* Küresel Rota Planlayıcı (Global Planner)",
                self._theme.path_line,
                [
                    "• 20m x 15m haritayı 0.20 m çözünürlüklü 100x75 ızgara olarak modeller.",
                    "• Sabit engelleri robot yarıçapı (0.35 m) + güvenlik payı (0.15 m) = 0.50 m ile şişirir.",
                    "• Kare hücrelere en kısa Öklid mesafesi ve sürekli doğru parçası güvenliği uygular.",
                    "• Yalnızca statik haritayı kullanır; dinamik engeller statik ızgarayı değiştirmez.",
                ],
            ),
            (
                "2. Otonom Rota Takipçisi (Waypoint Follower — Mevcut Faz 3.2)",
                self._theme.nav_blue,
                [
                    "• A* tarafından üretilen ara hedeflere (waypoint) yönelim hatası hesaplayarak ilerler.",
                    "• Yönelim hatası büyükse yerinde döner; hizalandığında doğrusal hızı artırır.",
                    "• Dinamik engel güvenlik mesafesine (0.15 m) girdiğinde robotu güvenli şekilde durdurur.",
                    "• Engelin etrafından kendi başına yeniden rota çizmez (bu özellik Faz 4 DWA ile eklenecektir).",
                ],
            ),
            (
                "3. DWA — Dynamic Window Approach (Faz 4 Hazırlığı)",
                self._theme.dyn_obs_fill,
                [
                    "• Robotun kinematik hız penceresi (v, ω) içinde aday yörüngeler örnekleyecektir.",
                    "• Her aday yörüngeyi hedefe yakınlık, hız ve dinamik engel açıklığına göre puanlayacaktır.",
                    "• Şu an yalnızca arayüz ve telemetri alanı ayrılmıştır; sahte DWA verisi üretilmez.",
                ],
            ),
        ]
        y = 236
        for heading, color, bullets in cards:
            card = pygame.Rect(92, y, 1256, 178)
            pygame.draw.rect(surface, self._theme.panel_bg, card, border_radius=8)
            pygame.draw.rect(surface, color, card, width=1, border_radius=8)
            surface.blit(self._font_h2.render(heading, True, color), (114, y + 14))
            by = y + 48
            for b in bullets:
                surface.blit(self._font_body.render(b, True, self._theme.text_primary), (114, by))
                by += 28
            y += 198

    def _draw_safety_tab(self, surface: pygame.Surface) -> None:
        surface.blit(
            self._font_h2.render("GÜVENLİK GÖSTERGELERİ VE MESAFE TANIMLARI", True, self._theme.accent),
            (92, 190),
        )
        levels = [
            ("YEŞİL — GÜVENLİ", "Yüzey Açıklığı > 0.30 m", "Robot tüm engellerden güvenli mesafededir.", self._theme.safety_safe),
            ("SARI — DİKKAT", "0.15 m < Yüzey Açıklığı <= 0.30 m", "Engele yaklaşılıyor; erken uyarı durumu.", self._theme.safety_caution),
            ("TURUNCU — KRİTİK", "0.00 m < Yüzey Açıklığı <= 0.15 m", "Gerekli güvenlik payı (0.15 m) ihlal edildi; Ramak Kala olayı.", self._theme.safety_critical),
            ("KIRMIZI — ÇARPIŞMA", "Yüzey Açıklığı <= 0.00 m", "Fiziksel geometrik örtüşme (gerçek çarpışma).", self._theme.safety_collision),
        ]
        y = 236
        for name, cond, desc, col in levels:
            row = pygame.Rect(92, y, 1256, 72)
            pygame.draw.rect(surface, self._theme.panel_bg, row, border_radius=8)
            pygame.draw.rect(surface, col, row, width=1, border_radius=8)
            pygame.draw.circle(surface, col, (120, y + 36), 10)
            surface.blit(self._font_h2.render(name, True, col), (146, y + 12))
            surface.blit(self._font_body.render(cond, True, self._theme.text_primary), (146, y + 40))
            surface.blit(self._font_body.render(desc, True, self._theme.text_secondary), (520, y + 26))
            y += 88

        note_box = pygame.Rect(92, 608, 1256, 200)
        pygame.draw.rect(surface, self._theme.panel_bg, note_box, border_radius=8)
        pygame.draw.rect(surface, self._theme.border, note_box, width=1, border_radius=8)
        surface.blit(
            self._font_h2.render("FİZİKSEL HESAPLAMA İLKELERİ", True, self._theme.nav_blue),
            (114, 624),
        )
        notes = [
            "• Gerçek Açıklık (Surface-to-Surface): Sabit dikdörtgen engellerde en kısa kenar/köşe mesafesinden robot yarıçapı (0.35 m) çıkarılır.",
            "• Hareketli dairesel engellerde merkezler arası mesafeden (robot_yarıçapı + engel_yarıçapı) çıkarılır.",
            "• Güvenlik payı ihlali (0 < açıklık <= 0.15 m) asla fiziksel çarpışma olarak sayılmaz; Ramak Kala ve Uyarı ayrı izlenir.",
            "• Tahmini Çarpışma Süresi (TTC): Yalnızca yaklaşma hızı > 0 ve açıklık > 0 olduğunda hesaplanır; aksi halde 'Yok' gösterilir.",
        ]
        ny = 660
        for n in notes:
            surface.blit(self._font_body.render(n, True, self._theme.text_primary), (114, ny))
            ny += 32
