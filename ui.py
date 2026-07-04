"""
ui.py - Interface graphique CustomTkinter du logiciel FAO.

Responsabilite unique : afficher les controles, collecter les saisies
utilisateur et deleguer tout calcul aux modules specialises.
Aucune logique d'usinage ici.

Panneau gauche (scrollable) :
  - Boutons Importer / Generer
  - Magasin d'outils dynamique : T# / D / F / S / L / [X] + bouton "+"
  - Affectation Contour/Poche/Percage -> T#
  - Machine : type controleur / limites table X,Y / materiau brut
  - Legende couleurs + statut

Zone droite :
  - Haut (3/4) : visualiseur Matplotlib 2D
  - Bas  (1/4) : console texte (messages erreur en rouge, avertissements en orange)

Persistance :
  - Sauvegarde automatique dans config.json a chaque generation ou modification
    du magasin d'outils.
  - Rechargement automatique au lancement si config.json existe.
"""

from __future__ import annotations

import json
import math
import pathlib

import customtkinter as ctk
from tkinter import filedialog
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 -- enregistre la projection '3d'
from mpl_toolkits.mplot3d.art3d import Line3DCollection

import dxf_parser
import gcode_generator
import step_parser
from gcode_generator import SecritePhysiqueError
from models import (
    DxfEntity, MachiningConfig, ToolMagazine, ToolParams,
    MACHINE_TYPES, MATERIAL_OPTIONS,
)

matplotlib.use("TkAgg")

# ---------------------------------------------------------------------------
# Theme global
# ---------------------------------------------------------------------------
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ---------------------------------------------------------------------------
# Palette graphique
# ---------------------------------------------------------------------------
_PLOT_BG        = "#1a1a1a"
_PLOT_FIG_BG    = "#1e1e1e"
_PLOT_GRID      = "#2e2e2e"
_PLOT_AXIS_TEXT = "#666666"
_PLOT_GEOMETRY  = "#00c8ff"   # cyan   - contournage
_PLOT_PERCAGE   = "#ff6b35"   # orange - percage
_PLOT_POCHE     = "#7dff6b"   # vert   - poche
_PLOT_RAPIDE    = "#888888"   # gris   - deplacement rapide (G00 / hors matiere)
_PLOT_STEP_MESH = "#3a8fd0"   # bleu   - maillage filaire STEP

# Couleurs console
_CON_ERROR   = "#ff4444"
_CON_WARNING = "#ffaa33"
_CON_SUCCESS = "#7dff6b"
_CON_INFO    = "#cccccc"

# Choix machine UI -> valeur interne MachiningConfig
_MACHINE_OPTIONS: dict[str, str] = {
    "ISO Standard (Fanuc/Heidenhain)": "ISO_Standard",
    "GRBL (Shapeoko / Genmitsu)":      "GRBL_CNC",
    "Haas / Fanuc":                    "Haas_Fanuc",
}

# Configuration des axes (flux STEP) -> mode interne gcode_generator
_AXES_OPTIONS: dict[str, str] = {
    "3 axes (Multi-Gcode par face)": "3_axes",
    "4 axes (Continu / Indexe)":     "4_axes",
}

# Valeurs par defaut d'un nouvel outil
_TOOL_DEFAULT_DIAM:   float = 6.0
_TOOL_DEFAULT_LENGTH: float = 30.0
_TOOL_DEFAULT_FEED:   float = 1200.0
_TOOL_DEFAULT_SPIN:   int   = 18000

# Diametre minimum vraisemblable pour une fraise/un foret (mm). En dessous,
# une saisie ou un config.json corrompu (ex: 0.01mm au lieu de 6.0mm) est
# traite comme invalide plutot que d'etre silencieusement accepte : un rayon
# quasi-nul bloque calculer_offset_contour / calculer_trajectoires_poche
# (geometrie vide ou ignoree) sans message d'erreur clair pour l'utilisateur.
_TOOL_MIN_DIAM: float = 0.5

# Chemin du fichier de persistance (a cote du script)
_CONFIG_PATH = pathlib.Path(__file__).parent / "config.json"


# ===========================================================================
# LIGNE D'OUTIL (widget autonome)
# ===========================================================================

class _ToolRow:
    """Une ligne outil dans le magasin : T# / D / F / S / L / [X].

    Colonnes :
        col 0 : label T# (largeur fixe)
        col 1 : Diametre (mm)
        col 2 : Avance XY (mm/min)
        col 3 : Broche (tr/min)
        col 4 : Longueur utile (mm)
        col 5 : Bouton Supprimer

    Attributs publics :
        tool_number  -- numero T fixe a la creation (int)
        entry_diam   -- CTkEntry diametre mm
        entry_feed   -- CTkEntry avance XY mm/min
        entry_spin   -- CTkEntry vitesse broche tr/min
        entry_length -- CTkEntry longueur utile mm
    """

    def __init__(
        self,
        parent:      ctk.CTkFrame,
        row_index:   int,
        tool_number: int,
        on_delete,
        diam:   float = _TOOL_DEFAULT_DIAM,
        feed:   float = _TOOL_DEFAULT_FEED,
        spin:   int   = _TOOL_DEFAULT_SPIN,
        length: float = _TOOL_DEFAULT_LENGTH,
    ) -> None:
        self.tool_number = tool_number
        self._on_delete  = on_delete

        font_en = ctk.CTkFont(size=11)
        EW = 42   # largeur uniforme des champs de saisie

        # T# label
        self._lbl_num = ctk.CTkLabel(
            parent,
            text=f"T{tool_number}",
            font=ctk.CTkFont(size=11, weight="bold"),
            text_color="#5ba4cf",
            width=26,
            anchor="w",
        )
        self._lbl_num.grid(row=row_index, column=0, padx=(4, 1), pady=2, sticky="w")

        # Diametre
        self.entry_diam = ctk.CTkEntry(parent, width=EW, font=font_en, justify="center")
        self.entry_diam.insert(0, str(diam))
        self.entry_diam.grid(row=row_index, column=1, padx=1, pady=2)

        # Avance XY
        self.entry_feed = ctk.CTkEntry(parent, width=EW, font=font_en, justify="center")
        self.entry_feed.insert(0, str(int(feed)))
        self.entry_feed.grid(row=row_index, column=2, padx=1, pady=2)

        # Broche
        self.entry_spin = ctk.CTkEntry(parent, width=EW, font=font_en, justify="center")
        self.entry_spin.insert(0, str(int(spin)))
        self.entry_spin.grid(row=row_index, column=3, padx=1, pady=2)

        # Longueur utile
        self.entry_length = ctk.CTkEntry(parent, width=EW, font=font_en, justify="center")
        self.entry_length.insert(0, str(int(length)))
        self.entry_length.grid(row=row_index, column=4, padx=1, pady=2)

        # Bouton supprimer
        self._btn_del = ctk.CTkButton(
            parent,
            text="X",
            font=ctk.CTkFont(size=10, weight="bold"),
            width=22, height=24, corner_radius=4,
            fg_color="#5a1a1a", hover_color="#8b2020",
            command=lambda: self._on_delete(self),
        )
        self._btn_del.grid(row=row_index, column=5, padx=(1, 3), pady=2)

    # --- Lecture des champs ---

    @staticmethod
    def _lire(entry: ctk.CTkEntry, defaut: float) -> float:
        try:
            v = float(entry.get().replace(",", "."))
            return v if v > 0 else defaut
        except ValueError:
            return defaut

    def get_tool(self) -> ToolParams:
        """Lit LES VALEURS ACTUELLES des champs de saisie (entry.get() au
        moment de l'appel -- jamais une valeur mise en cache ni une valeur
        du config.json, qui ne sert qu'a pre-remplir ces champs au demarrage)
        et retourne un ToolParams valide.

        Le diametre est borne a _TOOL_MIN_DIAM ici meme : quel que soit
        l'appelant, il est impossible d'obtenir un outil au diametre
        microscopique (ex: 0.01mm) qui bloquerait les algorithmes
        geometriques (offset nul, poche en micro-passes).
        """
        diam   = self._lire(self.entry_diam,   _TOOL_DEFAULT_DIAM)
        feed   = self._lire(self.entry_feed,   _TOOL_DEFAULT_FEED)
        spin   = int(self._lire(self.entry_spin, float(_TOOL_DEFAULT_SPIN)))
        length = self._lire(self.entry_length, _TOOL_DEFAULT_LENGTH)
        plunge = max(feed * 0.25, 100.0)   # avance Z = 25% avance XY

        if diam < _TOOL_MIN_DIAM:
            print(
                f"[ui] T{self.tool_number} : diametre {diam:g}mm invraisemblable "
                f"(< {_TOOL_MIN_DIAM}mm), remplace par {_TOOL_DEFAULT_DIAM}mm"
            )
            diam = _TOOL_DEFAULT_DIAM

        return ToolParams(
            tool_number     = self.tool_number,
            tool_diameter   = diam,
            spindle_speed   = spin,
            feedrate        = feed,
            plunge_feedrate = plunge,
            tool_length     = length,
        )

    # --- Positionnement dans la grille ---

    def regrid(self, row_index: int) -> None:
        """Repositionne tous les widgets de la ligne a row_index."""
        self._lbl_num.grid(   row=row_index, column=0, padx=(4, 1), pady=2, sticky="w")
        self.entry_diam.grid( row=row_index, column=1, padx=1, pady=2)
        self.entry_feed.grid( row=row_index, column=2, padx=1, pady=2)
        self.entry_spin.grid( row=row_index, column=3, padx=1, pady=2)
        self.entry_length.grid(row=row_index, column=4, padx=1, pady=2)
        self._btn_del.grid(   row=row_index, column=5, padx=(1, 3), pady=2)

    def destroy(self) -> None:
        """Supprime tous les widgets de la ligne."""
        for w in (
            self._lbl_num, self.entry_diam, self.entry_feed,
            self.entry_spin, self.entry_length, self._btn_del,
        ):
            w.destroy()


# ===========================================================================
# FENETRE PRINCIPALE
# ===========================================================================

class FaoMainWindow(ctk.CTk):
    """Fenetre principale de l'application FAO."""

    WINDOW_WIDTH  = 1220
    WINDOW_HEIGHT = 750

    def __init__(self) -> None:
        super().__init__()
        self.title("MonLogicielFAO - Generateur G-code")
        self.geometry(f"{self.WINDOW_WIDTH}x{self.WINDOW_HEIGHT}")
        self.minsize(920, 560)

        self._dxf_filepath: str | None = None
        self._dxf_cercles:  list[dict]  = []
        self._dxf_poches:   list[tuple] = []

        self._step_filepath: str | None = None
        self._source_actif: str | None = None   # "dxf" | "step" -- source du dernier import
        self._step_maillage: dict | None = None  # maillage filaire cache (superposition
                                                  # piece/trajectoires apres generation)

        self._tool_rows:     list[_ToolRow] = []
        self._next_tool_num: int            = 1

        self._build_layout()

        # Rechargement config precedente (remplace les 3 outils par defaut si trouve)
        if not self._load_config():
            pass   # defaults deja poses par _build_layout

    # ------------------------------------------------------------------
    # Construction de l'interface
    # ------------------------------------------------------------------

    def _build_layout(self) -> None:
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self._build_sidebar()
        self._build_content_area()

    def _build_sidebar(self) -> None:
        """Panneau gauche entierement scrollable."""
        outer = ctk.CTkFrame(self, width=300, corner_radius=0)
        outer.grid(row=0, column=0, sticky="nsew")
        outer.grid_propagate(False)
        outer.grid_rowconfigure(0, weight=1)
        outer.grid_columnconfigure(0, weight=1)

        scroll = ctk.CTkScrollableFrame(outer, fg_color="transparent", width=290)
        scroll.grid(row=0, column=0, sticky="nsew")
        scroll.grid_columnconfigure(0, weight=1)

        def sep():
            ctk.CTkFrame(scroll, height=1, fg_color="gray30").grid(
                padx=12, pady=4, sticky="ew"
            )

        def section_label(text: str):
            ctk.CTkLabel(
                scroll,
                text=text,
                font=ctk.CTkFont(size=10, weight="bold"),
                text_color="gray50",
                anchor="w",
            ).grid(padx=16, pady=(8, 2), sticky="w")

        # --- Titre ---
        ctk.CTkLabel(
            scroll,
            text="MonLogiciel\nFAO",
            font=ctk.CTkFont(family="Roboto", size=22, weight="bold"),
        ).grid(padx=20, pady=(24, 2))

        ctk.CTkLabel(
            scroll,
            text="Generateur G-code ISO",
            font=ctk.CTkFont(size=11),
            text_color="gray60",
        ).grid(padx=20, pady=(0, 12))

        sep()

        # --- Boutons ---
        self._btn_import = ctk.CTkButton(
            scroll,
            text="Importer DXF",
            font=ctk.CTkFont(size=13, weight="bold"),
            height=40, corner_radius=8,
            command=self._on_import_dxf,
        )
        self._btn_import.grid(padx=16, pady=(10, 5), sticky="ew")

        self._btn_import_step = ctk.CTkButton(
            scroll,
            text="Importer STEP",
            font=ctk.CTkFont(size=13, weight="bold"),
            height=40, corner_radius=8,
            fg_color="#7a4a2a", hover_color="#5c3620",
            command=self._on_import_step,
        )
        self._btn_import_step.grid(padx=16, pady=(0, 5), sticky="ew")

        self._btn_generate = ctk.CTkButton(
            scroll,
            text="Generer G-code",
            font=ctk.CTkFont(size=13, weight="bold"),
            height=40, corner_radius=8,
            fg_color="#2a7a2a", hover_color="#1f5c1f",
            command=self._on_generate_gcode,
        )
        self._btn_generate.grid(padx=16, pady=(0, 10), sticky="ew")

        sep()

        # ----------------------------------------------------------------
        # MAGASIN D'OUTILS
        # ----------------------------------------------------------------
        section_label("MAGASIN D'OUTILS")

        # En-tete colonnes
        hdr = ctk.CTkFrame(scroll, fg_color="transparent")
        hdr.grid(padx=4, sticky="ew")
        for ci, (txt, w) in enumerate([
            ("#",   26), ("D mm", 42), ("F mm", 42), ("S rpm", 42), ("L mm", 42), ("", 22),
        ]):
            ctk.CTkLabel(
                hdr, text=txt, width=w,
                font=ctk.CTkFont(size=9), text_color="gray40",
                anchor="center",
            ).grid(row=0, column=ci, padx=1)

        # Grille des lignes
        self._tool_frame = ctk.CTkFrame(scroll, fg_color="transparent")
        self._tool_frame.grid(padx=4, sticky="ew")
        for ci, w in enumerate([26, 42, 42, 42, 42, 22]):
            self._tool_frame.grid_columnconfigure(ci, minsize=w)

        # Outils par defaut
        self._add_tool(diam=6.0, feed=1200.0, spin=18000, length=30.0)   # T1
        self._add_tool(diam=4.0, feed=800.0,  spin=20000, length=25.0)   # T2
        self._add_tool(diam=3.0, feed=600.0,  spin=24000, length=22.0)   # T3

        self._btn_add_tool = ctk.CTkButton(
            scroll,
            text="+ Ajouter un outil",
            font=ctk.CTkFont(size=11),
            height=28, corner_radius=6,
            fg_color="transparent",
            border_width=1, border_color="gray40",
            hover_color="#2a2a2a",
            command=lambda: self._add_tool(),
        )
        self._btn_add_tool.grid(padx=16, pady=(4, 8), sticky="ew")

        sep()

        # ----------------------------------------------------------------
        # AFFECTATION DES OUTILS
        # ----------------------------------------------------------------
        section_label("AFFECTATION DES OUTILS")

        for attr, label, default in [
            ("_om_contour", "Contour  -> T#", "1"),
            ("_om_poche",   "Poche    -> T#", "2"),
            ("_om_percage", "Percage  -> T#", "3"),
        ]:
            ctk.CTkLabel(
                scroll, text=label,
                font=ctk.CTkFont(size=11), text_color="gray70", anchor="w",
            ).grid(padx=16, pady=(4, 0), sticky="w")
            om = ctk.CTkOptionMenu(
                scroll, values=["1"],
                font=ctk.CTkFont(size=11), width=110,
            )
            om.set(default)
            om.grid(padx=16, pady=(2, 2), sticky="w")
            setattr(self, attr, om)

        self._refresh_tool_options()

        sep()

        # ----------------------------------------------------------------
        # MACHINE
        # ----------------------------------------------------------------
        section_label("MACHINE")

        # Configuration des axes (flux STEP : multi-fichiers ou 4 axes)
        ctk.CTkLabel(
            scroll, text="Configuration Axes",
            font=ctk.CTkFont(size=11), text_color="gray70", anchor="w",
        ).grid(padx=16, sticky="w")

        self._om_axes = ctk.CTkOptionMenu(
            scroll,
            values=list(_AXES_OPTIONS.keys()),
            font=ctk.CTkFont(size=11),
            dynamic_resizing=False, width=260,
        )
        self._om_axes.set(list(_AXES_OPTIONS.keys())[0])
        self._om_axes.grid(padx=16, pady=(2, 8), sticky="ew")

        # Type de controleur
        ctk.CTkLabel(
            scroll, text="Type de controleur",
            font=ctk.CTkFont(size=11), text_color="gray70", anchor="w",
        ).grid(padx=16, sticky="w")

        self._optmenu_machine = ctk.CTkOptionMenu(
            scroll,
            values=list(_MACHINE_OPTIONS.keys()),
            font=ctk.CTkFont(size=11),
            dynamic_resizing=False, width=260,
        )
        self._optmenu_machine.set(list(_MACHINE_OPTIONS.keys())[0])
        self._optmenu_machine.grid(padx=16, pady=(2, 8), sticky="ew")

        # Limites table
        ctk.CTkLabel(
            scroll, text="Dimensions table (mm)",
            font=ctk.CTkFont(size=11), text_color="gray70", anchor="w",
        ).grid(padx=16, pady=(4, 0), sticky="w")

        frm_limits = ctk.CTkFrame(scroll, fg_color="transparent")
        frm_limits.grid(padx=16, pady=(2, 6), sticky="w")

        ctk.CTkLabel(frm_limits, text="X max:", font=ctk.CTkFont(size=11),
                     text_color="gray60").grid(row=0, column=0, padx=(0, 4))
        self._entry_limit_x = ctk.CTkEntry(frm_limits, width=70,
                                            font=ctk.CTkFont(size=11), justify="center")
        self._entry_limit_x.insert(0, "300")
        self._entry_limit_x.grid(row=0, column=1, padx=(0, 12))

        ctk.CTkLabel(frm_limits, text="Y max:", font=ctk.CTkFont(size=11),
                     text_color="gray60").grid(row=0, column=2, padx=(0, 4))
        self._entry_limit_y = ctk.CTkEntry(frm_limits, width=70,
                                            font=ctk.CTkFont(size=11), justify="center")
        self._entry_limit_y.insert(0, "200")
        self._entry_limit_y.grid(row=0, column=3)

        # Materiau brut
        ctk.CTkLabel(
            scroll, text="Materiau brut",
            font=ctk.CTkFont(size=11), text_color="gray70", anchor="w",
        ).grid(padx=16, pady=(4, 0), sticky="w")

        self._om_material = ctk.CTkOptionMenu(
            scroll, values=MATERIAL_OPTIONS,
            font=ctk.CTkFont(size=11), width=160,
        )
        self._om_material.set("Aluminium")
        self._om_material.grid(padx=16, pady=(2, 10), sticky="w")

        sep()

        # ----------------------------------------------------------------
        # LEGENDE
        # ----------------------------------------------------------------
        section_label("LEGENDE")

        for label, couleur in [
            ("Contournage",  _PLOT_GEOMETRY),
            ("Poche",        _PLOT_POCHE),
            ("Percage",      _PLOT_PERCAGE),
            ("Rapide (G00)", _PLOT_RAPIDE),
        ]:
            row_f = ctk.CTkFrame(scroll, fg_color="transparent")
            row_f.grid(padx=16, pady=1, sticky="w")
            ctk.CTkFrame(row_f, width=12, height=12,
                         fg_color=couleur, corner_radius=2).pack(side="left")
            ctk.CTkLabel(row_f, text=f"  {label}",
                         font=ctk.CTkFont(size=11), text_color="gray60").pack(side="left")

        self._status_label = ctk.CTkLabel(
            scroll,
            text="Aucun fichier charge",
            font=ctk.CTkFont(size=10),
            text_color="gray50",
            wraplength=260,
        )
        self._status_label.grid(padx=16, pady=(12, 20))

    def _build_content_area(self) -> None:
        content = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        content.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        content.grid_columnconfigure(0, weight=1)
        content.grid_rowconfigure(0, weight=3)
        content.grid_rowconfigure(1, weight=1)
        self._build_plot_area(content)
        self._build_console_area(content)

    def _build_plot_area(self, parent: ctk.CTkFrame) -> None:
        plot_frame = ctk.CTkFrame(parent, corner_radius=8, fg_color=_PLOT_FIG_BG)
        plot_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        plot_frame.grid_columnconfigure(0, weight=1)
        plot_frame.grid_rowconfigure(0, weight=1)

        self._fig: Figure = plt.figure(facecolor=_PLOT_FIG_BG, tight_layout=True)
        self._ax:  Axes   = self._fig.add_subplot(111, projection="3d")
        self._appliquer_style_sombre(self._ax)
        self._ax.text2D(
            0.5, 0.5,
            "Importez un fichier DXF ou STEP\npour visualiser la geometrie",
            ha="center", va="center",
            color="#444444", fontsize=13,
            transform=self._ax.transAxes,
        )

        self._canvas = FigureCanvasTkAgg(self._fig, master=plot_frame)
        self._canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        self._canvas.draw()

    def _build_console_area(self, parent: ctk.CTkFrame) -> None:
        console_frame = ctk.CTkFrame(parent, corner_radius=8, fg_color="transparent")
        console_frame.grid(row=1, column=0, sticky="nsew")
        console_frame.grid_columnconfigure(0, weight=1)
        console_frame.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(
            console_frame, text="Console",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color="gray60", anchor="w",
        ).grid(row=0, column=0, sticky="ew", pady=(0, 4))

        self._console = ctk.CTkTextbox(
            console_frame,
            font=ctk.CTkFont(family="Courier New", size=11),
            corner_radius=8, wrap="none", state="disabled", height=120,
        )
        self._console.grid(row=1, column=0, sticky="nsew")

        # Couleurs des niveaux de log
        tb = self._console._textbox
        tb.tag_config("error",   foreground=_CON_ERROR)
        tb.tag_config("warning", foreground=_CON_WARNING)
        tb.tag_config("success", foreground=_CON_SUCCESS)
        tb.tag_config("info",    foreground=_CON_INFO)

        self._console_print("Bienvenue dans MonLogicielFAO.")
        self._console_print("Configurez le magasin d'outils, les limites de table")
        self._console_print("et le materiau, puis importez un DXF et generez le G-code.")

    # ------------------------------------------------------------------
    # Magasin dynamique
    # ------------------------------------------------------------------

    def _add_tool(
        self,
        tool_number: int | None = None,
        diam:   float = _TOOL_DEFAULT_DIAM,
        feed:   float = _TOOL_DEFAULT_FEED,
        spin:   int   = _TOOL_DEFAULT_SPIN,
        length: float = _TOOL_DEFAULT_LENGTH,
    ) -> None:
        """Ajoute une ligne outil. tool_number=None -> auto-increment."""
        num = tool_number if tool_number is not None else self._next_tool_num
        self._next_tool_num = max(self._next_tool_num, num + 1)

        row = _ToolRow(
            parent      = self._tool_frame,
            row_index   = len(self._tool_rows),
            tool_number = num,
            on_delete   = self._on_delete_tool,
            diam=diam, feed=feed, spin=spin, length=length,
        )
        self._tool_rows.append(row)
        self._refresh_tool_options()

    def _on_delete_tool(self, row: _ToolRow) -> None:
        if len(self._tool_rows) <= 1:
            self._console_print("[!] Impossible de supprimer le dernier outil.", "warning")
            return
        row.destroy()
        self._tool_rows.remove(row)
        for idx, r in enumerate(self._tool_rows):
            r.regrid(idx)
        self._refresh_tool_options()
        self._save_config()

    def _refresh_tool_options(self) -> None:
        """Met a jour les dropdowns d'affectation selon les T# disponibles."""
        if "_om_contour" not in self.__dict__:
            return   # widgets pas encore crees

        nums = [str(r.tool_number) for r in self._tool_rows] or ["1"]
        prev = {
            "_om_contour": self._om_contour.get(),
            "_om_poche":   self._om_poche.get(),
            "_om_percage": self._om_percage.get(),
        }
        for attr, fallback_idx in [
            ("_om_contour", 0),
            ("_om_poche",   min(1, len(nums) - 1)),
            ("_om_percage", len(nums) - 1),
        ]:
            om = getattr(self, attr)
            om.configure(values=nums)
            val = prev[attr]
            om.set(val if val in nums else nums[fallback_idx])

    def _set_affectation_manuelle(self, active: bool) -> None:
        """Active/desactive les menus d'affectation manuelle Contour/Poche/Percage.

        Le flux STEP affecte les outils automatiquement (selection par
        diametre depuis le magasin, dans gcode_generator) : les menus sont
        grises pour signifier qu'ils n'ont aucun effet dans ce mode.
        Le flux DXF les reactive (affectation manuelle par role).
        """
        etat = "normal" if active else "disabled"
        for attr in ("_om_contour", "_om_poche", "_om_percage"):
            om = getattr(self, attr, None)
            if om is not None:
                om.configure(state=etat)

    # ------------------------------------------------------------------
    # Persistance config.json
    # ------------------------------------------------------------------

    def _save_config(self) -> None:
        """Sauvegarde la configuration courante dans config.json."""
        data = {
            "version": 2,
            "tools": [
                {
                    "tool_number":   r.tool_number,
                    "tool_diameter": _ToolRow._lire(r.entry_diam,   _TOOL_DEFAULT_DIAM),
                    "tool_length":   _ToolRow._lire(r.entry_length, _TOOL_DEFAULT_LENGTH),
                    "feedrate":      _ToolRow._lire(r.entry_feed,   _TOOL_DEFAULT_FEED),
                    "spindle_speed": int(_ToolRow._lire(r.entry_spin, float(_TOOL_DEFAULT_SPIN))),
                }
                for r in self._tool_rows
            ],
            "assignments": {
                "contour": self._om_contour.get(),
                "poche":   self._om_poche.get(),
                "percage": self._om_percage.get(),
            },
            "machine": {
                "type":      _MACHINE_OPTIONS.get(self._optmenu_machine.get(), "ISO_Standard"),
                "axes_mode": _AXES_OPTIONS.get(self._om_axes.get(), "3_axes"),
                "limit_x":   _parse_float_entry(self._entry_limit_x, 300.0),
                "limit_y":   _parse_float_entry(self._entry_limit_y, 200.0),
                "material":  self._om_material.get(),
            },
        }
        try:
            with open(_CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except OSError as exc:
            self._console_print(f"[!] Sauvegarde config.json impossible : {exc}", "warning")

    def _load_config(self) -> bool:
        """Charge config.json si disponible. Retourne True si succes."""
        if not _CONFIG_PATH.exists():
            return False
        try:
            with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError):
            return False

        # --- Outils ---
        tools = data.get("tools", [])
        if tools:
            for r in list(self._tool_rows):
                r.destroy()
            self._tool_rows.clear()
            self._next_tool_num = 1
            for td in tools:
                diam_json = float(td.get("tool_diameter", _TOOL_DEFAULT_DIAM))
                if diam_json < _TOOL_MIN_DIAM:
                    self._console_print(
                        f"[!] config.json : T{td.get('tool_number')} avait un diametre "
                        f"{diam_json:g}mm invraisemblable -- remplace par "
                        f"{_TOOL_DEFAULT_DIAM}mm.",
                        "warning",
                    )
                    diam_json = _TOOL_DEFAULT_DIAM
                self._add_tool(
                    tool_number = td.get("tool_number"),
                    diam        = diam_json,
                    length      = float(td.get("tool_length",   _TOOL_DEFAULT_LENGTH)),
                    feed        = float(td.get("feedrate",       _TOOL_DEFAULT_FEED)),
                    spin        = int(td.get("spindle_speed",    _TOOL_DEFAULT_SPIN)),
                )

        # --- Affectations ---
        nums    = [str(r.tool_number) for r in self._tool_rows]
        assigns = data.get("assignments", {})
        for attr, key in [
            ("_om_contour", "contour"),
            ("_om_poche",   "poche"),
            ("_om_percage", "percage"),
        ]:
            val = str(assigns.get(key, ""))
            if val in nums:
                getattr(self, attr).set(val)

        # --- Machine ---
        machine = data.get("machine", {})

        machine_type  = machine.get("type", "ISO_Standard")
        machine_label = next(
            (k for k, v in _MACHINE_OPTIONS.items() if v == machine_type), None
        )
        if machine_label:
            self._optmenu_machine.set(machine_label)

        axes_mode  = machine.get("axes_mode", "3_axes")
        axes_label = next(
            (k for k, v in _AXES_OPTIONS.items() if v == axes_mode), None
        )
        if axes_label:
            self._om_axes.set(axes_label)

        def _set_entry(entry: ctk.CTkEntry, val: float) -> None:
            entry.delete(0, "end")
            entry.insert(0, str(val))

        _set_entry(self._entry_limit_x, machine.get("limit_x", 300.0))
        _set_entry(self._entry_limit_y, machine.get("limit_y", 200.0))

        mat = machine.get("material", "Aluminium")
        if mat in MATERIAL_OPTIONS:
            self._om_material.set(mat)

        self._console_print("Config precedente rechargee depuis config.json", "success")
        return True

    # ------------------------------------------------------------------
    # Construction de MachiningConfig
    # ------------------------------------------------------------------

    def _build_config(self) -> MachiningConfig:
        """Construit MachiningConfig a partir de l'etat COURANT des widgets,
        lu de force au moment de l'appel (donc au clic sur 'Generer G-code') :

          - Magasin : chaque row.get_tool() relit entry_diam.get(),
            entry_feed.get(), entry_spin.get(), entry_length.get() -- les
            valeurs affichees a l'ecran a cet instant precis.
          - Table : _entry_limit_x.get() / _entry_limit_y.get(), idem.

        config.json n'est JAMAIS lu ici : il ne sert qu'au demarrage pour
        pre-remplir ces memes widgets (cf. _load_config). Il ne peut donc
        pas ecraser une saisie en direct.

        Un recapitulatif de tout ce qui a ete lu est affiche dans la console
        a chaque appel : si ce recapitulatif n'apparait pas apres un clic sur
        'Generer G-code', c'est que l'application qui tourne n'execute PAS ce
        fichier (ancienne instance restee ouverte, .exe fige de dist/,
        ou copie du projet dans un autre dossier).
        """
        magazine = ToolMagazine()
        for row in self._tool_rows:
            try:
                outil = row.get_tool()
            except Exception as exc:
                self._console_print(f"[!] T{row.tool_number} invalide : {exc}", "warning")
                continue

            if outil.tool_diameter < _TOOL_MIN_DIAM:
                self._console_print(
                    f"[!] T{outil.tool_number} : diametre {outil.tool_diameter:g}mm "
                    f"invraisemblable (< {_TOOL_MIN_DIAM}mm) -- remplace par "
                    f"{_TOOL_DEFAULT_DIAM}mm par securite. Corrigez la case D "
                    f"dans le magasin d'outils.",
                    "warning",
                )
                outil.tool_diameter = _TOOL_DEFAULT_DIAM

            magazine.add(outil)

        def _tnum(om: ctk.CTkOptionMenu, rows: list[_ToolRow], idx: int) -> int:
            try:
                return int(om.get())
            except ValueError:
                return rows[min(idx, len(rows) - 1)].tool_number if rows else 1

        limit_x = _parse_float_entry(self._entry_limit_x, 300.0)
        limit_y = _parse_float_entry(self._entry_limit_y, 200.0)

        # Recapitulatif de ce qui vient d'etre lu A L'ECRAN (diagnostic) :
        outils_str = "  ".join(
            f"T{t.tool_number}=D{t.tool_diameter:g}"
            for t in magazine.tools.values()
        )
        self._console_print(
            f">> Config lue a l'ecran : {outils_str} | "
            f"Table {limit_x:g}x{limit_y:g}mm | {self._om_material.get()}",
            "info",
        )

        return MachiningConfig(
            safety_z             = 5.0,
            default_target_depth = -6.0,
            default_pass_depth   = 2.0,
            magazine             = magazine,
            tool_number_contour  = _tnum(self._om_contour, self._tool_rows, 0),
            tool_number_poche    = _tnum(self._om_poche,   self._tool_rows, 1),
            tool_number_drill    = _tnum(self._om_percage, self._tool_rows, -1),
            peck_depth           = 2.0,
            stepover_poche       = 0.5,
            machine_type         = _MACHINE_OPTIONS.get(
                self._optmenu_machine.get(), "ISO_Standard"
            ),
            cnc_limit_x  = limit_x,
            cnc_limit_y  = limit_y,
            material_type = self._om_material.get(),
        )

    # ------------------------------------------------------------------
    # Gestionnaires d'evenements
    # ------------------------------------------------------------------

    def _on_import_dxf(self) -> None:
        filepath = filedialog.askopenfilename(
            title="Selectionner un fichier DXF",
            filetypes=[("Fichiers DXF", "*.dxf"), ("Tous les fichiers", "*.*")],
        )
        if not filepath:
            self._console_print("Import annule.")
            return

        self._dxf_filepath = filepath
        self._source_actif = "dxf"
        self._step_maillage = None   # plus de filigrane STEP en mode DXF
        self._set_affectation_manuelle(True)
        filename = filepath.replace("\\", "/").split("/")[-1]
        self._status_label.configure(text=filename, text_color="#4CAF50")
        self._console_print(f"DXF : {filename}", "info")

        try:
            document = dxf_parser.load_dxf_document(filepath)
            entites  = dxf_parser.extract_all_entities(document)
        except Exception as exc:
            self._console_print(f"[ERREUR] Lecture DXF : {exc}", "error")
            return

        liste_lignes: list[tuple] = []
        self._dxf_poches  = []
        self._dxf_cercles = []

        for e in entites:
            if e.entity_type == "LINE":
                seg = (
                    e.raw_data["start"][0], e.raw_data["start"][1],
                    e.raw_data["end"][0],   e.raw_data["end"][1],
                )
                if gcode_generator._est_calque_poche(e.layer):
                    self._dxf_poches.append(seg)
                else:
                    liste_lignes.append(seg)
            elif e.entity_type == "CIRCLE":
                self._dxf_cercles.append(e.raw_data)

        # Calques et profondeurs detectees
        profondeurs: set[str] = set()
        for e in entites:
            z = gcode_generator._parse_z_from_layer(e.layer, -6.0)
            profondeurs.add(f"{e.layer} -> Z={z:.1f}mm")

        self._console_print(
            f"{len(entites)} entites | "
            f"{len(liste_lignes)} contour | "
            f"{len(self._dxf_poches)} poche | "
            f"{len(self._dxf_cercles)} percage",
            "info",
        )
        for info in sorted(profondeurs):
            self._console_print(f"  {info}", "info")

        self._update_plot(liste_lignes, self._dxf_cercles, self._dxf_poches)

    def _on_import_step(self) -> None:
        """Import STEP : bounding box du brut + features (poches/percages) via
        step_parser, puis affichage du maillage filaire 3D de la piece.
        """
        filepath = filedialog.askopenfilename(
            title="Selectionner un fichier STEP",
            filetypes=[("Fichiers STEP", "*.step *.stp"), ("Tous les fichiers", "*.*")],
        )
        if not filepath:
            self._console_print("Import annule.")
            return

        self._step_filepath = filepath
        self._source_actif = "step"
        self._set_affectation_manuelle(False)
        filename = filepath.replace("\\", "/").split("/")[-1]
        self._status_label.configure(text=filename, text_color="#4CAF50")
        self._console_print(f"STEP : {filename}", "info")
        self._console_print(
            "Affectation des outils : AUTOMATIQUE (selection par diametre "
            "depuis le magasin -- les menus Contour/Poche/Percage sont ignores)",
            "info",
        )

        try:
            dimensions = step_parser.extraire_dimensions_brut(filepath)
            features   = step_parser.analyser_features_3d(filepath)
        except Exception as exc:
            self._console_print(f"[ERREUR] Lecture STEP : {exc}", "error")
            return

        self._console_print(
            f"Brut : {dimensions['x']:.1f} x {dimensions['y']:.1f} x "
            f"{dimensions['z']:.1f} mm",
            "info",
        )
        self._console_print(
            f"{len(features['faces_planes'])} face(s) plane(s) | "
            f"{len(features['percages'])} percage(s) detecte(s)",
            "info",
        )
        for face in features["faces_planes"]:
            self._console_print(
                f"  Poche Z={face['z']:.2f}mm  aire={face['aire']:.1f}mm2", "info"
            )
        for trou in features["percages"]:
            self._console_print(
                f"  Percage x={trou['x']:.1f} y={trou['y']:.1f} "
                f"r={trou['rayon']:.1f} prof={trou['profondeur']:.2f}mm",
                "info",
            )

        self._update_plot_step(filepath)

    def _on_generate_gcode(self) -> None:
        """Distribue vers le flux DXF ou STEP selon la derniere source importee."""
        if self._source_actif == "step":
            self._on_generate_gcode_step()
        elif self._source_actif == "dxf":
            self._on_generate_gcode_dxf()
        else:
            self._console_print("[!] Aucun fichier DXF ou STEP charge.", "warning")

    def _on_generate_gcode_dxf(self) -> None:
        """Lecture DXF -> verifications securite -> generation -> sauvegarde."""
        if self._dxf_filepath is None:
            self._console_print("[!] Aucun fichier DXF charge.", "warning")
            return

        config = self._build_config()

        self._console_print("-" * 52, "info")
        self._console_print("[1/4] Lecture DXF...", "info")
        try:
            document       = dxf_parser.load_dxf_document(self._dxf_filepath)
            toutes_entites = dxf_parser.extract_all_entities(document)
        except Exception as exc:
            self._console_print(f"[ERREUR] {exc}", "error")
            return

        # Comptage
        nb_cont  = sum(
            1 for e in toutes_entites
            if e.entity_type == "LINE" and not gcode_generator._est_calque_poche(e.layer)
        )
        nb_poche = sum(
            1 for e in toutes_entites
            if e.entity_type == "LINE" and gcode_generator._est_calque_poche(e.layer)
        )
        nb_drill = sum(1 for e in toutes_entites if e.entity_type == "CIRCLE")
        self._console_print(
            f"[2/4] {nb_cont} contour | {nb_poche} poche | {nb_drill} percage",
            "info",
        )
        if not toutes_entites:
            self._console_print("[!] Aucune geometrie. Annule.", "warning")
            return

        # Info outil et machine
        post_label = gcode_generator.get_post_processeur(config.machine_type).label
        mat_info   = f"{config.material_type} | Table {config.cnc_limit_x:.0f}x{config.cnc_limit_y:.0f}mm"
        outils = []
        for role, tnum in [
            ("contour", config.tool_number_contour),
            ("poche",   config.tool_number_poche),
            ("percage", config.tool_number_drill),
        ]:
            t = config.magazine.get(tnum)
            if t:
                outils.append(f"T{tnum} D{t.tool_diameter:.0f}mm L{t.tool_length:.0f}mm ({role})")
        self._console_print(f"[3/4] {post_label} | {mat_info}", "info")
        for o in outils:
            self._console_print(f"      {o}", "info")

        # Generation (inclut les securites physiques)
        try:
            programme_gcode: str = gcode_generator.generer_programme_complet(
                entites_dxf=toutes_entites,
                config=config,
            )
        except SecritePhysiqueError as exc:
            # Securite physique violee : affichage en rouge, blocage total
            self._console_print("", "error")
            self._console_print("*** GENERATION BLOQUEE - SECURITE ***", "error")
            for ligne in str(exc).splitlines():
                self._console_print(f"  {ligne}", "error")
            self._console_print("", "error")
            return
        except (ValueError, RuntimeError, KeyError) as exc:
            self._console_print(f"[ERREUR] Generation : {exc}", "error")
            return

        # Sauvegarde fichier
        chemin: str = filedialog.asksaveasfilename(
            title="Enregistrer le G-code",
            defaultextension=".nc",
            filetypes=[
                ("Fichiers G-code",   "*.nc"),
                ("Fichiers texte",    "*.txt"),
                ("Tous les fichiers", "*.*"),
            ],
        )
        if not chemin:
            self._console_print("[!] Enregistrement annule.", "warning")
            return

        try:
            with open(chemin, "w", encoding="utf-8") as f:
                f.write(programme_gcode)
        except OSError as exc:
            self._console_print(f"[ERREUR] Ecriture : {exc}", "error")
            return

        nom = chemin.replace("\\", "/").split("/")[-1]
        nb_lignes = len(programme_gcode.splitlines())
        self._console_print(
            f"[4/4] {nom} -- {nb_lignes} lignes G-code", "success"
        )
        self._console_print("-" * 52, "info")

        # Mise a jour du visualiseur avec les trajectoires reelles (G01 uniquement)
        self._update_plot_from_gcode(programme_gcode)

        # Persistance config
        self._save_config()

    def _on_generate_gcode_step(self) -> None:
        """Lecture STEP -> analyse multi-faces -> generation selon le mode Axes.

        Mode "3 axes (Multi-Gcode par face)" : un fichier G-code distinct par
        face du modele necessitant de l'usinage (suffixe _Face_Z_PLUS,
        _Face_X_PLUS, ...), chacun avec son instruction de retournement.
        Mode "4 axes (Continu / Indexe)"     : un fichier unique, la piece
        pivotee entre les faces par l'axe rotatif A (G00 A90/A180/A270).
        """
        if self._step_filepath is None:
            self._console_print("[!] Aucun fichier STEP charge.", "warning")
            return

        config    = self._build_config()
        mode_axes = _AXES_OPTIONS.get(self._om_axes.get(), "3_axes")

        self._console_print("-" * 52, "info")
        self._console_print(
            f"[1/4] Lecture STEP + analyse multi-faces ({self._om_axes.get()})...",
            "info",
        )
        try:
            dimensions     = step_parser.extraire_dimensions_brut(self._step_filepath)
            features_multi = step_parser.analyser_features_multi_faces(self._step_filepath)
        except Exception as exc:
            self._console_print(f"[ERREUR] {exc}", "error")
            return

        config.definir_brut_depuis_step(dimensions)

        self._console_print(
            f"[2/4] Brut {dimensions['x']:.1f}x{dimensions['y']:.1f}x{dimensions['z']:.1f}mm | "
            f"{len(features_multi)} orientation(s) a usiner",
            "info",
        )
        for nom, data in features_multi.items():
            self._console_print(
                f"      {nom}: {len(data['faces_planes'])} face(s), "
                f"{len(data['percages'])} percage(s)",
                "info",
            )

        post_label = gcode_generator.get_post_processeur(config.machine_type).label
        mat_info   = f"{config.material_type} | Table {config.cnc_limit_x:.0f}x{config.cnc_limit_y:.0f}mm"
        self._console_print(f"[3/4] {post_label} | {mat_info}", "info")

        try:
            gcodes, erreurs = gcode_generator.generer_gcodes_multi_faces(
                features_multi, config, mode=mode_axes,
            )
        except SecritePhysiqueError as exc:
            self._console_print("", "error")
            self._console_print("*** GENERATION BLOQUEE - SECURITE ***", "error")
            for ligne in str(exc).splitlines():
                self._console_print(f"  {ligne}", "error")
            self._console_print("", "error")
            return
        except (ValueError, RuntimeError, KeyError) as exc:
            self._console_print(f"[ERREUR] Generation : {exc}", "error")
            return

        for nom, msg in erreurs.items():
            self._console_print(f"[!] Face {nom} NON generee (securite) :", "warning")
            self._console_print(f"    {msg.splitlines()[0]}", "warning")

        if not gcodes:
            self._console_print("[!] Aucune face generable. Annule.", "warning")
            return

        chemin: str = filedialog.asksaveasfilename(
            title="Enregistrer le G-code (nom de base pour le multi-faces)",
            defaultextension=".nc",
            filetypes=[
                ("Fichiers G-code",   "*.nc"),
                ("Fichiers texte",    "*.txt"),
                ("Tous les fichiers", "*.*"),
            ],
        )
        if not chemin:
            self._console_print("[!] Enregistrement annule.", "warning")
            return

        base = pathlib.Path(chemin)
        ecrits: list[str] = []
        try:
            for nom, programme in gcodes.items():
                if mode_axes == "4_axes":
                    cible = base   # fichier unique : le nom choisi tel quel
                else:
                    cible = base.with_name(
                        f"{base.stem}_Face_{nom}{base.suffix or '.nc'}"
                    )
                with open(cible, "w", encoding="utf-8") as f:
                    f.write(programme)
                ecrits.append(cible.name)
        except OSError as exc:
            self._console_print(f"[ERREUR] Ecriture : {exc}", "error")
            return

        total_lignes = sum(len(p.splitlines()) for p in gcodes.values())
        self._console_print(
            f"[4/4] {len(ecrits)} fichier(s) -- {total_lignes} lignes G-code au total",
            "success",
        )
        for nom_fichier in ecrits:
            self._console_print(f"      {nom_fichier}", "success")
        self._console_print("-" * 52, "info")

        # Visualiseur : trajectoires de la face principale (Z_PLUS) en 3 axes,
        # ou du programme unifie en 4 axes.
        programme_affiche = gcodes.get("Z_PLUS") or next(iter(gcodes.values()))
        self._update_plot_from_gcode(programme_affiche)

        self._save_config()

    # ------------------------------------------------------------------
    # Visualisation graphique
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_gcode_paths(
        gcode: str,
    ) -> tuple[list[tuple], list[tuple], list[tuple], list[tuple]]:
        """Parse le G-code et extrait les trajectoires 3D de l'outil.

        Returns:
            cuts   : (x1,y1,z1,x2,y2,z2) -- G01 a plat, Z <= 0 (coupe en matiere)
            ramps  : (x1,y1,z1,x2,y2,z2) -- G01 avec Z changeant (rampe/plongee)
            rapids : (x1,y1,z1,x2,y2,z2) -- G00, ou G01 hors matiere (Z > 0) :
                     deplacements a vide, "en l'air" au-dessus de la piece
            drills : (x,y,z) -- position et profondeur finale de chaque G83/G81
        """
        import re

        import math as _math

        cuts:   list[tuple] = []
        ramps:  list[tuple] = []
        rapids: list[tuple] = []
        drills: list[tuple] = []

        cx, cy, cz = 0.0, 0.0, 0.0

        re_x = re.compile(r'X(-?\d+\.?\d*)', re.I)
        re_y = re.compile(r'Y(-?\d+\.?\d*)', re.I)
        re_z = re.compile(r'Z(-?\d+\.?\d*)', re.I)
        re_i = re.compile(r'I(-?\d+\.?\d*)', re.I)
        re_j = re.compile(r'J(-?\d+\.?\d*)', re.I)

        for raw in gcode.splitlines():
            line = raw.strip()
            if not line or line.startswith(';') or line.startswith('(') \
                    or line.startswith('%'):
                continue

            mx = re_x.search(line)
            my = re_y.search(line)
            mz = re_z.search(line)

            nx = float(mx.group(1)) if mx else cx
            ny = float(my.group(1)) if my else cy
            nz = float(mz.group(1)) if mz else cz

            if 'G00' in line:
                if (nx, ny, nz) != (cx, cy, cz):
                    rapids.append((cx, cy, cz, nx, ny, nz))
                cx, cy, cz = nx, ny, nz

            elif 'G01' in line:
                if nz <= 0.001:                    # outil dans la matiere a l'arrivee
                    if abs(nz - cz) > 0.001:       # Z change = rampe
                        ramps.append((cx, cy, cz, nx, ny, nz))
                    else:                           # Z constant = coupe a plat
                        cuts.append((cx, cy, cz, nx, ny, nz))
                else:                               # deplacement lineaire hors matiere
                    rapids.append((cx, cy, cz, nx, ny, nz))
                cx, cy, cz = nx, ny, nz

            elif 'G02' in line or 'G03' in line:
                # Arc (ou helice si Z change) echantillonne en 24 segments
                # pour l'affichage. I/J = decalage du centre depuis le depart.
                mi = re_i.search(line)
                mj = re_j.search(line)
                if mi is None and mj is None:
                    cx, cy, cz = nx, ny, nz
                    continue
                i_off = float(mi.group(1)) if mi else 0.0
                j_off = float(mj.group(1)) if mj else 0.0
                centre_x, centre_y = cx + i_off, cy + j_off
                rayon_arc = _math.hypot(i_off, j_off)
                a_dep = _math.atan2(cy - centre_y, cx - centre_x)
                a_fin = _math.atan2(ny - centre_y, nx - centre_x)
                horaire = 'G02' in line
                balayage = a_fin - a_dep
                if horaire and balayage >= -0.001:
                    balayage -= 2.0 * _math.pi     # cercle complet ou arc CW
                elif not horaire and balayage <= 0.001:
                    balayage += 2.0 * _math.pi
                px, py, pz = cx, cy, cz
                for k in range(1, 25):
                    frac = k / 24.0
                    a = a_dep + balayage * frac
                    qx = centre_x + rayon_arc * _math.cos(a)
                    qy = centre_y + rayon_arc * _math.sin(a)
                    qz = cz + (nz - cz) * frac
                    seg = (px, py, pz, qx, qy, qz)
                    if abs(nz - cz) > 0.001:
                        ramps.append(seg)
                    elif qz <= 0.001:
                        cuts.append(seg)
                    else:
                        rapids.append(seg)
                    px, py, pz = qx, qy, qz
                cx, cy, cz = nx, ny, nz

            elif 'G83' in line or 'G81' in line:
                drills.append((nx, ny, nz))
                cx, cy, cz = nx, ny, nz

        return cuts, ramps, rapids, drills

    def _update_plot_from_gcode(self, gcode: str) -> None:
        """Affiche les trajectoires 3D reelles du G-code, en relief.

        G01 a plat en matiere (Z<=0) : trait plein cyan.
        G01 rampe / plongee          : trait pointille vert.
        G00 (ou G01 hors matiere)    : trait pointille gris, au-dessus de la piece.
        G83/G81                      : marqueurs de percage orange, a la profondeur reelle.

        Si la source est un STEP, le maillage filaire de la piece reste
        affiche en filigrane sous les trajectoires : on peut ainsi comparer
        directement le parcours d'outil a la geometrie d'origine.
        """
        cuts, ramps, rapids, drills = self._parse_gcode_paths(gcode)

        self._ax.cla()
        self._appliquer_style_sombre(self._ax)

        if not cuts and not ramps and not rapids and not drills:
            self._console_print("[!] Aucune trajectoire de coupe a afficher.", "warning")
            self._canvas.draw()
            return

        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []

        # Piece STEP en filigrane sous les trajectoires (comparaison directe)
        if self._source_actif == "step" and self._step_maillage:
            vertices = self._step_maillage["vertices"]
            edges    = self._step_maillage["edges"]
            if vertices and edges:
                self._ax.add_collection3d(Line3DCollection(
                    [(vertices[a], vertices[b]) for a, b in edges],
                    colors=_PLOT_STEP_MESH, linewidths=0.4, alpha=0.22,
                ))
                xs.extend(v[0] for v in vertices)
                ys.extend(v[1] for v in vertices)
                zs.extend(v[2] for v in vertices)

        def _trace_segments(
            segments: list[tuple], color: str, lw: float,
            style: str, alpha: float, ordre: int,
        ) -> None:
            for x1, y1, z1, x2, y2, z2 in segments:
                self._ax.plot(
                    [x1, x2], [y1, y2], [z1, z2],
                    color=color, linewidth=lw, linestyle=style, alpha=alpha,
                    solid_capstyle="round", zorder=ordre,
                )
                xs.extend((x1, x2))
                ys.extend((y1, y2))
                zs.extend((z1, z2))

        # Deplacements rapides d'abord (sous les traits de coupe, zorder bas)
        _trace_segments(rapids, _PLOT_RAPIDE,   0.6, ":",  0.5, 1)
        _trace_segments(ramps,  _PLOT_POCHE,    0.8, "--", 0.7, 2)
        _trace_segments(cuts,   _PLOT_GEOMETRY, 1.1, "-",  0.9, 3)

        # Points de percage G83/G81, a leur profondeur reelle
        for px, py, pz in drills:
            bras = 2.5
            self._ax.plot([px - bras, px + bras], [py, py], [pz, pz],
                          color=_PLOT_PERCAGE, linewidth=1.2, zorder=4)
            self._ax.plot([px, px], [py - bras, py + bras], [pz, pz],
                          color=_PLOT_PERCAGE, linewidth=1.2, zorder=4)
            self._ax.scatter([px], [py], [pz],
                             color=_PLOT_PERCAGE, s=25, zorder=5)
            xs.append(px)
            ys.append(py)
            zs.append(pz)

        if xs and ys and zs:
            self._appliquer_aspect_3d_egal(self._ax, xs, ys, zs)

        self._canvas.draw()

    def _update_plot(
        self,
        liste_lignes:  list[tuple],
        liste_cercles: list[dict]  | None = None,
        liste_poches:  list[tuple] | None = None,
    ) -> None:
        """Affiche la geometrie DXF (2D) a plat, en Z=0, dans le viewer 3D."""
        self._ax.cla()
        self._appliquer_style_sombre(self._ax)

        if not liste_lignes and not liste_cercles and not liste_poches:
            self._ax.text2D(
                0.5, 0.5, "Aucune entite LINE ou CIRCLE trouvee",
                ha="center", va="center", color="#555555", fontsize=12,
                transform=self._ax.transAxes,
            )
            self._canvas.draw()
            return

        xs: list[float] = []
        ys: list[float] = []

        for x1, y1, x2, y2 in liste_lignes:
            self._ax.plot([x1, x2], [y1, y2], color=_PLOT_GEOMETRY, linewidth=1.0,
                          solid_capstyle="round")
            xs.extend((x1, x2))
            ys.extend((y1, y2))
        if liste_lignes:
            self._ax.scatter(
                [s[0] for s in liste_lignes], [s[1] for s in liste_lignes],
                color=_PLOT_GEOMETRY, s=6, zorder=3, alpha=0.6,
            )

        for x1, y1, x2, y2 in (liste_poches or []):
            self._ax.plot([x1, x2], [y1, y2], color=_PLOT_POCHE, linewidth=1.2,
                          linestyle="--", solid_capstyle="round")
            xs.extend((x1, x2))
            ys.extend((y1, y2))

        for cercle in (liste_cercles or []):
            cx, cy, rayon = cercle["center"][0], cercle["center"][1], cercle["radius"]
            bras = max(rayon * 0.6, 1.5)
            self._ax.plot([cx - bras, cx + bras], [cy, cy],
                          color=_PLOT_PERCAGE, linewidth=1.2, zorder=4)
            self._ax.plot([cx, cx], [cy - bras, cy + bras],
                          color=_PLOT_PERCAGE, linewidth=1.2, zorder=4)
            angles = [2 * math.pi * i / 64 for i in range(65)]
            self._ax.plot(
                [cx + rayon * math.cos(a) for a in angles],
                [cy + rayon * math.sin(a) for a in angles],
                color=_PLOT_PERCAGE, linewidth=0.8, linestyle="--", alpha=0.7, zorder=3,
            )
            xs.extend((cx - rayon, cx + rayon))
            ys.extend((cy - rayon, cy + rayon))

        if xs and ys:
            self._appliquer_aspect_3d_egal(self._ax, xs, ys, [0.0])

        self._canvas.draw()

    def _update_plot_step(self, filepath: str) -> None:
        """Affiche le maillage filaire 3D d'une piece STEP (theme sombre).

        Delegue entierement la lecture/tessellation a step_parser -- ici on ne
        fait que tracer les aretes recues sous forme de Line3DCollection.
        """
        try:
            maillage = step_parser.extraire_maillage_filaire(filepath)
        except Exception as exc:
            self._console_print(f"[ERREUR] Maillage STEP : {exc}", "error")
            return

        # Cache pour la superposition piece/trajectoires apres generation
        self._step_maillage = maillage

        self._ax.cla()
        self._appliquer_style_sombre(self._ax)

        vertices = maillage["vertices"]
        edges    = maillage["edges"]

        if not vertices or not edges:
            self._ax.text2D(
                0.5, 0.5, "Maillage STEP vide ou illisible",
                ha="center", va="center", color="#555555", fontsize=12,
                transform=self._ax.transAxes,
            )
            self._canvas.draw()
            return

        segments = [(vertices[a], vertices[b]) for a, b in edges]
        self._ax.add_collection3d(Line3DCollection(
            segments, colors=_PLOT_STEP_MESH, linewidths=0.5, alpha=0.85,
        ))

        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        zs = [v[2] for v in vertices]
        self._appliquer_aspect_3d_egal(self._ax, xs, ys, zs)

        self._canvas.draw()

    @staticmethod
    def _appliquer_aspect_3d_egal(
        ax: Axes, xs: list[float], ys: list[float], zs: list[float],
    ) -> None:
        """Force un ratio d'aspect 1:1:1 sur les 3 axes.

        Sans ca, matplotlib etire chaque axe independamment et deforme la
        piece (un cercle apparait ovale, un cube devient un pave).
        """
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)

        demi_etendue = max(x_max - x_min, y_max - y_min, z_max - z_min, 1.0) / 2.0
        cx = (x_max + x_min) / 2.0
        cy = (y_max + y_min) / 2.0
        cz = (z_max + z_min) / 2.0

        ax.set_xlim(cx - demi_etendue, cx + demi_etendue)
        ax.set_ylim(cy - demi_etendue, cy + demi_etendue)
        ax.set_zlim(cz - demi_etendue, cz + demi_etendue)
        try:
            ax.set_box_aspect((1, 1, 1))
        except AttributeError:
            pass   # matplotlib trop ancien -- degrade gracieusement

    @staticmethod
    def _appliquer_style_sombre(ax: Axes) -> None:
        ax.set_facecolor(_PLOT_BG)
        ax.grid(True, color=_PLOT_GRID, linewidth=0.5, linestyle="--", alpha=0.8)
        ax.set_axisbelow(True)

        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            try:
                axis.set_pane_color((0.10, 0.10, 0.10, 1.0))
            except AttributeError:
                pass
            try:
                axis._axinfo["grid"].update(color=_PLOT_GRID, linewidth=0.5)
            except (AttributeError, KeyError):
                pass
            try:
                axis.line.set_color(_PLOT_GRID)
            except AttributeError:
                pass

        ax.tick_params(colors=_PLOT_AXIS_TEXT, labelsize=8)
        ax.set_xlabel("X (mm)", color=_PLOT_AXIS_TEXT, fontsize=9)
        ax.set_ylabel("Y (mm)", color=_PLOT_AXIS_TEXT, fontsize=9)
        ax.set_zlabel("Z (mm)", color=_PLOT_AXIS_TEXT, fontsize=9)

    # ------------------------------------------------------------------
    # Console
    # ------------------------------------------------------------------

    def _console_print(self, message: str, level: str = "info") -> None:
        """Affiche un message dans la console avec la couleur du niveau.

        level : 'info' (gris clair) | 'success' (vert) |
                'warning' (orange)  | 'error' (rouge vif)
        """
        print(message)
        self._console.configure(state="normal")
        self._console._textbox.insert("end", message + "\n", level)
        self._console.see("end")
        self._console.configure(state="disabled")


# ===========================================================================
# UTILITAIRE MODULE
# ===========================================================================

def _parse_float_entry(entry: ctk.CTkEntry, defaut: float) -> float:
    """Lit un float depuis un CTkEntry. Retourne defaut si invalide ou <= 0."""
    try:
        v = float(entry.get().replace(",", "."))
        return v if v > 0 else defaut
    except (ValueError, AttributeError):
        return defaut
