"""
models.py -- Structures de donnees metier FAO.

Centralise tous les parametres d'usinage afin que chaque module
importe depuis ici plutot que de redefinir ses propres dicts.

Hierarchie :
  ToolParams      -- un outil coupant (diametre, longueur utile, avances, broche)
  ToolMagazine    -- magasin : dict T# -> ToolParams
  MachiningConfig -- configuration complete (magasin + limites machine + materiau)
  DxfEntity       -- entite geometrique brute issue du parseur DXF

Types de machines reconnus par gcode_generator :
  "ISO_Standard" -- Fanuc / Heidenhain / Mazak  : T{n} M06, M30, %
  "GRBL_CNC"     -- Shapeoko / Genmitsu / OpenBuilds : M00 (pause), M2
  "Haas_Fanuc"   -- Haas / Fanuc industriel : % debut+fin, O0001, G43 H{n}
  (extensible via gcode_generator._REGISTRY)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal

# Types de machines supportes
MachineType = Literal["ISO_Standard", "GRBL_CNC", "Haas_Fanuc"]

MACHINE_TYPES: dict[str, str] = {
    "ISO Standard (Fanuc / Heidenhain)": "ISO_Standard",
    "GRBL (Shapeoko / Genmitsu)":        "GRBL_CNC",
    "Haas / Fanuc":                      "Haas_Fanuc",
}

# Materiaux bruts reconnus par les securites physiques
MATERIAL_OPTIONS: list[str] = ["Plastique", "Bois", "Aluminium", "Acier"]

# Seuils de securite pour l'Acier
ACIER_MAX_PASS_DEPTH: float = 1.0    # mm
ACIER_MAX_FEEDRATE:   float = 400.0  # mm/min


# ===========================================================================
# OUTIL COUPANT
# ===========================================================================

@dataclass
class ToolParams:
    """Parametres complets d'un outil coupant.

    Attributes:
        tool_number:     Numero T dans le magasin (ex: 1, 2).
        tool_diameter:   Diametre en mm (ex: 6.0 pour une fraise D6).
        spindle_speed:   Vitesse broche en tr/min.
        feedrate:        Avance de contournage XY en mm/min.
        plunge_feedrate: Avance de plongee Z en mm/min.
        tool_length:     Longueur utile (partie coupante sous mandrin) en mm.
                         Utilisee pour la detection de collision mandrin.
    """
    tool_number:     int
    tool_diameter:   float
    spindle_speed:   int
    feedrate:        float
    plunge_feedrate: float
    tool_length:     float = 30.0


# ===========================================================================
# MAGASIN D'OUTILS
# ===========================================================================

@dataclass
class ToolMagazine:
    """Magasin d'outils : associe un numero T a ses parametres.

    Exemples :
        mag = ToolMagazine()
        mag.add(ToolParams(1, 6.0, 18000, 1200, 300, tool_length=30))
        mag.add(ToolParams(2, 4.0, 20000,  800, 150, tool_length=25))
        t1 = mag.require(1)

    Attributes:
        tools: Dictionnaire {numero_T: ToolParams}.
    """
    tools: dict[int, ToolParams] = field(default_factory=dict)

    def add(self, tool: ToolParams) -> None:
        """Ajoute ou ecrase l'outil au numero tool.tool_number."""
        self.tools[tool.tool_number] = tool

    def remove(self, number: int) -> None:
        """Supprime l'outil T{number} si present."""
        self.tools.pop(number, None)

    def get(self, number: int) -> ToolParams | None:
        """Retourne l'outil T{number}, ou None s'il est absent."""
        return self.tools.get(number)

    def require(self, number: int) -> ToolParams:
        """Retourne l'outil T{number} ou leve KeyError s'il est absent."""
        tool = self.tools.get(number)
        if tool is None:
            available = sorted(self.tools.keys())
            raise KeyError(
                f"Outil T{number} absent du magasin. "
                f"Outils disponibles : {available}"
            )
        return tool

    def numbers(self) -> list[int]:
        """Retourne la liste triee des numeros T disponibles."""
        return sorted(self.tools.keys())


# ===========================================================================
# CONFIGURATION D'USINAGE
# ===========================================================================

@dataclass
class MachiningConfig:
    """Configuration complete d'usinage.

    Regroupe le magasin d'outils, les profondeurs par defaut,
    les limites physiques de la machine et le materiau brut.

    La profondeur peut etre surchargee calque par calque dans le DXF
    via la convention '_Z<valeur>' (ex: 'Contour_Ext_Z-8').
    En l'absence d'information dans le calque, default_target_depth s'applique.

    Attributes:
        safety_z:             Hauteur de degagement en mm (positif).
        default_target_depth: Profondeur finale si aucune info calque (negatif).
        default_pass_depth:   Profondeur par passe (positif).
        magazine:             Magasin d'outils.
        tool_number_contour:  T# pour contournage.
        tool_number_poche:    T# pour pocketing.
        tool_number_drill:    T# pour percage.
        peck_depth:           Pas de debourrage G83 en mm.
        stepover_poche:       Taux de recouvrement ]0, 1[ pour pocketing.
        machine_type:         Type de controleur (cle de _REGISTRY).
        cnc_limit_x:          Dimension max de la table en X (mm).
        cnc_limit_y:          Dimension max de la table en Y (mm).
        material_type:        Materiau brut (valeur de MATERIAL_OPTIONS).
        stock_dimensions:     Dimensions (x, y, z) du brut en mm, renseignees
                               automatiquement depuis step_parser.extraire_dimensions_brut
                               lors d'un import STEP. None pour un flux DXF classique
                               (pas de notion de brut 3D explicite dans ce cas).
                               Utilise par le Bouclier de Securite (gcode_generator)
                               pour valider la bbox 3D vs la table et la longueur
                               utile des outils vs la profondeur totale du brut.
    """
    safety_z:             float        = 5.0
    default_target_depth: float        = -6.0
    default_pass_depth:   float        = 2.0
    magazine:             ToolMagazine = field(default_factory=ToolMagazine)
    tool_number_contour:  int          = 1
    tool_number_poche:    int          = 2
    tool_number_drill:    int          = 2
    peck_depth:           float        = 2.0
    stepover_poche:       float        = 0.5
    machine_type:         str          = "ISO_Standard"
    cnc_limit_x:          float        = 300.0
    cnc_limit_y:          float        = 200.0
    material_type:        str          = "Aluminium"
    stock_dimensions:     tuple[float, float, float] | None = None

    def tool_contour(self) -> ToolParams:
        return self.magazine.require(self.tool_number_contour)

    def tool_poche(self) -> ToolParams:
        return self.magazine.require(self.tool_number_poche)

    def tool_drill(self) -> ToolParams:
        return self.magazine.require(self.tool_number_drill)

    def compute_pass_count(self, target_depth: float | None = None) -> int:
        depth = target_depth if target_depth is not None else self.default_target_depth
        return math.ceil(abs(depth) / self.default_pass_depth)

    def definir_brut_depuis_step(self, dimensions_brut: dict) -> None:
        """Renseigne stock_dimensions depuis le dict retourne par
        step_parser.extraire_dimensions_brut (cles 'x', 'y', 'z' en mm).

        Ne depend pas de step_parser (couplage evite) : accepte tout
        mapping possedant ces trois cles.
        """
        self.stock_dimensions = (
            float(dimensions_brut["x"]),
            float(dimensions_brut["y"]),
            float(dimensions_brut["z"]),
        )


# ===========================================================================
# ENTITE DXF
# ===========================================================================

@dataclass
class DxfEntity:
    """Representation generique d'une entite geometrique extraite du DXF.

    Attributes:
        entity_type: 'LINE', 'ARC', 'CIRCLE', 'LWPOLYLINE', etc.
        layer:       Nom du calque tel que lu dans le fichier DXF.
        raw_data:    Donnees brutes issues de ezdxf (dict).
    """
    entity_type: str
    layer:       str
    raw_data:    dict


# ===========================================================================
# COMPATIBILITE ASCENDANTE
# ===========================================================================

@dataclass
class MachiningParams:
    """Ancienne structure de parametres -- conservee pour compatibilite.
    Preferez MachiningConfig pour les nouveaux developpements.
    """
    safety_z:     float
    target_depth: float
    pass_depth:   float
    tool: ToolParams = field(default_factory=lambda: ToolParams(
        tool_number=1, tool_diameter=6.0, spindle_speed=18000,
        feedrate=1200.0, plunge_feedrate=300.0, tool_length=30.0,
    ))

    def compute_pass_count(self) -> int:
        return math.ceil(abs(self.target_depth) / self.pass_depth)
