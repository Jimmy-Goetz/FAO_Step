"""
gcode_generator.py -- Generation G-code ISO pour la FAO.

Point d'entree principal :
  generer_programme_complet(entites_dxf, config) -> str

Avant toute generation, effectue des verifications de securite physique :
  - Bounding box des entites vs limites table (cnc_limit_x / cnc_limit_y)
  - Coordonnees negatives (hors table)
  - Profondeur maximale vs longueur utile de l'outil (collision mandrin)
  - Materiau Acier : passe Z et avance F trop elevees

Architecture post-processeur :
  La generation des blocs machine-specifiques est deleguee a des classes
  PostProcesseur decouplees du coeur algorithmique. Ajout d'un controleur :
    1. Sous-classez PostProcesseur.
    2. Surchargez les methodes necessaires.
    3. Enregistrez dans _REGISTRY.
"""

from __future__ import annotations

import math
import re
from collections import defaultdict
from dataclasses import replace

from models import (
    DxfEntity, MachiningConfig, ToolParams,
    ACIER_MAX_FEEDRATE, ACIER_MAX_PASS_DEPTH,
)
from geometry import calculer_offset_contour, calculer_trajectoires_poche, CoteOffset

_SAFETY_Z: float = 5.0
TOLERANCE_FACE_FOND_BRUT: float = 0.5   # mm, pour exclure le dessous du brut STEP des poches


# ===========================================================================
# ARCHITECTURE POST-PROCESSEUR
# ===========================================================================

class PostProcesseur:
    """Interface de base pour tous les post-processeurs G-code.

    Chaque methode retourne une liste de lignes G-code (str).
    Sous-classez et surchargez pour ajouter un nouveau controleur,
    puis enregistrez l'instance dans _REGISTRY.
    """
    label: str = "Base"

    def initialisation(self) -> list[str]:
        """Blocs de mise en mode places en debut de programme (apres en-tete)."""
        return [
            "G21   ; millimetres",
            "G90   ; coordonnees absolues",
            "G94   ; avance en mm/min",
            "G17   ; plan XY (requis pour les arcs G02/G03)",
        ]

    def changement_outil(self, tool: ToolParams, safety_z: float) -> list[str]:
        raise NotImplementedError

    def pied_programme(self, safety_z: float) -> list[str]:
        raise NotImplementedError

    def pre_programme(self) -> list[str]:
        """Lignes absolument premieres, avant tout commentaire d'en-tete.
        Utilise pour le marqueur '%' Haas/Fanuc qui doit etre sur la ligne 1.
        """
        return []

    def commentaire(self, texte: str) -> str:
        """Formate un commentaire selon la syntaxe du controleur."""
        return f"; {texte}"

    @staticmethod
    def coord(v: float) -> str:
        return f"{v:.3f}"


class PostProcesseurISO(PostProcesseur):
    """Fanuc / Heidenhain / Mazak -- norme ISO 6983.

    Changement outil : T{n} M06 (ATC automatique)
    Fin programme    : M30 + marqueur %
    Commentaires     : point-virgule (;)
    """
    label = "ISO Standard"

    def initialisation(self) -> list[str]:
        return [
            "G21   ; millimetres",
            "G90   ; coordonnees absolues",
            "G94   ; avance en mm/min",
            "G17   ; plan XY (requis pour les arcs G02/G03)",
        ]

    def changement_outil(self, tool: ToolParams, safety_z: float) -> list[str]:
        return [
            f"; ============================================================",
            f"; CHANGEMENT OUTIL : T{tool.tool_number}",
            f";   Diametre       : D{tool.tool_diameter:.1f} mm",
            f";   Longueur utile : {tool.tool_length:.0f} mm",
            f";   Broche         : S{tool.spindle_speed} tr/min",
            f";   Avance XY      : F{tool.feedrate:.0f} mm/min",
            f";   Avance Z       : F{tool.plunge_feedrate:.0f} mm/min",
            f"; ============================================================",
            f"T{tool.tool_number} M06",
            f"M03 S{tool.spindle_speed}",
            f"G00 Z{self.coord(safety_z)}",
            "",
        ]

    def pied_programme(self, safety_z: float) -> list[str]:
        return [
            "; ============================================================",
            "; FIN DE PROGRAMME",
            "; ============================================================",
            f"G00 Z{self.coord(safety_z)}",
            "M05",
            "M30",
            "%",
        ]


class PostProcesseurGRBL(PostProcesseur):
    """GRBL 1.1 -- Shapeoko, Genmitsu, OpenBuilds, CNC Router Parts.

    M06 inconnu sur GRBL -> M00 (feed hold) pour changement manuel.
    Fin programme : M2
    Commentaires  : (...) pour messages operateur, ; pour technique
    """
    label = "GRBL CNC"

    def initialisation(self) -> list[str]:
        return [
            "G21   ; millimetres",
            "G90   ; coordonnees absolues",
            "G94   ; avance en mm/min",
            "G17   ; plan XY (requis sur certains GRBL)",
        ]

    def changement_outil(self, tool: ToolParams, safety_z: float) -> list[str]:
        return [
            f"(--- CHANGEMENT OUTIL T{tool.tool_number} ---)",
            f"(  Installer : D{tool.tool_diameter:.1f}mm  L={tool.tool_length:.0f}mm"
            f"  S{tool.spindle_speed}tr/min  F{tool.feedrate:.0f}mm/min)",
            f"(  Puis appuyer sur CYCLE START pour continuer)",
            "M00",
            f"M03 S{tool.spindle_speed}",
            f"G00 Z{self.coord(safety_z)}",
            "",
        ]

    def pied_programme(self, safety_z: float) -> list[str]:
        return [
            f"G00 Z{self.coord(safety_z)}",
            "M05",
            "M2",
            "%",
        ]

    def commentaire(self, texte: str) -> str:
        return f"; {texte}"


class PostProcesseurHaasFanuc(PostProcesseur):
    """Haas / Fanuc -- armoires industrielles standard.

    Specificites par rapport a ISO generique :
      - Marqueur '%' OBLIGATOIRE en premiere ET derniere ligne du fichier
        (heritage lecture bande perforee, requis par parseur CNC industriel)
      - Numero de programme O0001 en debut de fichier
      - Commentaires entre parentheses (...) en majuscules (syntaxe Haas)
      - G43 H{n} : compensation longueur outil (TLO - Tool Length Offset)
      - G49 : annulation TLO en entete
      - G80 : annulation cycles fixes en entete
      - Coordonnees a 3 decimales (standard Fanuc)
    """
    label = "Haas / Fanuc"

    def pre_programme(self) -> list[str]:
        # Le '%' doit etre la toute premiere ligne du fichier Haas/Fanuc.
        # Il est injecte avant l'en-tete commentaires via pre_programme().
        return ["%"]

    def initialisation(self) -> list[str]:
        return [
            "O0001  (PROGRAMME GENERE PAR MONLOGICIELFAO)",
            "G21   (MILLIMETRES)",
            "G90   (COORDONNEES ABSOLUES)",
            "G94   (AVANCE EN MM/MIN)",
            "G17   (PLAN XY)",
            "G49   (ANNULATION COMPENSATION LONGUEUR)",
            "G80   (ANNULATION CYCLES FIXES)",
        ]

    def changement_outil(self, tool: ToolParams, safety_z: float) -> list[str]:
        return [
            f"(============================================================)",
            f"(CHANGEMENT OUTIL : T{tool.tool_number})",
            f"(  DIAMETRE       : D{tool.tool_diameter:.1f}MM)",
            f"(  LONGUEUR UTILE : {tool.tool_length:.0f}MM)",
            f"(  BROCHE         : S{tool.spindle_speed}TR/MIN)",
            f"(  AVANCE XY      : F{tool.feedrate:.0f}MM/MIN)",
            f"(  AVANCE Z       : F{tool.plunge_feedrate:.0f}MM/MIN)",
            f"(============================================================)",
            f"T{tool.tool_number} M06",
            f"G43 H{tool.tool_number}  (COMPENSATION LONGUEUR T{tool.tool_number})",
            f"M03 S{tool.spindle_speed}",
            f"G00 Z{self.coord(safety_z)}",
            "",
        ]

    def pied_programme(self, safety_z: float) -> list[str]:
        return [
            "(============================================================)",
            "(FIN DE PROGRAMME)",
            "(============================================================)",
            f"G00 Z{self.coord(safety_z)}",
            "M05",
            "G49   (ANNULATION COMPENSATION LONGUEUR)",
            "M30",
            "%",  # Marqueur fin de bande obligatoire
        ]

    def commentaire(self, texte: str) -> str:
        # Haas/Fanuc : commentaires entre parentheses, majuscules
        return f"({texte.upper()})"


# ---------------------------------------------------------------------------
# Registre : ajouter ici tout nouveau post-processeur
# ---------------------------------------------------------------------------
_REGISTRY: dict[str, PostProcesseur] = {
    "ISO_Standard": PostProcesseurISO(),
    "GRBL_CNC":     PostProcesseurGRBL(),
    "Haas_Fanuc":   PostProcesseurHaasFanuc(),
}


def get_post_processeur(machine_type: str) -> PostProcesseur:
    """Retourne le post-processeur pour machine_type.
    Retourne PostProcesseurISO() si machine_type est inconnu.
    """
    return _REGISTRY.get(machine_type, PostProcesseurISO())


# ===========================================================================
# SECURITES PHYSIQUES
# ===========================================================================

class SecritePhysiqueError(ValueError):
    """Exception levee lors d'une violation de barriere de securite physique."""


def _bounding_box_entites(
    entites: list[DxfEntity],
) -> tuple[float, float, float, float] | None:
    """Calcule la bounding box (xmin, ymin, xmax, ymax) de toutes les entites.
    Retourne None si aucune entite geometrique."""
    xs: list[float] = []
    ys: list[float] = []
    for e in entites:
        if e.entity_type == "LINE":
            xs += [e.raw_data["start"][0], e.raw_data["end"][0]]
            ys += [e.raw_data["start"][1], e.raw_data["end"][1]]
        elif e.entity_type == "CIRCLE":
            cx, cy = e.raw_data["center"][0], e.raw_data["center"][1]
            r = e.raw_data["radius"]
            xs += [cx - r, cx + r]
            ys += [cy - r, cy + r]
    return (min(xs), min(ys), max(xs), max(ys)) if xs else None


def _valider_geometrie_et_config(
    entites: list[DxfEntity],
    config: MachiningConfig,
    contour_par_z: dict[float, list],
    poche_par_z:   dict[float, list],
    drill_par_z:   dict[float, list],
    verifier_collision_brut: bool = True,
) -> None:
    """Verifie les barrieres de securite physique avant toute generation.

    Verifications effectuees (dans l'ordre) :
      1. Bounding box vs limites table (cnc_limit_x / cnc_limit_y)
      2. Coordonnees negatives (geometrie hors table)
      3. Profondeur maximale vs longueur utile de l'outil (collision mandrin)
      4. Brut STEP (si config.stock_dimensions defini) : bbox 3D vs table et
         longueur utile de l'outil vs profondeur totale du brut
      5. Materiau Acier : avance F et profondeur de passe

    Raises:
        SecritePhysiqueError: si une condition n'est pas respectee.
    """
    # ------------------------------------------------------------------
    # 1. Bounding box vs limites table
    # ------------------------------------------------------------------
    bb = _bounding_box_entites(entites)
    if bb is not None:
        xmin, ymin, xmax, ymax = bb

        if xmin < 0.0 or ymin < 0.0:
            raise SecritePhysiqueError(
                f"[SECURITE] Coordonnees hors table :\n"
                f"  X_min = {xmin:.3f} mm  |  Y_min = {ymin:.3f} mm\n"
                f"L'origine DXF doit etre a X >= 0 et Y >= 0 (decalage G54).\n"
                f"Deplacez la geometrie dans le quadrant positif."
            )
        if xmax > config.cnc_limit_x:
            raise SecritePhysiqueError(
                f"[SECURITE] Depassement table en X :\n"
                f"  X_max = {xmax:.3f} mm  >  Limite X = {config.cnc_limit_x:.0f} mm\n"
                f"Reduisez la piece ou augmentez la limite X dans la section Machine."
            )
        if ymax > config.cnc_limit_y:
            raise SecritePhysiqueError(
                f"[SECURITE] Depassement table en Y :\n"
                f"  Y_max = {ymax:.3f} mm  >  Limite Y = {config.cnc_limit_y:.0f} mm\n"
                f"Reduisez la piece ou augmentez la limite Y dans la section Machine."
            )

    # ------------------------------------------------------------------
    # 2. Longueur utile outil vs profondeur maximale (collision mandrin)
    # ------------------------------------------------------------------
    for ops, tnum, role in [
        (contour_par_z, config.tool_number_contour, "Contournage"),
        (poche_par_z,   config.tool_number_poche,   "Poche"),
        (drill_par_z,   config.tool_number_drill,   "Percage"),
    ]:
        if not ops:
            continue
        t = config.magazine.get(tnum)
        if t is None:
            continue
        prof_max = max(abs(z) for z in ops.keys())
        if prof_max > t.tool_length:
            raise SecritePhysiqueError(
                f"[SECURITE] Risque collision mandrin ({role}) :\n"
                f"  Profondeur max    = {prof_max:.1f} mm\n"
                f"  Longueur utile T{t.tool_number} = {t.tool_length:.0f} mm\n"
                f"Montez l'outil plus bas dans le mandrin ou choisissez "
                f"une fraise plus longue."
            )

    # ------------------------------------------------------------------
    # 4. Brut STEP (si dimensions 3D fournies) : bbox vs table + collision mandrin
    # ------------------------------------------------------------------
    if config.stock_dimensions is not None:
        stock_x, stock_y, stock_z = config.stock_dimensions

        if stock_x > config.cnc_limit_x:
            raise SecritePhysiqueError(
                f"[SECURITE STEP] Brut trop grand en X :\n"
                f"  Brut X = {stock_x:.3f} mm  >  Limite X = {config.cnc_limit_x:.0f} mm\n"
                f"Reduisez le brut ou augmentez la limite X dans la section Machine."
            )
        if stock_y > config.cnc_limit_y:
            raise SecritePhysiqueError(
                f"[SECURITE STEP] Brut trop grand en Y :\n"
                f"  Brut Y = {stock_y:.3f} mm  >  Limite Y = {config.cnc_limit_y:.0f} mm\n"
                f"Reduisez le brut ou augmentez la limite Y dans la section Machine."
            )

        # Collision mandrin : la fraise doit descendre jusqu'a la face la plus
        # basse du brut (Z = -stock_z, convention step_parser : face superieure
        # de la piece = Z=0) sans que le mandrin ne percute le dessus de la piece.
        # stock_z est la borne haute (pire cas) : aucune feature du STEP ne peut
        # etre plus profonde que la bounding box entiere du brut.
        # Ce controle n'a de sens que si le programme detoure la silhouette sur
        # toute la profondeur (verifier_collision_brut) : les faces secondaires
        # du mode multi-faces n'usinent que leurs features, dont la profondeur
        # est deja verifiee individuellement (_verifier_mandrin_outil_auto).
        if verifier_collision_brut:
            for tnum, role in [
                (config.tool_number_contour, "Contournage"),
                (config.tool_number_poche,   "Poche"),
                (config.tool_number_drill,   "Percage"),
            ]:
                t = config.magazine.get(tnum)
                if t is None:
                    continue
                if stock_z > t.tool_length:
                    raise SecritePhysiqueError(
                        f"[SECURITE STEP] Risque collision mandrin ({role}) :\n"
                        f"  Profondeur brut STEP = {stock_z:.1f} mm\n"
                        f"  Longueur utile T{t.tool_number} = {t.tool_length:.0f} mm\n"
                        f"La fraise T{t.tool_number} est trop courte pour atteindre le bas "
                        f"de la piece sans que le mandrin ne percute le dessus du brut.\n"
                        f"Montez l'outil plus bas ou choisissez une fraise plus longue."
                    )

    # ------------------------------------------------------------------
    # 5. Acier : avances et passes trop agressives
    # ------------------------------------------------------------------
    if config.material_type == "Acier":
        if config.default_pass_depth > ACIER_MAX_PASS_DEPTH:
            raise SecritePhysiqueError(
                f"[SECURITE ACIER] Profondeur de passe trop elevee :\n"
                f"  Passe configuree = {config.default_pass_depth:.1f} mm\n"
                f"  Maximum pour Acier = {ACIER_MAX_PASS_DEPTH:.1f} mm\n"
                f"Reduisez la profondeur de passe dans la configuration."
            )
        for tnum, role in [
            (config.tool_number_contour, "Contournage"),
            (config.tool_number_poche,   "Poche"),
            (config.tool_number_drill,   "Percage"),
        ]:
            t = config.magazine.get(tnum)
            if t and t.feedrate > ACIER_MAX_FEEDRATE:
                raise SecritePhysiqueError(
                    f"[SECURITE ACIER] Avance trop elevee ({role}) :\n"
                    f"  Avance T{t.tool_number} = {t.feedrate:.0f} mm/min\n"
                    f"  Maximum pour Acier = {ACIER_MAX_FEEDRATE:.0f} mm/min\n"
                    f"Reduisez l'avance XY de T{t.tool_number} dans le magasin."
                )


# ===========================================================================
# POINT D'ENTREE PRINCIPAL
# ===========================================================================

def generer_programme_complet(
    entites_dxf: list[DxfEntity],
    config: MachiningConfig,
) -> str:
    """Genere un programme G-code complet a partir des entites DXF brutes.

    Effectue les verifications de securite physique AVANT la generation.
    Leve SecritePhysiqueError (sous-classe de ValueError) si une barriere
    est violee. Leve KeyError si un outil requis est absent du magasin.

    Args:
        entites_dxf: Liste de DxfEntity issue de dxf_parser.
        config:      Configuration complete (MachiningConfig).

    Returns:
        Programme G-code complet en une seule chaine multiligne.
    """
    post          = get_post_processeur(config.machine_type)
    tool_contour  = config.tool_contour()
    tool_poche_   = config.tool_poche()
    tool_drill    = config.tool_drill()
    rayon_contour = tool_contour.tool_diameter / 2.0
    rayon_poche   = tool_poche_.tool_diameter  / 2.0

    # --- Groupement par role et profondeur ---
    contour_par_z: dict[float, list[tuple]] = defaultdict(list)
    poche_par_z:   dict[float, list[tuple]] = defaultdict(list)
    drill_par_z:   dict[float, list[dict]]  = defaultdict(list)

    for e in entites_dxf:
        z = _parse_z_from_layer(e.layer, config.default_target_depth)
        if e.entity_type == "LINE":
            seg = (
                e.raw_data["start"][0], e.raw_data["start"][1],
                e.raw_data["end"][0],   e.raw_data["end"][1],
            )
            if _est_calque_poche(e.layer):
                poche_par_z[z].append(seg)
            else:
                contour_par_z[z].append(seg)
        elif e.entity_type == "CIRCLE":
            drill_par_z[z].append(e.raw_data)

    if not contour_par_z and not poche_par_z and not drill_par_z:
        return "; Aucune geometrie a usiner.\n%"

    # --- Securites physiques ---
    _valider_geometrie_et_config(
        entites_dxf, config, contour_par_z, poche_par_z, drill_par_z
    )

    # --- Construction du programme ---
    sections:     list[str] = []
    outil_actuel: int | None = None

    def _changer_outil_si_besoin(numero: int) -> list[str]:
        nonlocal outil_actuel
        if outil_actuel == numero:
            return []
        outil_actuel = numero
        return post.changement_outil(
            config.magazine.require(numero), config.safety_z
        )

    # Ligne(s) pre-programme (ex: '%' obligatoire sur ligne 1 pour Haas/Fanuc)
    pre = post.pre_programme()
    entete = _entete_global(config, post, contour_par_z, poche_par_z, drill_par_z)
    if pre:
        sections.append("\n".join(pre) + "\n\n" + entete)
    else:
        sections.append(entete)

    # --- Contournage ---
    for z in sorted(contour_par_z.keys(), key=abs):
        lignes  = contour_par_z[z]
        t       = tool_contour
        paliers = _calculer_paliers_z(z, config.default_pass_depth)
        blocs: list[str] = []
        blocs += _changer_outil_si_besoin(config.tool_number_contour)
        blocs.append(post.commentaire(
            f"=== CONTOURNAGE  T{t.tool_number} (D{t.tool_diameter:.1f})"
            f"  Z_cible={_coord(z)}mm  {len(paliers)} passe(s) ==="
        ))
        blocs += _segments_multi_passes(
            calculer_offset_contour(lignes, rayon_contour, "exterieur"),
            paliers, t.feedrate, t.plunge_feedrate, tool=t,
        )
        sections.append("\n".join(blocs))

    # --- Pocketing ---
    for z in sorted(poche_par_z.keys(), key=abs):
        lignes  = poche_par_z[z]
        t       = tool_poche_
        paliers = _calculer_paliers_z(z, config.default_pass_depth)
        try:
            trajectoires = calculer_trajectoires_poche(
                lignes, rayon_poche, config.stepover_poche
            )
        except RuntimeError as exc:
            sections.append(post.commentaire(f"[POCHE Z={z}] IGNOREE : {exc}") + "\n")
            continue

        blocs: list[str] = []
        blocs += _changer_outil_si_besoin(config.tool_number_poche)
        blocs.append(post.commentaire(
            f"=== POCKETING  T{t.tool_number} (D{t.tool_diameter:.1f})"
            f"  Z_cible={_coord(z)}mm  {len(paliers)} passe(s)"
            f"  {len(trajectoires)} anneau(x) ==="
        ))
        blocs += _passes_poche(
            trajectoires, paliers, t.feedrate, t.plunge_feedrate, tool=t,
        )
        sections.append("\n".join(blocs))

    # --- Percage ---
    for z in sorted(drill_par_z.keys(), key=abs):
        cercles = drill_par_z[z]
        t       = tool_drill
        blocs: list[str] = []
        blocs += _changer_outil_si_besoin(config.tool_number_drill)
        blocs.append(post.commentaire(
            f"=== PERCAGE  T{t.tool_number} (D{t.tool_diameter:.1f})"
            f"  Z_cible={_coord(z)}mm  {len(cercles)} trou(s) ==="
        ))
        blocs += _cycles_percage(cercles, z, t.plunge_feedrate, config.peck_depth, tool=t)
        sections.append("\n".join(blocs))

    sections.append("\n".join(post.pied_programme(config.safety_z)))
    return "\n\n".join(sections)


# ===========================================================================
# TRADUCTION STEP -> STRUCTURES DXF-COMPATIBLES
# ===========================================================================

def _rectangle_vers_segments(
    largeur: float, hauteur: float, origine: tuple[float, float] = (0.0, 0.0),
) -> list[tuple]:
    """Construit les 4 segments chaines d'un rectangle ferme (sens horaire).

    Traduit une bounding box XY (silhouette du brut ou d'une face STEP) dans
    le meme format (x1,y1,x2,y2) que les entites LINE issues du DXF, afin de
    rester directement consommable par calculer_offset_contour /
    calculer_trajectoires_poche sans aucune adaptation de ces fonctions.
    """
    x0, y0 = origine
    x1, y1 = x0 + largeur, y0 + hauteur
    return [
        (x0, y0, x1, y0),
        (x1, y0, x1, y1),
        (x1, y1, x0, y1),
        (x0, y1, x0, y0),
    ]


def _cercle_vers_segments(
    cx: float, cy: float, rayon: float, nb_segments: int = 64,
) -> list[tuple]:
    """Approxime un cercle par un polygone a nb_segments cotes, chaine dans
    le sens trigonometrique (chaque segment finit exactement ou le suivant
    commence -- meme format (x1,y1,x2,y2) que les entites LINE du DXF).

    Sert a traduire un percage STEP (centre + rayon, deja ajustes par
    step_parser via moindres carres) en contour de poche circulaire,
    directement consommable par calculer_trajectoires_poche : le trou est
    alors fraise en colimacon (anneaux concentriques) plutot que perce au
    foret -- utile quand aucun foret de diametre exact n'est configure, ou
    que le trou est trop grand pour etre perce en un seul passage.
    """
    angles = [2.0 * math.pi * i / nb_segments for i in range(nb_segments + 1)]
    points = [(cx + rayon * math.cos(a), cy + rayon * math.sin(a)) for a in angles]
    return [
        (points[i][0], points[i][1], points[i + 1][0], points[i + 1][1])
        for i in range(nb_segments)
    ]


def _boucle_vers_segments(boucle: list[tuple[float, float]]) -> list[tuple]:
    """Convertit une boucle de points (x, y) en segments chaines fermes."""
    n = len(boucle)
    return [
        (boucle[i][0], boucle[i][1], boucle[(i + 1) % n][0], boucle[(i + 1) % n][1])
        for i in range(n)
    ]


def _face_step_vers_segments(face: dict) -> list[tuple]:
    """Traduit une face plane detectee par step_parser en contour de poche.

    Priorite au contour REEL de la face ('contours' : boucles frontieres
    extraites du maillage, y compris les ilots) : une poche en L ou avec
    ilot central est usinee fidele a sa forme. A defaut, repli sur la bbox
    (min_xy/max_xy), puis sur un carre d'aire equivalente (compatibilite
    avec d'anciennes structures FacePlane).
    """
    contours = face.get("contours") or []
    segments: list[tuple] = []
    for boucle in contours:
        if len(boucle) >= 3:
            segments.extend(_boucle_vers_segments(boucle))
    if segments:
        return segments

    if "min_xy" in face and "max_xy" in face:
        x0, y0 = face["min_xy"]
        x1, y1 = face["max_xy"]
        return _rectangle_vers_segments(x1 - x0, y1 - y0, (x0, y0))

    aire = face.get("aire", 0.0)
    if aire <= 0:
        return []
    cote = math.sqrt(aire)
    cx, cy = face["centre"]
    return _rectangle_vers_segments(cote, cote, (cx - cote / 2.0, cy - cote / 2.0))


def _choisir_outil_contour_auto(config: MachiningConfig) -> tuple[ToolParams, str]:
    """Selectionne automatiquement l'outil de contournage : le plus grand
    diametre du magasin, pour degager la silhouette exterieure du brut avec
    le moins de passes laterales possible (aucune contrainte de largeur en
    exterieur : l'outil travaille autour de la piece, pas dedans).

    Repli sur config.tool_contour() si le magasin est vide (KeyError si
    l'affectation configuree est elle aussi invalide -- comportement
    identique au flux DXF).

    Returns:
        (outil_choisi, raison) -- raison est un texte court destine au
        commentaire G-code, explicitant pourquoi cet outil a ete retenu.
    """
    outils = list(config.magazine.tools.values())
    if not outils:
        return config.tool_contour(), "magasin vide -- repli sur l'outil contour configure"

    meilleur = max(outils, key=lambda o: o.tool_diameter)
    return meilleur, "plus grand diametre du magasin, degagement rapide de la silhouette"


def _choisir_outil_forme(
    largeur_forme: float, config: MachiningConfig,
) -> tuple[ToolParams, str]:
    """Selectionne automatiquement, dans config.magazine, l'outil le plus
    adapte pour evider une forme interieure de largeur donnee : le diametre
    d'un percage, ou la plus petite dimension d'une poche rectangulaire.

    Ordre de priorite :
      1. Correspondance parfaite : un outil dont le diametre == la largeur
         de la forme (ex : un foret ou une fraise de Ø10mm pour un trou de
         Ø10mm).
      2. Repli securise : le plus grand outil du magasin dont le diametre
         reste strictement INFERIEUR a la largeur de la forme -- necessaire
         pour pouvoir la vider par anneaux concentriques (un outil plus
         grand que la forme ne rentre pas dedans).
      3. Repli final : config.tool_poche(), si aucun outil du magasin ne
         convient (magasin vide, ou tous les outils sont trop gros).

    Args:
        largeur_forme: Largeur utile de la forme a vider (mm) : diametre du
            trou (2 * rayon) ou plus petite dimension de la poche.
        config: Configuration d'usinage (magasin + outil de poche par defaut).

    Returns:
        (outil_choisi, raison) -- raison est un texte court destine au
        commentaire G-code, explicitant pourquoi cet outil a ete retenu.
    """
    outils = list(config.magazine.tools.values())

    for outil in outils:
        if math.isclose(outil.tool_diameter, largeur_forme, abs_tol=1e-6):
            return outil, "correspondance exacte de diametre"

    plus_petits = [o for o in outils if o.tool_diameter < largeur_forme]
    if plus_petits:
        meilleur = max(plus_petits, key=lambda o: o.tool_diameter)
        return meilleur, "plus grand outil du magasin sous la largeur de la forme"

    return config.tool_poche(), "aucun outil adapte dans le magasin -- repli sur l'outil de poche par defaut"


def _verifier_mandrin_outil_auto(
    outil: ToolParams, profondeur: float, contexte: str,
) -> None:
    """Verifie qu'un outil choisi automatiquement est assez long pour
    atteindre la profondeur demandee sans que le mandrin percute la piece.

    Complement indispensable du Bouclier de Securite global : celui-ci ne
    controle que les outils affectes par role dans la config
    (tool_number_contour / poche / drill), pas ceux substitues a la volee
    par la selection automatique.

    Raises:
        SecritePhysiqueError: si outil.tool_length < |profondeur|.
    """
    if outil.tool_length < abs(profondeur):
        raise SecritePhysiqueError(
            f"[SECURITE STEP] Risque collision mandrin ({contexte}) :\n"
            f"  Profondeur demandee = {abs(profondeur):.1f} mm\n"
            f"  Outil auto-selectionne T{outil.tool_number} "
            f"(D{outil.tool_diameter:.1f}mm) -- "
            f"longueur utile = {outil.tool_length:.0f} mm\n"
            f"L'outil choisi automatiquement est trop court pour cette operation. "
            f"Ajoutez au magasin un outil de diametre compatible et de longueur "
            f"suffisante, ou rallongez T{outil.tool_number}."
        )


# ===========================================================================
# POINT D'ENTREE STEP -- TRADUCTION + REUTILISATION DU MOTEUR EXISTANT
# ===========================================================================

def generer_gcode_depuis_step(
    features_3d: dict[str, list],
    config: MachiningConfig,
    inclure_contournage: bool = True,
    commentaires_entete: list[str] | None = None,
    corps_seulement: bool = False,
) -> str:
    """Traduit les features 3D d'une piece STEP en G-code complet.

    Args (specifiques multi-faces) :
        inclure_contournage: False pour les faces secondaires (le detourage
            de la silhouette n'est fait qu'une fois, sur la face Z_PLUS).
        commentaires_entete: Lignes de commentaire inserees en tete de
            programme (ex : instruction de retournement de la piece).
        corps_seulement: True pour ne retourner que le corps des operations
            (sans en-tete ni pied de programme) -- utilise par le mode
            4 axes pour assembler plusieurs faces dans un programme unique.

    Fonction "traductrice" uniquement : aucune nouvelle logique d'usinage
    n'est ecrite ici. Reutilise integralement calculer_offset_contour,
    calculer_trajectoires_poche, le Bouclier de Securite
    (_valider_geometrie_et_config) et l'architecture PostProcesseur
    (GRBL / ISO Standard / Haas-Fanuc avec G43 H et %), exactement comme
    generer_programme_complet pour le flux DXF.

    AFFECTATION DES OUTILS : entierement AUTOMATIQUE dans ce flux. Les
    affectations manuelles par role (tool_number_contour / poche / drill)
    ne sont PAS utilisees ici -- seul le contenu du magasin
    (config.magazine) compte. Chaque choix automatique est trace par un
    commentaire explicite dans le G-code genere.

    Traduction effectuee :
      1. Contournage : silhouette exterieure maximale (X, Y) = la bounding
         box du brut (config.stock_dimensions, calculee a l'etape 1 par
         step_parser.extraire_dimensions_brut). Les passes descendent
         jusqu'a la profondeur totale du bloc (config.stock_dimensions[2]).
         Outil auto : le plus grand diametre du magasin
         (_choisir_outil_contour_auto) -- degagement rapide, aucune
         contrainte de largeur en usinage exterieur.
      2. Poches : chaque face plane horizontale de
         features_3d["faces_planes"] devient un contour rectangulaire
         (bbox reelle de la face), injecte dans calculer_trajectoires_poche
         (anneaux concentriques). La face du dessous du brut entier
         (Z = -profondeur du bloc) est ignoree : ce n'est pas une poche,
         c'est le fond du contournage traversant, deja couvert par le
         contournage ci-dessus. Outil auto par face (_choisir_outil_forme,
         largeur = plus petite dimension de la face) : correspondance
         exacte de diametre, sinon le plus grand outil strictement sous la
         largeur, sinon config.tool_poche() en dernier recours.
      3. Percage : chaque trou de features_3d["percages"] (centre X/Y et
         rayon deja ajustes par step_parser via moindres carres) devient un
         contour de poche circulaire (_cercle_vers_segments), injecte dans
         calculer_trajectoires_poche : le trou est fraise en colimacon
         (anneaux concentriques), pas perce au foret. Outil auto par trou
         (_choisir_outil_forme, largeur = diametre du trou), memes regles
         que les poches.

    Args:
        features_3d: Retour de step_parser.analyser_features_3d :
            {"faces_planes": [...], "percages": [...]}.
        config: Configuration d'usinage. config.stock_dimensions doit avoir
            ete renseigne au prealable via
            config.definir_brut_depuis_step(dimensions_brut).

    Returns:
        Programme G-code complet (str), commente en francais, produit par
        le meme post-processeur que le flux DXF.

    Raises:
        ValueError: si config.stock_dimensions n'est pas renseigne.
        SecritePhysiqueError: si une barriere de securite physique est
            violee (bbox brut vs table, collision mandrin, limites Acier).
        KeyError: si un outil requis (contour/poche/percage) est absent
            du magasin.
    """
    if config.stock_dimensions is None:
        raise ValueError(
            "config.stock_dimensions doit etre renseigne avant generation STEP "
            "(appelez config.definir_brut_depuis_step(dimensions_brut) au "
            "prealable, avec le dict retourne par "
            "step_parser.extraire_dimensions_brut)."
        )

    stock_x, stock_y, stock_z = config.stock_dimensions
    profondeur_bloc = -abs(stock_z)   # convention : profondeur negative

    post = get_post_processeur(config.machine_type)

    # ------------------------------------------------------------------
    # 1. CONTOURNAGE : silhouette exterieure maximale du brut, passes
    #    jusqu'a la profondeur totale du bloc. Outil auto : plus grand
    #    diametre du magasin (les affectations manuelles par role ne sont
    #    pas utilisees dans le flux STEP -- seul le magasin compte).
    #    Desactive sur les faces secondaires (multi-faces) : le detourage
    #    n'est fait qu'une fois, sur la face principale Z_PLUS.
    # ------------------------------------------------------------------
    contour_par_z: dict[float, list[tuple]] = {}
    tool_contour: ToolParams | None = None
    raison_contour = ""
    rayon_contour = 0.0
    if inclure_contournage:
        tool_contour, raison_contour = _choisir_outil_contour_auto(config)
        rayon_contour = tool_contour.tool_diameter / 2.0
        _verifier_mandrin_outil_auto(
            tool_contour, profondeur_bloc, "Contournage auto-selectionne"
        )
        # Silhouette REELLE de la piece (projection XY calculee par
        # step_parser) si disponible : une piece non rectangulaire est
        # detouree suivant sa vraie forme. Repli sur la bbox du brut sinon.
        silhouette = features_3d.get("silhouette")
        if silhouette and len(silhouette) >= 3:
            contour_par_z[profondeur_bloc] = _boucle_vers_segments(silhouette)
        else:
            contour_par_z[profondeur_bloc] = _rectangle_vers_segments(stock_x, stock_y)

    # ------------------------------------------------------------------
    # 2. POCHES : chaque face plane horizontale -> contour rectangulaire.
    #    La face du dessous du brut entier est exclue.
    # ------------------------------------------------------------------
    # IMPORTANT : chaque feature (face plane OU percage) est conservee comme
    # une operation de pocketing INDEPENDANTE (un dict par operation), jamais
    # fusionnee avec une autre meme si elles partagent la meme profondeur Z.
    # calculer_trajectoires_poche() fait un unary_union avant de retrecir :
    # si on lui passe les segments de DEUX trous disjoints en un seul appel,
    # elle produit un anneau combine -- l'outil tracerait alors une coupe
    # rectiligne parasite entre les deux trous, a travers la matiere qui doit
    # rester en place. Un appel separe par feature elimine ce risque a la racine.
    #
    # Chaque operation porte son propre outil ("outil"), choisi
    # AUTOMATIQUEMENT par _choisir_outil_forme selon la largeur de la forme
    # (plus petite dimension d'une face, diametre d'un trou), et sa propre
    # explication ("desc") destinee au commentaire G-code.
    operations_poche: list[dict] = []

    for face in features_3d.get("faces_planes", []):
        if abs(face["z"] - profondeur_bloc) < TOLERANCE_FACE_FOND_BRUT:
            continue   # dessous du brut entier, pas une poche
        segments = _face_step_vers_segments(face)
        if not segments:
            continue

        # Largeur utile de la face = sa plus petite dimension : c'est elle
        # qui contraint le diametre maximal d'outil capable d'y entrer.
        if "min_xy" in face and "max_xy" in face:
            largeur_face = min(
                face["max_xy"][0] - face["min_xy"][0],
                face["max_xy"][1] - face["min_xy"][1],
            )
        else:
            largeur_face = math.sqrt(max(face.get("aire", 0.0), 0.0))

        z_face = round(face["z"], 4)
        outil_choisi, raison = _choisir_outil_forme(largeur_face, config)
        _verifier_mandrin_outil_auto(
            outil_choisi, z_face, "Poche STEP auto-selectionnee"
        )

        operations_poche.append({
            "z": z_face, "segments": segments, "label": "FACE STEP",
            "outil": outil_choisi,
            "desc": (
                f"Outil T{outil_choisi.tool_number} "
                f"(D{outil_choisi.tool_diameter:.1f}) selectionne automatiquement "
                f"pour la poche STEP de largeur {largeur_face:.1f}mm ({raison})"
            ),
        })

    # ------------------------------------------------------------------
    # 3. PERCAGE : chaque trou -> contour de poche circulaire independant,
    #    fraise en colimacon (anneaux concentriques) comme les faces planes
    #    ci-dessus. Aucun foret / cycle canne G81-G83. Outil choisi
    #    AUTOMATIQUEMENT par trou (largeur = diametre du trou) : exactitude
    #    de diametre en priorite, sinon le plus grand outil du magasin
    #    strictement sous le diametre, sinon config.tool_poche().
    # ------------------------------------------------------------------
    for trou in features_3d.get("percages", []):
        z_fond = round(trou["z_depart"] + trou["profondeur"], 4)
        segments = _cercle_vers_segments(trou["x"], trou["y"], trou["rayon"])
        diametre_trou = 2.0 * trou["rayon"]

        outil_choisi, raison = _choisir_outil_forme(diametre_trou, config)
        _verifier_mandrin_outil_auto(
            outil_choisi, z_fond, "Percage STEP auto-selectionne"
        )

        operations_poche.append({
            "z": z_fond, "segments": segments, "label": "PERCAGE STEP",
            "outil": outil_choisi,
            # cercle : declenche la generation en arcs natifs G02 (helice de
            # descente + anneaux circulaires) au lieu de polygones G01.
            "cercle": (trou["x"], trou["y"], trou["rayon"]),
            "desc": (
                f"Outil T{outil_choisi.tool_number} "
                f"(D{outil_choisi.tool_diameter:.1f}) selectionne automatiquement "
                f"pour le percage STEP de D{diametre_trou:.1f}mm ({raison})"
            ),
        })

    # Vue agregee par profondeur -- uniquement pour le Bouclier de Securite et
    # l'entete (comptages, profondeur max vs longueur d'outil) : la fusion par
    # cle Z y est sans consequence, ces fonctions ne lisent jamais le contour.
    poche_par_z: dict[float, list[tuple]] = defaultdict(list)
    for op in operations_poche:
        poche_par_z[op["z"]].extend(op["segments"])

    # drill_par_z : aucun percage n'est perce au foret dans ce flux STEP ;
    # conserve (vide) uniquement pour la signature partagee avec le flux DXF
    # (_valider_geometrie_et_config / _entete_global).
    drill_par_z: dict[float, list[dict]] = {}

    # --- Securites physiques (meme bouclier que le flux DXF) ---
    # verifier_collision_brut suit inclure_contournage : seul le detourage de
    # la silhouette descend jusqu'au bas du brut ; les faces secondaires ont
    # leurs profondeurs verifiees operation par operation.
    _valider_geometrie_et_config(
        [], config, contour_par_z, poche_par_z, drill_par_z,
        verifier_collision_brut=inclure_contournage,
    )

    # --- Construction du programme (identique a generer_programme_complet) ---
    sections:     list[str] = []
    outil_actuel: int | None = None

    def _changer_outil_si_besoin(numero: int) -> list[str]:
        nonlocal outil_actuel
        if outil_actuel == numero:
            return []
        outil_actuel = numero
        return post.changement_outil(
            config.magazine.require(numero), config.safety_z
        )

    pre = post.pre_programme()
    entete = _entete_global(config, post, contour_par_z, poche_par_z, drill_par_z)
    if pre:
        sections.append("\n".join(pre) + "\n\n" + entete)
    else:
        sections.append(entete)

    sections.append(post.commentaire(
        "SOURCE : piece 3D (fichier STEP) -- contours et poches traduits "
        "automatiquement depuis le maillage"
    ))

    for texte in (commentaires_entete or []):
        sections.append(post.commentaire(texte))

    # --- Contournage ---
    for z in sorted(contour_par_z.keys(), key=abs):
        lignes  = contour_par_z[z]
        t       = tool_contour
        paliers = _calculer_paliers_z(z, config.default_pass_depth)
        blocs: list[str] = []
        blocs += _changer_outil_si_besoin(t.tool_number)
        blocs.append(post.commentaire(
            f"Outil T{t.tool_number} (D{t.tool_diameter:.1f}) selectionne "
            f"automatiquement pour le contournage ({raison_contour})"
        ))
        blocs.append(post.commentaire(
            f"=== CONTOURNAGE (BRUT STEP)  T{t.tool_number} (D{t.tool_diameter:.1f})"
            f"  Z_cible={_coord(z)}mm  {len(paliers)} passe(s) ==="
        ))
        blocs += _segments_multi_passes(
            calculer_offset_contour(lignes, rayon_contour, "exterieur"),
            paliers, t.feedrate, t.plunge_feedrate, tool=t,
        )
        sections.append("\n".join(blocs))

    # --- Pocketing (une operation independante par face/trou, cf. note ci-dessus) ---
    for indice, op in enumerate(
        sorted(operations_poche, key=lambda o: (abs(o["z"]), o["label"], o["segments"][0][0]))
    ):
        z, lignes, label = op["z"], op["segments"], op["label"]
        t           = op["outil"]
        rayon_outil = t.tool_diameter / 2.0
        paliers     = _calculer_paliers_z(z, config.default_pass_depth)

        # --- Percage circulaire : arcs natifs G02 au lieu de polygones G01 ---
        if op.get("cercle") is not None:
            cx, cy, rayon_trou = op["cercle"]
            blocs: list[str] = []
            blocs += _changer_outil_si_besoin(t.tool_number)
            blocs.append(post.commentaire(op["desc"]))
            rayon_max_anneau = rayon_trou - rayon_outil
            if rayon_max_anneau <= 0.05:
                # Outil au diametre du trou : helice impossible (rayon nul),
                # percage au cycle G83 (debourrage) -- l'usage prevu d'une
                # correspondance exacte foret/trou.
                blocs.append(post.commentaire(
                    f"=== PERCAGE CYCLE ({label} #{indice})  T{t.tool_number} "
                    f"(D{t.tool_diameter:.1f})  Z_cible={_coord(z)}mm ==="
                ))
                blocs += _cycles_percage(
                    [{"center": (cx, cy), "radius": rayon_trou}],
                    z, t.plunge_feedrate, config.peck_depth, tool=t,
                )
            else:
                rayons = _rayons_anneaux(rayon_max_anneau, rayon_outil,
                                         config.stepover_poche)
                blocs.append(post.commentaire(
                    f"=== FRAISAGE CIRCULAIRE G02 ({label} #{indice})  "
                    f"T{t.tool_number} (D{t.tool_diameter:.1f})  "
                    f"Z_cible={_coord(z)}mm  {len(paliers)} passe(s)  "
                    f"{len(rayons)} anneau(x) ==="
                ))
                blocs += _passes_poche_circulaire(
                    cx, cy, rayons, paliers, t.feedrate, t.plunge_feedrate, tool=t,
                )
            sections.append("\n".join(blocs))
            continue

        try:
            trajectoires = calculer_trajectoires_poche(
                lignes, rayon_outil, config.stepover_poche
            )
        except RuntimeError as exc:
            sections.append(post.commentaire(
                f"[{label} #{indice} Z={z}] IGNOREE : {exc}"
            ) + "\n")
            continue

        if not trajectoires:
            # calculer_trajectoires_poche() renvoie [] (sans lever d'exception)
            # quand le contour est trop petit pour l'outil -- sans ce garde-fou,
            # la section serait generee "vide" (juste un changement d'outil et
            # un commentaire), sans aucun mouvement G01 reel, silencieusement.
            sections.append(post.commentaire(
                f"[{label} #{indice} Z={z}] IGNOREE : contour trop petit pour l'outil "
                f"T{t.tool_number} (D{t.tool_diameter:.1f}mm) -- "
                f"choisissez une fraise de pocketing plus fine"
            ) + "\n")
            continue

        blocs: list[str] = []
        blocs += _changer_outil_si_besoin(t.tool_number)
        blocs.append(post.commentaire(op["desc"]))
        blocs.append(post.commentaire(
            f"=== POCKETING ({label} #{indice})  T{t.tool_number} (D{t.tool_diameter:.1f})"
            f"  Z_cible={_coord(z)}mm  {len(paliers)} passe(s)"
            f"  {len(trajectoires)} anneau(x) ==="
        ))
        blocs += _passes_poche(
            trajectoires, paliers, t.feedrate, t.plunge_feedrate, tool=t,
        )
        sections.append("\n".join(blocs))

    sections.append("\n".join(post.pied_programme(config.safety_z)))

    if corps_seulement:
        # Corps reutilisable pour l'assemblage 4 axes : sans pre-programme /
        # en-tete global (sections[0]) ni pied de programme (dernier element).
        return "\n\n".join(sections[1:-1])

    return "\n\n".join(sections)


def _rayons_anneaux(
    rayon_max: float, rayon_outil: float, stepover_ratio: float,
) -> list[float]:
    """Rayons des anneaux concentriques pour vider un cercle, de l'exterieur
    vers le centre. Garantit que le centre est couvert : le dernier anneau
    est toujours a un rayon <= rayon_outil (le flanc de l'outil balaie alors
    le centre du trou).
    """
    pas = 2.0 * rayon_outil * stepover_ratio
    rayons: list[float] = []
    r = rayon_max
    while r > 0.05:
        rayons.append(round(r, 3))
        r -= pas
    if rayons and rayons[-1] > rayon_outil:
        rayons.append(round(rayon_outil * 0.5, 3))
    return rayons


def _passes_poche_circulaire(
    cx: float, cy: float,
    rayons: list[float],
    paliers_z: list[float],
    feedrate: float,
    plunge_feedrate: float,
    tool: ToolParams | None = None,
) -> list[str]:
    """Fraisage d'un trou en arcs natifs G02 : pour chaque palier Z et chaque
    anneau, descente en HELICE (G02 avec interpolation Z sur un tour complet)
    puis tour de finition a plat. Zero plongee verticale, zero segment
    polygonal : le trou est un vrai cercle pour le controleur (G17 requis,
    present dans l'initialisation de tous les post-processeurs).
    """
    if not rayons or not paliers_z:
        return []

    outil_desc = f"T{tool.tool_number} (D{tool.tool_diameter:.1f})" if tool else "?"
    blocs: list[str] = []

    for pi, z in enumerate(paliers_z):
        blocs.append(
            f"; --- PASSE CIRCULAIRE {pi+1}/{len(paliers_z)}"
            f" - OUTIL {outil_desc} - Z={_coord(z)} ---"
        )
        for ai, r in enumerate(rayons):
            x_dep = cx + r
            blocs.append(f"; Anneau {ai+1}/{len(rayons)}  rayon={_coord(r)}")
            blocs.append(f"G00 Z{_coord(_SAFETY_Z)}")
            blocs.append(f"G00 X{_coord(x_dep)} Y{_coord(cy)}")
            blocs.append(
                f"G02 X{_coord(x_dep)} Y{_coord(cy)}"
                f" I{_coord(-r)} J0.000 Z{_coord(z)} F{_coord(plunge_feedrate)}"
            )
            blocs.append(
                f"G02 X{_coord(x_dep)} Y{_coord(cy)}"
                f" I{_coord(-r)} J0.000 F{_coord(feedrate)}"
            )
        if pi == len(paliers_z) - 1:
            blocs.append(f"G00 Z{_coord(_SAFETY_Z)}")
            blocs.append("")

    return blocs


# ===========================================================================
# GENERATION MULTI-FACES (3 axes : un fichier par face / 4 axes : unifie)
# ===========================================================================

# Angle de l'axe rotatif A (autour de X) presentant chaque face a la broche.
# Les faces X_PLUS / X_MINUS sont hors de portee d'un rotatif d'axe X :
# elles necessitent un retournement manuel (M00) meme en mode 4 axes.
_ANGLES_AXE_A: dict[str, int] = {
    "Z_PLUS": 0, "Y_PLUS": 90, "Z_MINUS": 180, "Y_MINUS": 270,
}
_ORDRE_ORIENTATIONS: list[str] = [
    "Z_PLUS", "Y_PLUS", "Z_MINUS", "Y_MINUS", "X_PLUS", "X_MINUS",
]


def generer_gcodes_multi_faces(
    features_par_orientation: dict[str, dict],
    config: MachiningConfig,
    mode: str = "3_axes",
) -> dict[str, str]:
    """Genere le(s) programme(s) G-code pour une piece multi-faces.

    Args:
        features_par_orientation: Retour de
            step_parser.analyser_features_multi_faces : pour chaque
            orientation ayant des features, les faces/percages exprimes
            dans le repere de cette face + les dimensions du brut vu
            sous cette orientation.
        config: Configuration d'usinage (le stock_dimensions est remplace
            par celui de chaque orientation, la config n'est pas modifiee).
        mode: "3_axes" -> un programme complet PAR face (l'operateur
              retourne physiquement la piece entre les fichiers) ;
              "4_axes" -> un programme UNIQUE, la piece pivotee par l'axe
              rotatif A (G00 A90/A180/A270) entre les sections.

    Returns:
        (programmes, erreurs) :
          programmes -- en 3 axes, un couple {suffixe: gcode} par face
            usinable (ex : "Z_PLUS", "X_PLUS") ; en 4 axes, un seul couple
            "4_AXES_A".
          erreurs -- {suffixe: message} pour les faces dont la generation a
            ete BLOQUEE par le Bouclier de Securite (ex : trou lateral plus
            profond que la longueur utile de tous les outils). Une face en
            erreur n'empeche pas la generation des autres.
    """
    if mode == "4_axes":
        return (
            {"4_AXES_A": _generer_gcode_4_axes(features_par_orientation, config)},
            {},
        )

    resultats: dict[str, str] = {}
    erreurs: dict[str, str] = {}
    for nom in _ORDRE_ORIENTATIONS:
        data = features_par_orientation.get(nom)
        if data is None:
            continue
        dims = data["dimensions"]
        cfg = replace(
            config, stock_dimensions=(dims["x"], dims["y"], dims["z"])
        )
        commentaires = None
        if nom != "Z_PLUS":
            commentaires = [
                f"OPERATION RETOURNEMENT PIECE : placer la face {nom} "
                f"vers le haut de la machine",
                "Origine piece a re-referencer : coin inferieur gauche de la "
                "face presentee = X0 Y0, dessus de la face = Z0",
            ]
        try:
            resultats[nom] = generer_gcode_depuis_step(
                data, cfg,
                inclure_contournage=(nom == "Z_PLUS"),
                commentaires_entete=commentaires,
            )
        except SecritePhysiqueError as exc:
            erreurs[nom] = str(exc)
    return resultats, erreurs


def _generer_gcode_4_axes(
    features_par_orientation: dict[str, dict],
    config: MachiningConfig,
) -> str:
    """Assemble un programme 4 axes indexe unique : entre chaque face, l'axe
    rotatif A (autour de X) pivote la piece devant la broche (G00 A...).

    Mode INDEXE (3+1) : chaque section usine a plat apres rotation ; les
    coordonnees de chaque section sont exprimees dans le repere de la face
    presentee (l'origine piece est re-referencee par le decalage travail de
    la machine, pratique standard du 3+1).
    """
    post = get_post_processeur(config.machine_type)
    dims_top = features_par_orientation["Z_PLUS"]["dimensions"]

    sections: list[str] = []
    entete = [
        post.commentaire("============================================================"),
        post.commentaire("PROGRAMME 4 AXES (indexe -- axe rotatif A autour de X)"),
        post.commentaire(
            f"Brut : {dims_top['x']:.0f} x {dims_top['y']:.0f} "
            f"x {dims_top['z']:.0f} mm"
        ),
        post.commentaire(
            "Chaque section pivote la piece (G00 A...) puis usine la face presentee"
        ),
        post.commentaire(
            "ATTENTION : origine piece re-referencee sur chaque face (mode indexe)"
        ),
        post.commentaire("============================================================"),
    ] + post.initialisation() + [
        f"G00 Z{_coord(config.safety_z)}",
        "G00 A0",
    ]
    pre = post.pre_programme()
    if pre:
        sections.append("\n".join(pre) + "\n\n" + "\n".join(entete))
    else:
        sections.append("\n".join(entete))

    for nom in _ORDRE_ORIENTATIONS:
        data = features_par_orientation.get(nom)
        if data is None:
            continue
        dims = data["dimensions"]
        cfg = replace(
            config, stock_dimensions=(dims["x"], dims["y"], dims["z"])
        )

        bloc = [
            post.commentaire(f"================ FACE {nom} ================"),
            f"G00 Z{_coord(config.safety_z)}",
        ]
        angle = _ANGLES_AXE_A.get(nom)
        if angle is not None:
            bloc.append(f"G00 A{angle}")
            bloc.append(post.commentaire(
                f"Axe A pivote a {angle} deg : face {nom} presentee a la broche"
            ))
        else:
            bloc.append(post.commentaire(
                f"FACE {nom} HORS PORTEE DE L'AXE A (rotatif autour de X) :"
            ))
            bloc.append(post.commentaire(
                "retournement MANUEL requis -- reprendre l'origine puis CYCLE START"
            ))
            bloc.append("M00")

        try:
            corps = generer_gcode_depuis_step(
                data, cfg,
                inclure_contournage=(nom == "Z_PLUS"),
                corps_seulement=True,
            )
        except SecritePhysiqueError as exc:
            avert = [post.commentaire(
                f"FACE {nom} NON GENEREE -- BLOQUEE PAR LE BOUCLIER DE SECURITE :"
            )]
            avert += [post.commentaire(f"  {l}") for l in str(exc).splitlines()]
            sections.append("\n".join(bloc) + "\n\n" + "\n".join(avert))
            continue
        sections.append("\n".join(bloc) + "\n\n" + corps)

    sections.append(
        "G00 A0\n" + "\n".join(post.pied_programme(config.safety_z))
    )
    return "\n\n".join(sections)


# ===========================================================================
# FONCTIONS PUBLIQUES UNITAIRES (compatibilite ascendante)
# ===========================================================================

def generer_gcode_iso(
    liste_lignes: list[tuple],
    target_depth: float,
    feedrate: float,
    tool: ToolParams | None = None,
    plunge_feedrate: float | None = None,
    rayon_outil: float = 0.0,
    cote: CoteOffset = "exterieur",
    pass_depth: float | None = None,
) -> str:
    if target_depth >= 0:
        raise ValueError(f"target_depth doit etre negatif (recu : {target_depth}).")
    if feedrate <= 0:
        raise ValueError(f"feedrate doit etre positif (recu : {feedrate}).")
    lignes   = (
        calculer_offset_contour(liste_lignes, rayon_outil, cote)
        if rayon_outil > 0.0 else liste_lignes
    )
    avance_p = plunge_feedrate if plunge_feedrate is not None else feedrate
    prof_p   = pass_depth if pass_depth is not None else abs(target_depth)
    paliers  = _calculer_paliers_z(target_depth, prof_p)
    post     = PostProcesseurISO()
    blocs    = _entete_detourage(lignes, target_depth, feedrate, tool, rayon_outil, cote, paliers)
    blocs   += _segments_multi_passes(lignes, paliers, feedrate, avance_p, tool=tool)
    blocs   += post.pied_programme(_SAFETY_Z)
    return "\n".join(blocs)


def generer_gcode_percage(
    liste_cercles: list[dict],
    target_depth: float,
    plunge_feedrate: float,
    tool: ToolParams | None = None,
    peck_depth: float | None = None,
) -> str:
    if not liste_cercles:
        raise ValueError("liste_cercles ne peut pas etre vide.")
    if target_depth >= 0:
        raise ValueError(f"target_depth doit etre negatif (recu : {target_depth}).")
    post  = PostProcesseurISO()
    blocs = _entete_percage(liste_cercles, target_depth, plunge_feedrate, tool, peck_depth)
    blocs += _cycles_percage(liste_cercles, target_depth, plunge_feedrate, peck_depth, tool=tool)
    blocs += post.pied_programme(_SAFETY_Z)
    return "\n".join(blocs)


def generer_gcode_poche_complete(
    liste_lignes: list[tuple],
    target_depth: float,
    feedrate: float,
    rayon_outil: float,
    tool: ToolParams | None = None,
    plunge_feedrate: float | None = None,
    pass_depth: float | None = None,
    stepover_ratio: float = 0.5,
) -> str:
    if target_depth >= 0:
        raise ValueError(f"target_depth doit etre negatif (recu : {target_depth}).")
    if feedrate <= 0:
        raise ValueError(f"feedrate doit etre positif (recu : {feedrate}).")
    if rayon_outil <= 0:
        raise ValueError(f"rayon_outil doit etre positif (recu : {rayon_outil}).")
    avance_p     = plunge_feedrate if plunge_feedrate is not None else feedrate
    prof_p       = pass_depth if pass_depth is not None else abs(target_depth)
    paliers      = _calculer_paliers_z(target_depth, prof_p)
    trajectoires = calculer_trajectoires_poche(liste_lignes, rayon_outil, stepover_ratio)
    if not trajectoires:
        raise RuntimeError(f"Poche trop petite pour un outil de rayon {rayon_outil} mm.")
    post  = PostProcesseurISO()
    blocs = _entete_poche(trajectoires, target_depth, feedrate, tool, rayon_outil, stepover_ratio, paliers)
    blocs += _passes_poche(trajectoires, paliers, feedrate, avance_p, tool=tool)
    blocs += post.pied_programme(_SAFETY_Z)
    return "\n".join(blocs)


# ===========================================================================
# PARSING DU NOM DE CALQUE
# ===========================================================================

_RE_Z = re.compile(r'[_\-]Z(-?\d+(?:[.,]\d+)?)', re.IGNORECASE)
_PREFIXES_POCHE = ("POCHE", "POCHE_INT", "POCKET")


def _parse_z_from_layer(layer: str, default: float) -> float:
    """Extrait la profondeur Z encodee dans le nom du calque DXF.
    'Contour_Ext_Z-8' -> -8.0  |  'CONTOUR' -> default
    Toujours retournee negative.
    """
    m = _RE_Z.search(layer)
    if m:
        val = float(m.group(1).replace(",", "."))
        return val if val <= 0 else -val
    return default


def _est_calque_poche(layer: str) -> bool:
    """True si le calque est un calque de poche (POCHE / POCHE_INT / POCKET)."""
    nom = _RE_Z.sub("", layer).strip("_").upper()
    return any(nom == p or nom.startswith(p) for p in _PREFIXES_POCHE)


# ===========================================================================
# UTILITAIRES
# ===========================================================================

def _coord(v: float) -> str:
    return f"{v:.3f}"


def _calculer_paliers_z(target_depth: float, pass_depth: float) -> list[float]:
    """[target=-5, pass=2] -> [-2.000, -4.000, -5.000]"""
    nb = math.ceil(abs(target_depth) / pass_depth)
    return [max(-i * pass_depth, target_depth) for i in range(1, nb + 1)]


# ===========================================================================
# DECOMPOSITION EN CONTOURS CHAINES
# ===========================================================================

def _segments_sont_chaines(a: tuple, b: tuple, tol: float = 0.01) -> bool:
    _, _, x2, y2 = a
    x1, y1, _, _ = b
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 <= tol


def _decomposer_en_contours(
    liste_lignes: list[tuple], tolerance: float = 0.01
) -> list[list[tuple]]:
    if not liste_lignes:
        return []
    contours: list[list[tuple]] = []
    courant = [liste_lignes[0]]
    for i in range(1, len(liste_lignes)):
        if _segments_sont_chaines(liste_lignes[i - 1], liste_lignes[i], tolerance):
            courant.append(liste_lignes[i])
        else:
            contours.append(courant)
            courant = [liste_lignes[i]]
    contours.append(courant)
    return contours


# ===========================================================================
# EN-TETE GLOBAL DU PROGRAMME
# ===========================================================================

def _entete_global(
    config: MachiningConfig,
    post: PostProcesseur,
    contour_par_z: dict,
    poche_par_z:   dict,
    drill_par_z:   dict,
) -> str:
    def _info(t: ToolParams | None) -> str:
        return (
            f"T{t.tool_number}  D{t.tool_diameter:.1f}mm  "
            f"L={t.tool_length:.0f}mm  S{t.spindle_speed}tr/min  F{t.feedrate:.0f}mm/min"
            if t else "non defini"
        )

    t_c = config.magazine.get(config.tool_number_contour)
    t_p = config.magazine.get(config.tool_number_poche)
    t_d = config.magazine.get(config.tool_number_drill)

    nb_cont  = sum(len(v) for v in contour_par_z.values())
    nb_poche = sum(len(v) for v in poche_par_z.values())
    nb_drill = sum(len(v) for v in drill_par_z.values())

    lignes = [
        "; ============================================================",
        f"; Programme    : MonLogicielFAO v2 -- {post.label}",
        f"; Materiau     : {config.material_type}",
        f"; Table        : X={config.cnc_limit_x:.0f}mm  Y={config.cnc_limit_y:.0f}mm",
        f"; Securite Z   : {_coord(config.safety_z)} mm",
        f"; Passe Z def. : {_coord(config.default_pass_depth)} mm",
        "; ------------------------------------------------------------",
        f"; Contournage  : {nb_cont} segments  {len(contour_par_z)} niveau(x) Z",
        f"; Pocketing    : {nb_poche} segments  {len(poche_par_z)} niveau(x) Z",
        f"; Percage      : {nb_drill} trou(s)   {len(drill_par_z)} niveau(x) Z",
        "; ------------------------------------------------------------",
        f"; T contour    : {_info(t_c)}",
        f"; T poche      : {_info(t_p)}",
        f"; T percage    : {_info(t_d)}",
        "; ============================================================",
        "",
    ] + post.initialisation() + [
        f"G00 Z{_coord(config.safety_z)}",
        "",
    ]
    return "\n".join(lignes)


# ===========================================================================
# EN-TETES POUR FONCTIONS UNITAIRES (legacy)
# ===========================================================================

def _entete_detourage(lignes, target_depth, feedrate, tool, rayon_outil, cote, paliers) -> list[str]:
    paliers_str = "  ".join(_coord(z) for z in paliers)
    info_outil = (
        f"; Outil      : T{tool.tool_number} -- D{tool.tool_diameter:.1f}mm  L={tool.tool_length:.0f}mm"
        if tool else "; Outil      : non defini"
    )
    return [
        "; ============================================================",
        "; Section      : Detourage",
        f"; Segments     : {len(lignes)}",
        f"; Prof. cible  : Z{_coord(target_depth)} mm",
        f"; Avance XY    : F{_coord(feedrate)} mm/min",
        info_outil,
        f"; Passes Z     : {len(paliers)} -> [{paliers_str}] mm",
        "; ============================================================",
        "G21", "G90", "G94",
    ] + ([f"T{tool.tool_number} M06", f"M03 S{tool.spindle_speed}"] if tool else []) + [
        f"G00 Z{_coord(_SAFETY_Z)}",
    ]


def _entete_percage(liste_cercles, target_depth, plunge_feedrate, tool, peck_depth) -> list[str]:
    cycle = "G83 (debourrage)" if peck_depth else "G81 (passe unique)"
    info_outil = (
        f"; Outil      : T{tool.tool_number} -- D{tool.tool_diameter:.1f}mm  L={tool.tool_length:.0f}mm"
        if tool else "; Outil      : non defini"
    )
    return [
        "; ============================================================",
        "; Section      : Percage",
        f"; Trous        : {len(liste_cercles)}",
        f"; Prof. cible  : Z{_coord(target_depth)} mm",
        f"; Avance Z     : F{_coord(plunge_feedrate)} mm/min",
        f"; Cycle        : {cycle}",
        info_outil,
        "; ============================================================",
        "G21", "G90", "G94",
    ] + ([f"T{tool.tool_number} M06", f"M03 S{tool.spindle_speed}"] if tool else []) + [
        f"G00 Z{_coord(_SAFETY_Z)}",
    ]


def _entete_poche(trajectoires, target_depth, feedrate, tool, rayon_outil, stepover, paliers) -> list[str]:
    paliers_str = "  ".join(_coord(z) for z in paliers)
    info_outil = (
        f"; Outil      : T{tool.tool_number} -- D{tool.tool_diameter:.1f}mm  L={tool.tool_length:.0f}mm"
        if tool else "; Outil      : non defini"
    )
    return [
        "; ============================================================",
        "; Section      : Pocketing",
        f"; Anneaux      : {len(trajectoires)}",
        f"; Prof. cible  : Z{_coord(target_depth)} mm",
        f"; Avance XY    : F{_coord(feedrate)} mm/min",
        f"; Stepover     : {stepover*100:.0f}%  Rayon : {_coord(rayon_outil)} mm",
        f"; Passes Z     : {len(paliers)} -> [{paliers_str}] mm",
        info_outil,
        "; ============================================================",
        "G21", "G90", "G94",
    ] + ([f"T{tool.tool_number} M06", f"M03 S{tool.spindle_speed}"] if tool else []) + [
        f"G00 Z{_coord(_SAFETY_Z)}",
    ]


# ===========================================================================
# CORPS DETOURAGE -- MULTI-PASSES Z
# ===========================================================================

def _segments_multi_passes(
    liste_lignes: list[tuple],
    paliers_z:    list[float],
    feedrate:     float,
    plunge_feedrate: float,
    tool: ToolParams | None = None,
) -> list[str]:
    """Detourage contour-first avec entree en rampe sur le premier segment.

    Strategie de plongee (identique au pocketing) :
      La plongee verticale G01 Z est bannie : elle creerait un trou non prevu
      et userait la pointe de la fraise sans avance radiale.
      A la place, le premier segment de chaque passe Z est parcouru en
      interpolation XY+Z simultanee (rampe douce) :
        - Depart  XY : premier point du contour, Z = niveau precedent (ou safety)
        - Arrivee XY : deuxieme point du contour, Z = niveau cible de la passe
      Les segments suivants s'executent a plat a la profondeur atteinte.
      Entre deux passes du MEME contour : repositionnement XY rapide sans lever Z
      (on reste dans le sillon deja usine, economie de temps).
      Remontee au safety_z uniquement apres la derniere passe.
    """
    if not liste_lignes:
        return ["; (Aucune geometrie a tracer)"]

    contours   = _decomposer_en_contours(liste_lignes)
    nb_passes  = len(paliers_z)
    nb_cont    = len(contours)
    outil_desc = f"T{tool.tool_number} (D{tool.tool_diameter:.1f})" if tool else "?"
    blocs: list[str] = []

    for ci, contour in enumerate(contours):
        if len(contour) < 1:
            continue

        # Point de depart de la rampe = debut du premier segment
        x_ramp_dep, y_ramp_dep = contour[0][0], contour[0][1]
        # Fin de la rampe = fin du premier segment (le segment est "consomme")
        x_ramp_fin, y_ramp_fin = contour[0][2], contour[0][3]

        for pi, z in enumerate(paliers_z):
            est_premiere = pi == 0
            est_derniere = pi == nb_passes - 1
            z_precedent  = paliers_z[pi - 1] if pi > 0 else _SAFETY_Z

            blocs.append(
                f"; --- CONTOURNAGE - OUTIL {outil_desc}"
                f" - Contour {ci+1}/{nb_cont}"
                f" - PASSE {pi+1}/{nb_passes}"
                f" - Z={_coord(z)} (rampe) ---"
            )
            if tool:
                blocs.append(
                    f"; Rampe : F{tool.plunge_feedrate:.0f}  XY : F{tool.feedrate:.0f}"
                )

            # Positionnement rapide au depart de la rampe
            if est_premiere:
                # Separer Z et XY : evite le G00 oblique 3D depuis position inconnue
                blocs.append(f"G00 Z{_coord(_SAFETY_Z)}")
                blocs.append(f"G00 X{_coord(x_ramp_dep)} Y{_coord(y_ramp_dep)}")
            else:
                # On est deja dans le sillon, simple repositionnement XY
                blocs.append(
                    f"G00 X{_coord(x_ramp_dep)} Y{_coord(y_ramp_dep)}"
                )

            # Rampe : descente Z + avance XY simultanees sur le 1er segment
            blocs.append(
                f"; Rampe : Z{_coord(z_precedent)} -> Z{_coord(z)}"
                f"  sur ({_coord(x_ramp_dep)},{_coord(y_ramp_dep)})"
                f" -> ({_coord(x_ramp_fin)},{_coord(y_ramp_fin)})"
            )
            blocs.append(
                f"G01 X{_coord(x_ramp_fin)} Y{_coord(y_ramp_fin)}"
                f" Z{_coord(z)} F{_coord(plunge_feedrate)}"
            )

            # Segments restants (1er segment consomme par la rampe)
            for x1, y1, x2, y2 in contour[1:]:
                blocs.append(f"G01 X{_coord(x2)} Y{_coord(y2)} F{_coord(feedrate)}")

            if est_derniere:
                blocs.append(f"G00 Z{_coord(_SAFETY_Z)}")
                blocs.append("")

    return blocs


# ===========================================================================
# CORPS PERCAGE -- G83 / G81
# ===========================================================================

def _cycles_percage(
    liste_cercles:  list[dict],
    target_depth:   float,
    plunge_feedrate: float,
    peck_depth:     float | None,
    tool: ToolParams | None = None,
) -> list[str]:
    """Genere G83 (debourrage) ou G81 (passe unique). G80 apres le dernier trou."""
    blocs: list[str] = []
    nb = len(liste_cercles)
    cycle = "G83 DEBOURRAGE" if peck_depth is not None else "G81 PASSE UNIQUE"
    outil_desc = f"T{tool.tool_number} (D{tool.tool_diameter:.1f})" if tool else "?"
    blocs.append(f"; Cycle : {cycle}  Outil : {outil_desc}  Z_fond : {_coord(target_depth)}")

    for idx, cercle in enumerate(liste_cercles):
        cx = cercle["center"][0]
        cy = cercle["center"][1]
        d  = cercle["radius"] * 2.0
        blocs.append(
            f"; Trou {idx+1:03d}/{nb:03d}"
            f" -- centre ({_coord(cx)}, {_coord(cy)}) -- D{d:.2f}mm"
        )
        blocs.append(f"G00 X{_coord(cx)} Y{_coord(cy)} Z{_coord(_SAFETY_Z)}")
        if peck_depth is not None:
            blocs.append(
                f"G83 X{_coord(cx)} Y{_coord(cy)} Z{_coord(target_depth)}"
                f" R{_coord(_SAFETY_Z)} Q{_coord(peck_depth)} F{_coord(plunge_feedrate)}"
            )
        else:
            blocs.append(
                f"G81 X{_coord(cx)} Y{_coord(cy)} Z{_coord(target_depth)}"
                f" R{_coord(_SAFETY_Z)} F{_coord(plunge_feedrate)}"
            )
    blocs += ["G80", f"G00 Z{_coord(_SAFETY_Z)}", ""]
    return blocs


# ===========================================================================
# CORPS POCKETING -- RAMPE D'ENTREE + ANNEAUX CONCENTRIQUES
# ===========================================================================

def _passes_poche(
    trajectoires:    list[list[tuple]],
    paliers_z:       list[float],
    feedrate:        float,
    plunge_feedrate: float,
    tool: ToolParams | None = None,
) -> list[str]:
    """Pocketing Z-level-first. Chaque anneau a sa propre remontee et rampe.

    REGLE D'OR : l'outil ne se deplace JAMAIS lateralement a Z negatif.
    Sequence invariable pour CHAQUE anneau (exterieur et interieurs) :
      1. G00 Z{safety}           -- remontee au plan de securite
      2. G00 X{dep} Y{dep}       -- positionnement XY rapide en air
      3. G01 X{fin} Y{fin} Z{z}  -- rampe douce (XYZ simultanes)
      4. G01 X Y ...             -- fraisage du reste de l'anneau a Z constant

    Cette strategie elimine :
      - Les G00 obliques 3D (XYZ simultanes depuis la position precedente)
      - Les deplacements lateraux G00 a Z < 0 entre anneaux
      - Les plongees verticales G01 Z isolees
    """
    if not trajectoires or not paliers_z:
        return []

    nb_paliers = len(paliers_z)
    nb_anneaux = len(trajectoires)
    outil_desc = f"T{tool.tool_number} (D{tool.tool_diameter:.1f})" if tool else "?"
    blocs: list[str] = []

    for pi, z in enumerate(paliers_z):
        est_derniere = (pi == nb_paliers - 1)

        blocs.append(
            f"; --- PASSE POCHE {pi+1}/{nb_paliers}"
            f" - OUTIL {outil_desc} - Z={_coord(z)} ---"
        )
        if tool:
            blocs.append(
                f"; Rampe F{tool.plunge_feedrate:.0f}"
                f"  XY F{tool.feedrate:.0f}"
                f"  {nb_anneaux} anneau(x)"
            )

        for ai, anneau in enumerate(trajectoires):
            if len(anneau) < 1:
                continue

            role  = "Ext" if ai == 0 else f"{ai+1}/{nb_anneaux}"
            x_dep = anneau[0][0]
            y_dep = anneau[0][1]
            x_fin = anneau[0][2]
            y_fin = anneau[0][3]

            blocs.append(f"; Anneau {role}")
            # 1. Remontee au Z de securite (evite tout deplacement lateral en matiere)
            blocs.append(f"G00 Z{_coord(_SAFETY_Z)}")
            # 2. Positionnement XY en l'air
            blocs.append(f"G00 X{_coord(x_dep)} Y{_coord(y_dep)}")
            # 3. Rampe XYZ simultanee : descente douce sur le premier segment
            blocs.append(
                f"; Rampe : Z{_coord(_SAFETY_Z)} -> Z{_coord(z)}"
            )
            blocs.append(
                f"G01 X{_coord(x_fin)} Y{_coord(y_fin)}"
                f" Z{_coord(z)} F{_coord(plunge_feedrate)}"
            )
            # 4. Fraisage des segments restants a Z constant
            for x1, y1, x2, y2 in anneau[1:]:
                blocs.append(f"G01 X{_coord(x2)} Y{_coord(y2)} F{_coord(feedrate)}")

        # Remontee finale apres la derniere passe
        if est_derniere:
            blocs.append(f"; Fin de poche")
            blocs.append(f"G00 Z{_coord(_SAFETY_Z)}")
            blocs.append("")

    return blocs
