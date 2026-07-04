"""
geometry.py -- Moteur de calcul geometrique pour la FAO.

Responsabilite unique : calculs purement mathematiques sur les entites
(longueurs, offsets, pocketing). Aucune dependance a l'UI ni au G-code.
"""

from __future__ import annotations

import math
from typing import Literal

from shapely.geometry import LineString, Polygon
from shapely.geometry.base import BaseGeometry


# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

CoteOffset = Literal["exterieur", "interieur"]

_TOLERANCE_CHAINING: float = 0.01


# ---------------------------------------------------------------------------
# API publique -- Offset de contour (detourage)
# ---------------------------------------------------------------------------

def calculer_offset_contour(
    liste_lignes: list[tuple],
    rayon_outil: float,
    cote: CoteOffset,
) -> list[tuple]:
    """Calcule la trajectoire outil compensee en rayon pour un contour 2D.

    Reconstruit les polylignes, applique un decalage via Shapely et retourne
    la nouvelle liste de segments prete pour le generateur G-code.

    Convention :
      'exterieur' -> offset positif (contour agrandi, outil tourne dehors).
      'interieur' -> offset negatif (contour retreci, outil usine dedans).

    Args:
        liste_lignes: Segments (x1, y1, x2, y2) issus du parseur DXF.
        rayon_outil:  Rayon outil en mm (positif). 0 = pas d'offset.
        cote:         'exterieur' ou 'interieur'.

    Returns:
        Liste de tuples (x1, y1, x2, y2) apres compensation.
        Retourne liste_lignes inchangee si rayon_outil == 0.

    Raises:
        ValueError:   si rayon_outil < 0 ou cote invalide.
        RuntimeError: si Shapely produit une geometrie vide.
    """
    if rayon_outil < 0:
        raise ValueError(f"rayon_outil doit etre positif (recu : {rayon_outil}).")
    if cote not in ("exterieur", "interieur"):
        raise ValueError(f"cote invalide : '{cote}'. Attendu 'exterieur' ou 'interieur'.")
    if rayon_outil == 0.0 or not liste_lignes:
        return liste_lignes

    distance = rayon_outil if cote == "exterieur" else -rayon_outil
    polylignes = _chainer_segments_en_polylignes(liste_lignes, _TOLERANCE_CHAINING)

    segments_decales: list[tuple] = []
    for poly in polylignes:
        geom = _appliquer_offset_polyligne(poly, distance)
        if geom is None or geom.is_empty:
            raise RuntimeError(
                f"L'offset de {rayon_outil} mm produit une geometrie vide. "
                "Reduisez le rayon ou verifiez le contour."
            )
        segments_decales.extend(_geometrie_vers_segments(geom))

    return segments_decales


# ---------------------------------------------------------------------------
# API publique -- Pocketing concentrique
# ---------------------------------------------------------------------------

def calculer_trajectoires_poche(
    liste_lignes: list[tuple],
    rayon_outil: float,
    stepover_ratio: float = 0.5,
) -> list[list[tuple]]:
    """Calcule les trajectoires concentriques pour vider une poche fermee.

    Algorithme de shrink iteratif via Shapely :
      1. Reconstruit un Polygon a partir des segments.
      2. Premier offset interieur = -rayon_outil (centre outil tangent au bord).
      3. Boucle while : applique .buffer(-pas) ou pas = 2 * rayon * stepover_ratio.
      4. A chaque iteration, extrait les segments du sous-polygone.
      5. S'arrete quand le polygone devient vide ou nul.

    Exemple : D6 (rayon=3), stepover=0.5 -> pas = 3 mm entre chaque anneau.
    Pour une poche 60x50 mm : 8 anneaux generes.

    Args:
        liste_lignes:   Segments (x1, y1, x2, y2) formant le contour ferme.
        rayon_outil:    Rayon outil en mm (positif, ex: 3.0 pour D6).
        stepover_ratio: Taux de recouvrement radial dans ]0.0, 1.0[.
                        0.5 = 50% (standard). 0.3 = plus de passes, meilleur fini.

    Returns:
        Liste ordonnee d'anneaux (index 0 = exterieur, dernier = centre).
        Chaque anneau est une liste de segments (x1, y1, x2, y2).
        Retourne [] si le contour est trop petit pour l'outil.

    Raises:
        ValueError: si rayon_outil <= 0 ou stepover_ratio hors ]0, 1[.
    """
    if rayon_outil <= 0:
        raise ValueError(f"rayon_outil doit etre positif (recu : {rayon_outil}).")
    if not (0.0 < stepover_ratio < 1.0):
        raise ValueError(
            f"stepover_ratio doit etre dans ]0.0, 1.0[ (recu : {stepover_ratio})."
        )
    if not liste_lignes:
        return []

    # Pas radial entre deux anneaux consecutifs
    pas_radial: float = 2.0 * rayon_outil * stepover_ratio

    # Reconstruction des polylignes depuis les segments bruts
    polylignes = _chainer_segments_en_polylignes(liste_lignes, _TOLERANCE_CHAINING)

    # Seuls les contours fermes definissent une surface a vider
    polylignes_fermees = [p for p in polylignes if _est_fermee(p)]
    if not polylignes_fermees:
        return []

    # Assemblage des polygones fermes en zone a vider, TROUS COMPRIS :
    # un contour ferme contenu dans un autre est un ILOT (matiere a
    # conserver au centre de la poche), pas une surface a vider. Un simple
    # unary_union le remplirait et l'outil le raserait.
    from shapely.ops import unary_union
    polygones: list[Polygon] = []
    for pts in polylignes_fermees:
        p = Polygon(pts)
        if not p.is_valid:
            p = p.buffer(0)   # auto-reparation topologique
        if not p.is_empty and p.area > 0:
            polygones.append(p)

    if not polygones:
        return []

    zone = _assembler_zone_avec_ilots(polygones)

    # Premier shrink : positionne le centre outil a rayon_outil du bord
    trajectoires: list[list[tuple]] = []
    zone = zone.buffer(-rayon_outil, join_style="mitre", mitre_limit=5.0)

    # Boucle concentrique : shrink iteratif jusqu'a effondrement.
    # Chaque anneau ferme (exterieur, ilots, sous-zones apres scission) est
    # emis SEPAREMENT : le generateur fait une remontee/rampe par anneau,
    # jamais de coupe rectiligne entre deux anneaux disjoints.
    while not zone.is_empty:
        trajectoires.extend(_anneaux_fermes(zone))
        zone = zone.buffer(-pas_radial, join_style="mitre", mitre_limit=5.0)

    return trajectoires


def _assembler_zone_avec_ilots(polygones: list[Polygon]) -> BaseGeometry:
    """Assemble des contours fermes en zone de pocketing avec ilots.

    Regle pair/impair a un niveau : les polygones contenus dans un autre
    deviennent des trous (ilots) de celui-ci ; les autres sont des surfaces
    independantes. L'imbrication a plus de deux niveaux (ilot dans un trou)
    n'est pas geree -- cas non rencontre en usinage de poches classiques.
    """
    from shapely.ops import unary_union

    coquilles = [
        p for p in polygones
        if not any(autre is not p and autre.contains(p) for autre in polygones)
    ]
    zones = []
    for coquille in coquilles:
        ilots = [
            list(p.exterior.coords) for p in polygones
            if p is not coquille and coquille.contains(p)
        ]
        zones.append(Polygon(list(coquille.exterior.coords), holes=ilots))

    return unary_union(zones)


def _anneaux_fermes(geom: BaseGeometry) -> list[list[tuple]]:
    """Extrait chaque anneau ferme d'une geometrie en liste de segments
    SEPAREE : contour exterieur ET contours interieurs (ilots) de chaque
    polygone, recursivement pour les Multi/Collections.
    """
    from shapely.geometry import MultiPolygon, MultiLineString, GeometryCollection

    if isinstance(geom, (MultiPolygon, MultiLineString, GeometryCollection)):
        anneaux: list[list[tuple]] = []
        for sous_geom in geom.geoms:
            anneaux.extend(_anneaux_fermes(sous_geom))
        return anneaux

    if isinstance(geom, Polygon):
        boucles = [geom.exterior] + list(geom.interiors)
    elif isinstance(geom, LineString):
        boucles = [geom]
    else:
        return []

    anneaux = []
    for boucle in boucles:
        coords = list(boucle.coords)
        segs = [
            (coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1])
            for i in range(len(coords) - 1)
        ]
        if segs:
            anneaux.append(segs)
    return anneaux


# ---------------------------------------------------------------------------
# Fonctions internes (prefixe _ = usage prive)
# ---------------------------------------------------------------------------

def _chainer_segments_en_polylignes(
    liste_lignes: list[tuple],
    tolerance: float,
) -> list[list[tuple[float, float]]]:
    """Regroupe des segments (x1,y1,x2,y2) en polylignes continues.

    Deux segments consecutifs sont enchaines si la distance entre la fin
    du premier et le debut du second est <= tolerance.

    Args:
        liste_lignes: Segments bruts.
        tolerance:    Distance max en mm pour considerer deux points confondus.

    Returns:
        Liste de polylignes (chaque polyligne = liste de points (x, y)).
    """
    if not liste_lignes:
        return []

    x1, y1, x2, y2 = liste_lignes[0]
    polyligne_courante: list[tuple[float, float]] = [(x1, y1), (x2, y2)]
    polylignes: list[list[tuple[float, float]]] = []

    for i in range(1, len(liste_lignes)):
        xp1, yp1, xp2, yp2 = liste_lignes[i]
        xfin, yfin = polyligne_courante[-1]
        if math.hypot(xp1 - xfin, yp1 - yfin) <= tolerance:
            polyligne_courante.append((xp2, yp2))
        else:
            polylignes.append(polyligne_courante)
            polyligne_courante = [(xp1, yp1), (xp2, yp2)]

    polylignes.append(polyligne_courante)
    return polylignes


def _est_fermee(
    polyligne: list[tuple[float, float]],
    tolerance: float = _TOLERANCE_CHAINING,
) -> bool:
    """Retourne True si premier et dernier point sont a moins de tolerance mm."""
    if len(polyligne) < 3:
        return False
    return math.hypot(
        polyligne[0][0] - polyligne[-1][0],
        polyligne[0][1] - polyligne[-1][1],
    ) <= tolerance


def _appliquer_offset_polyligne(
    polyligne: list[tuple[float, float]],
    distance: float,
) -> BaseGeometry | None:
    """Applique un offset Shapely sur une polyligne ouverte ou fermee.

    Contour ferme  -> Polygon.buffer() (gere angles et topologie).
    Polyligne ouverte -> LineString.offset_curve() (Shapely 2.x).

    Args:
        polyligne: Liste de points (x, y).
        distance:  Valeur d'offset en mm (+ = ext, - = int).

    Returns:
        Geometrie Shapely decalee, ou None si polyligne invalide.
    """
    if len(polyligne) < 2:
        return None

    if _est_fermee(polyligne):
        p = Polygon(polyligne)
        if not p.is_valid:
            p = p.buffer(0)
        return p.buffer(distance, join_style="mitre", mitre_limit=5.0)
    else:
        return LineString(polyligne).offset_curve(
            distance, join_style="mitre", mitre_limit=5.0
        )


def _geometrie_vers_segments(geom: BaseGeometry) -> list[tuple]:
    """Convertit une geometrie Shapely en liste de segments (x1,y1,x2,y2).

    Gere recursivement : Polygon, MultiPolygon, LineString,
    MultiLineString, GeometryCollection.

    Args:
        geom: Geometrie Shapely issue d'un offset ou d'un buffer.

    Returns:
        Liste de tuples (x1, y1, x2, y2).
    """
    from shapely.geometry import MultiPolygon, MultiLineString, GeometryCollection

    if isinstance(geom, (MultiPolygon, MultiLineString, GeometryCollection)):
        segments: list[tuple] = []
        for sous_geom in geom.geoms:
            segments.extend(_geometrie_vers_segments(sous_geom))
        return segments

    if isinstance(geom, Polygon):
        coords = list(geom.exterior.coords)
    elif isinstance(geom, LineString):
        coords = list(geom.coords)
    else:
        return []

    return [
        (coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1])
        for i in range(len(coords) - 1)
    ]


# ---------------------------------------------------------------------------
# Utilitaires geometriques
# ---------------------------------------------------------------------------

def compute_line_length(start: tuple[float, float], end: tuple[float, float]) -> float:
    """Longueur euclidienne d'un segment."""
    return math.hypot(end[0] - start[0], end[1] - start[1])


def compute_arc_length(radius: float, start_angle: float, end_angle: float) -> float:
    """Longueur d'arc entre deux angles en degres."""
    span = (end_angle - start_angle) % 360
    return radius * math.radians(span if span > 0 else 360.0)


def compute_arc_start_point(
    center: tuple[float, float], radius: float, start_angle: float
) -> tuple[float, float]:
    """Point de depart d'un arc (coordonnees absolues)."""
    a = math.radians(start_angle)
    return (center[0] + radius * math.cos(a), center[1] + radius * math.sin(a))


def compute_arc_end_point(
    center: tuple[float, float], radius: float, end_angle: float
) -> tuple[float, float]:
    """Point de fin d'un arc (identique a start avec end_angle)."""
    return compute_arc_start_point(center, radius, end_angle)


def compute_arc_center_offsets(
    center: tuple[float, float], start_point: tuple[float, float]
) -> tuple[float, float]:
    """Offsets I, J du centre par rapport au point de depart (format Fanuc)."""
    return (center[0] - start_point[0], center[1] - start_point[1])


def compute_circle_circumference(radius: float) -> float:
    """Perimetre d'un cercle complet."""
    return 2 * math.pi * radius


def compute_bounding_box(
    points: list[tuple[float, float]],
) -> tuple[float, float, float, float]:
    """Boite englobante d'un nuage de points. Retourne (xmin, ymin, xmax, ymax)."""
    if not points:
        raise ValueError("La liste de points ne peut pas etre vide.")
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def round_coordinate(value: float, decimals: int = 3) -> float:
    """Arrondit a la precision machine standard (defaut : micron)."""
    return round(value, decimals)
