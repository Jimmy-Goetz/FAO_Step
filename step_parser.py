"""
step_parser.py — Lecture et extraction des features 3D depuis un fichier STEP.

Responsabilite unique : ouvrir un fichier STEP (.stp / .step) via trimesh,
analyser le maillage resultant et retourner des structures simples (dict)
exploitables par le moteur de calcul existant (geometry.py / gcode_generator.py).
Aucune logique d'usinage ici : ce module ne fait que traduire la geometrie 3D
en donnees plates (bounding box, faces planes horizontales, percages
cylindriques).

Necessite : pip install trimesh scipy cascadio
(cascadio fournit le backend OpenCASCADE utilise par trimesh pour lire le STEP)
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import TypedDict

import numpy as np

try:
    import trimesh
except ImportError as exc:
    raise ImportError(
        "trimesh est requis : pip install trimesh scipy cascadio"
    ) from exc

try:
    from scipy.optimize import least_squares
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
except ImportError as exc:
    raise ImportError("scipy est requis : pip install scipy") from exc


# ---------------------------------------------------------------------------
# Structures de retour
# ---------------------------------------------------------------------------

class DimensionsBrut(TypedDict):
    """Dimensions du bloc englobant (bounding box) de la piece.

    Repere recale par _charger_mesh (coin inferieur gauche a X=0/Y=0, face
    superieure a Z=0) : min_corner vaut donc toujours (0, 0, -z) et
    max_corner (x, y, 0), quelle que soit l'origine du fichier STEP source.

    Attributes:
        x, y, z:     Dimensions du brut en mm.
        min_corner:  Coin inferieur (x, y, z) de la bounding box, repere recale.
        max_corner:  Coin superieur (x, y, z) de la bounding box, repere recale.
    """
    x: float
    y: float
    z: float
    min_corner: tuple[float, float, float]
    max_corner: tuple[float, float, float]


class FacePlane(TypedDict):
    """Face plane horizontale detectee -- candidate a une poche.

    Attributes:
        z:        Hauteur relative a la face superieure du brut (Z=0), donc <= 0.
        aire:     Aire cumulee de la face en mm^2.
        centre:   Centroide (x, y) de la face dans le plan.
        min_xy:   Coin inferieur (x, y) de la bbox 2D de la face.
        max_xy:   Coin superieur (x, y) de la bbox 2D de la face.
        contours: Boucles frontieres REELLES de la face, projetees en XY :
                  liste de boucles, chaque boucle = liste de points (x, y)
                  fermee implicitement. La premiere est generalement le
                  contour exterieur, les suivantes des ilots. Permet
                  d'usiner la vraie forme (poche en L, avec ilot...) au
                  lieu de la simple bbox.
    """
    z: float
    aire: float
    centre: tuple[float, float]
    min_xy: tuple[float, float]
    max_xy: tuple[float, float]
    contours: list[list[tuple[float, float]]]


class Percage(TypedDict):
    """Trou cylindrique detecte.

    Attributes:
        x, y:       Centre du cercle dans le plan (repere piece).
        rayon:      Rayon du percage en mm.
        profondeur: Profondeur signee (negative), coherente avec
                    MachiningConfig.default_target_depth.
        z_depart:   Altitude (relative a Z=0 en face superieure) ou commence
                    le percage -- utile pour les avant-trous/lamages.
    """
    x: float
    y: float
    rayon: float
    profondeur: float
    z_depart: float


# Tolerances geometriques
TOLERANCE_PLAN: float = 1e-3        # ecart de normale accepte pour "horizontal"
TOLERANCE_Z_GROUPE: float = 0.05    # mm, regroupement de faces a la meme hauteur
TOLERANCE_CERCLE_RATIO: float = 0.05  # residu de fit / rayon max tolere
MIN_SOMMETS_CERCLE: int = 10        # nb mini de points distincts autour du contour
                                    # (un polygone concyclique -- ex: rectangle --
                                    # peut avoir un residu de fit nul mais tres peu
                                    # de sommets ; un vrai cylindre tessellé en STEP
                                    # en compte toujours beaucoup plus)

# Tessellation STEP->maillage (passee a trimesh.load, forwardee a cascadio).
# Par defaut cascadio utilise tol_angular=0.5 rad (~28.6 deg), ce qui ne
# garantit qu'environ 2*pi/0.5 =~ 12-13 segments par cercle COMPLET, quel
# que soit son rayon -- dangereusement proche de MIN_SOMMETS_CERCLE (10) et
# suffisant pour faire echouer la detection d'un trou sur certains fichiers.
# On resserre explicitement pour garantir une marge confortable (~40+ segments).
STEP_TOL_LINEAR:  float = 0.01   # mm, fleche de corde max (deflection lineaire)
STEP_TOL_ANGULAR: float = 0.15   # rad, deflection angulaire max (~8.6 deg)

# Tolerance dediee a la detection des parois de percage (distincte de
# TOLERANCE_PLAN, utilisee pour les faces horizontales) : la tessellation
# reelle d'un cylindre STEP (cascadio) peut fragmenter la paroi d'un meme
# trou en plusieurs sous-groupes de facettes non exactement coplanaires --
# un seuil plus genereux evite d'en exclure inutilement une partie.
TOLERANCE_PAROI_VERTICALE: float = 0.02
MIN_POINTS_ARC: int = 5             # nb mini de points pour tenter un fit de
                                     # cercle preliminaire sur un fragment de paroi
TOLERANCE_FUSION_CERCLE: float = 0.5  # mm, ecart max centre/rayon entre deux
                                       # fragments pour les considerer comme la
                                       # meme paroi cylindrique physique

# Alignement d'axe requis pour qu'un cylindre soit machinable par une
# fraiseuse 3 axes (l'outil ne descend qu'a la verticale, axe Z) :
# |axe . Z| doit etre proche de 1 (axe quasi vertical, [0,0,1] ou [0,0,-1]).
TOLERANCE_AXE_VERTICAL: float = 0.05   # ~=  3 degres d'ecart tolere a la verticale
MIN_TRIANGLES_AXE: int = 4   # nb mini de triangles pour juger l'axe par PCA de
                              # facon fiable (trop peu de normales -> direction
                              # de variance minimale instable/non significative ;
                              # ces petits groupes sont laisses passer et geres
                              # par la fusion/filtrage habituels en aval)


# ---------------------------------------------------------------------------
# API publique
# ---------------------------------------------------------------------------

def test_import_step(filepath: str) -> None:
    """Fonction de test appelee par un futur bouton 'Importer STEP' de l'UI.

    Args:
        filepath: Chemin vers le fichier STEP selectionne.
    """
    print(f"[step_parser] Fichier recu : {filepath}")
    try:
        dimensions = extraire_dimensions_brut(filepath)
        print(f"[step_parser] Dimensions brut : {dimensions}")
        features = analyser_features_3d(filepath)
        print(f"[step_parser] Faces planes trouvees : {len(features['faces_planes'])}")
        print(f"[step_parser] Percages trouves : {len(features['percages'])}")
        for face in features["faces_planes"][:5]:
            print(f"  |-- face Z={face['z']} aire={face['aire']:.1f}mm^2")
        for trou in features["percages"][:5]:
            print(
                f"  |-- percage x={trou['x']} y={trou['y']} "
                f"r={trou['rayon']} prof={trou['profondeur']}"
            )
    except Exception as exc:
        print(f"[step_parser] ERREUR : {exc}")


def extraire_dimensions_brut(filepath: str | Path) -> DimensionsBrut:
    """Calcule la bounding box de la piece pour dimensionner le brut.

    La piece est recalee par _charger_mesh avant lecture de la bbox : le
    coin inferieur gauche est donc a X=0/Y=0 et la face superieure a Z=0,
    independamment de l'origine definie dans le fichier STEP source.

    Args:
        filepath: Chemin absolu ou relatif vers le fichier .stp/.step.

    Returns:
        DimensionsBrut avec les etendues X, Y, Z et les coins de la bbox
        (repere recale : min_corner = (0, 0, -z), max_corner = (x, y, 0)).

    Raises:
        FileNotFoundError: si le fichier n'existe pas.
        ValueError: si le fichier ne peut pas etre lu ou ne contient pas
            de geometrie exploitable.
    """
    mesh = _charger_mesh(filepath)
    min_corner, max_corner = mesh.bounds
    dx, dy, dz = mesh.extents

    return {
        "x": float(dx),
        "y": float(dy),
        "z": float(dz),
        "min_corner": (float(min_corner[0]), float(min_corner[1]), float(min_corner[2])),
        "max_corner": (float(max_corner[0]), float(max_corner[1]), float(max_corner[2])),
    }


def extraire_maillage_filaire(filepath: str | Path) -> dict[str, list]:
    """Extrait sommets et aretes du maillage pour un affichage filaire 3D.

    La piece est recalee par _charger_mesh (face superieure a Z=0, coin
    inferieur gauche a X=0/Y=0) afin que le viewer et l'analyse de features
    partagent exactement le meme repere.

    Args:
        filepath: Chemin absolu ou relatif vers le fichier .stp/.step.

    Returns:
        Dictionnaire a deux cles :
            "vertices" : list[tuple[float, float, float]] -- sommets uniques.
            "edges"    : list[tuple[int, int]] -- paires d'indices dans
                         "vertices" (aretes uniques du maillage).

    Raises:
        FileNotFoundError: si le fichier n'existe pas.
        ValueError: si le fichier ne peut pas etre lu ou ne contient pas
            de geometrie exploitable.
    """
    mesh = _charger_mesh(filepath)

    return {
        "vertices": [(float(v[0]), float(v[1]), float(v[2])) for v in mesh.vertices],
        "edges": [(int(a), int(b)) for a, b in mesh.edges_unique],
    }


# ---------------------------------------------------------------------------
# Orientations d'usinage (multi-faces 3 axes / 4 axes)
# ---------------------------------------------------------------------------
# Chaque orientation = (nom, rotation (angle, axe) a appliquer au maillage pour
# ramener la face concernee "vers le haut" (+Z). Une fois le maillage tourne
# puis recale (coin bas-gauche a l'origine, dessus a Z=0), TOUS les detecteurs
# existants (_detecter_faces_planes, _detecter_percages, filtres d'occlusion et
# d'axe vertical) fonctionnent sans modification dans le repere tourne.
_ORIENTATIONS_USINAGE: list[tuple[str, tuple[float, tuple[float, float, float]] | None]] = [
    ("Z_PLUS",  None),                              # face superieure (repere natif)
    ("Y_PLUS",  (math.pi / 2.0,  (1.0, 0.0, 0.0))), # flanc Y+ ramene dessus
    ("Z_MINUS", (math.pi,        (1.0, 0.0, 0.0))), # dessous ramene dessus
    ("Y_MINUS", (-math.pi / 2.0, (1.0, 0.0, 0.0))), # flanc Y- ramene dessus
    ("X_PLUS",  (-math.pi / 2.0, (0.0, 1.0, 0.0))), # flanc X+ ramene dessus
    ("X_MINUS", (math.pi / 2.0,  (0.0, 1.0, 0.0))), # flanc X- ramene dessus
]


def analyser_features_multi_faces(filepath: str | Path) -> dict[str, dict]:
    """Analyse le solide STEP sous TOUTES les orientations d'usinage 3 axes.

    Pour chaque orientation de _ORIENTATIONS_USINAGE, applique virtuellement
    la matrice de rotation qui ramene la face concernee vers le haut, recale
    le maillage tourne dans le repere machine (coin bas-gauche -> X0/Y0,
    dessus -> Z0), puis reutilise les detecteurs standards. Les features
    detectees sont donc exprimees dans le repere DE CETTE FACE : le G-code
    genere pour cette orientation suppose la piece retournee/pivotee pour
    presenter cette face a la broche.

    Deduplication :
      - orientations *_MINUS : les percages traversants sont retires (ils
        sont deja usines depuis la face opposee *_PLUS) ;
      - les petites faces planes qui ne sont que le FOND d'un percage deja
        detecte sont retirees (le fraisage en colimacon du trou les couvre).

    Returns:
        Dict {nom_orientation: {"faces_planes": [...], "percages": [...],
        "dimensions": {"x","y","z"}}} -- seules les orientations qui ont
        au moins une feature sont presentes (Z_PLUS toujours presente : elle
        porte le contournage du brut).
    """
    mesh_base = _charger_mesh(filepath)
    resultats: dict[str, dict] = {}

    # Registre des percages deja retenus, exprimes dans le repere D'ORIGINE
    # (extremites de l'axe + rayon) : le meme cylindre physique vu depuis
    # deux orientations differentes (ex : trou traversant vu de Y+ et Y-,
    # lamage vu du dessus et du dessous) n'est usine qu'UNE fois, depuis
    # l'orientation la plus prioritaire (ordre de _ORIENTATIONS_USINAGE).
    trous_retenus: list[tuple[np.ndarray, np.ndarray, float]] = []

    for nom, rotation in _ORIENTATIONS_USINAGE:
        m = mesh_base.copy()
        if rotation is not None:
            angle, axe = rotation
            matrice = trimesh.transformations.rotation_matrix(angle, list(axe))
            m.apply_transform(matrice)
            rot3 = matrice[:3, :3]
        else:
            rot3 = np.eye(3)
        min_corner, max_corner = m.bounds
        decal = np.array(
            [-float(min_corner[0]), -float(min_corner[1]), -float(max_corner[2])]
        )
        m.apply_translation(decal)
        dx, dy, dz = (float(v) for v in m.extents)

        print(f"[step_parser] --- orientation {nom} (brut vu {dx:.1f}x{dy:.1f}x{dz:.1f}mm) ---")
        faces = _detecter_faces_planes(m)
        percages = _detecter_percages(m)

        # Deduplication inter-faces via le repere d'origine :
        # p_origine = R^-1 @ (p_repere_face - decalage)
        percages_uniques: list[Percage] = []
        for p in percages:
            e1 = rot3.T @ (np.array([p["x"], p["y"], p["z_depart"]]) - decal)
            e2 = rot3.T @ (
                np.array([p["x"], p["y"], p["z_depart"] + p["profondeur"]]) - decal
            )
            deja_vu = any(
                abs(r - p["rayon"]) <= 0.5
                and (
                    (np.linalg.norm(e1 - a) <= 0.5 and np.linalg.norm(e2 - b) <= 0.5)
                    or (np.linalg.norm(e1 - b) <= 0.5 and np.linalg.norm(e2 - a) <= 0.5)
                )
                for a, b, r in trous_retenus
            )
            if deja_vu:
                print(
                    f"[step_parser]   percage D{2*p['rayon']:.1f} "
                    f"({p['x']:.1f},{p['y']:.1f}) IGNORE : deja usine depuis "
                    f"une orientation precedente"
                )
                continue
            trous_retenus.append((e1, e2, p["rayon"]))
            percages_uniques.append(p)
        percages = percages_uniques

        faces = _retirer_faces_couvertes_par_percages(faces, percages)

        if nom != "Z_PLUS" and not faces and not percages:
            continue   # rien a usiner sur cette face : pas de fichier a produire

        resultats[nom] = {
            "faces_planes": faces,
            "percages": percages,
            "dimensions": {"x": dx, "y": dy, "z": dz},
        }
        if nom == "Z_PLUS":
            resultats[nom]["silhouette"] = _silhouette_xy(m)

    return resultats


def _silhouette_xy(mesh: trimesh.Trimesh) -> list[tuple[float, float]] | None:
    """Contour exterieur REEL de la projection de la piece sur le plan XY.

    Remplace la bbox pour le contournage : une piece non rectangulaire est
    detouree suivant sa vraie silhouette. Retourne la boucle exterieure
    (liste de points (x, y), fermee implicitement) ou None si la projection
    echoue (le generateur retombe alors sur la bbox du brut).
    Les eventuels trous de la projection (fenetres traversantes) ne sont pas
    retournes : ils relevent du pocketing, pas du detourage exterieur.
    """
    try:
        from trimesh.path import polygons as trimesh_polygons
        projection = trimesh_polygons.projected(mesh, normal=[0.0, 0.0, 1.0])
        if projection is None or projection.is_empty:
            return None
        if projection.geom_type == "MultiPolygon":
            projection = max(projection.geoms, key=lambda p: p.area)
        boucle = [
            (float(x), float(y)) for x, y in list(projection.exterior.coords)[:-1]
        ]
        return boucle if len(boucle) >= 3 else None
    except Exception as exc:
        print(f"[step_parser] silhouette : projection impossible ({exc}) -- repli bbox")
        return None


def _retirer_faces_couvertes_par_percages(
    faces: list[FacePlane], percages: list[Percage],
) -> list[FacePlane]:
    """Retire les faces planes qui ne sont que le fond d'un percage detecte.

    Le fond plat d'un trou borgne est a la fois une 'face plane accessible'
    et le fond du cylindre fraise en colimacon : sans ce filtre, la zone
    serait usinee deux fois (pocketing du trou PUIS pocketing de son fond).
    Une face est couverte si sa bbox tient entierement dans le cercle d'un
    percage ET si elle se situe au fond de celui-ci.
    """
    restantes: list[FacePlane] = []
    for f in faces:
        couverte = False
        for p in percages:
            z_fond = p["z_depart"] + p["profondeur"]
            if abs(f["z"] - z_fond) > 0.1:
                continue
            # La face est couverte si sa bbox tient dans l'empreinte carree
            # du percage (le fond d'un trou est un disque : sa bbox est le
            # carre circonscrit, dont les coins sortent du cercle -- comparer
            # les bornes, pas la distance des coins au centre).
            marge = p["rayon"] + 0.1
            if (
                f["min_xy"][0] >= p["x"] - marge
                and f["max_xy"][0] <= p["x"] + marge
                and f["min_xy"][1] >= p["y"] - marge
                and f["max_xy"][1] <= p["y"] + marge
            ):
                couverte = True
                break
        if not couverte:
            restantes.append(f)
    return restantes


def analyser_features_3d(filepath: str | Path) -> dict[str, list]:
    """Analyse le solide STEP pour en extraire les faces planes et les percages.

    La piece est recalee par _charger_mesh : face superieure (Z max de la
    bbox d'origine) -> Z=0 et coin inferieur gauche (X min, Y min) -> X=0/Y=0.
    Toutes les hauteurs retournees sont donc negatives ou nulles, coherentes
    avec la convention de profondeur du moteur existant
    (MachiningConfig.default_target_depth), et toutes les coordonnees XY sont
    positives, coherentes avec l'origine machine (G54) attendue par le
    Bouclier de Securite de gcode_generator.py.

    Args:
        filepath: Chemin absolu ou relatif vers le fichier .stp/.step.

    Returns:
        Dictionnaire a deux cles :
            "faces_planes" : list[FacePlane] -- profondeurs de poches candidates.
            "percages"     : list[Percage]   -- trous cylindriques detectes.

    Raises:
        FileNotFoundError: si le fichier n'existe pas.
        ValueError: si le fichier ne peut pas etre lu ou ne contient pas
            de geometrie exploitable.
    """
    mesh = _charger_mesh(filepath)

    faces_planes = _detecter_faces_planes(mesh)
    percages = _detecter_percages(mesh)
    faces_planes = _retirer_faces_couvertes_par_percages(faces_planes, percages)

    return {
        "faces_planes": faces_planes,
        "percages": percages,
        "silhouette": _silhouette_xy(mesh),
    }


# ---------------------------------------------------------------------------
# Fonctions internes -- chargement
# ---------------------------------------------------------------------------

def _charger_mesh(filepath: str | Path) -> trimesh.Trimesh:
    """Ouvre le fichier STEP, tessellue en un maillage unique, et recale la
    piece dans le repere attendu par le moteur d'usinage existant :

      - Coin inferieur gauche du brut (X min, Y min) -> X = 0, Y = 0.
      - Face superieure du brut (Z max)               -> Z = 0, de sorte que
        toute la matiere se trouve en Z <= 0 (convention de profondeur de
        MachiningConfig.default_target_depth et de tout gcode_generator.py).

    Le recalage est applique une seule fois, ici, a la source : toutes les
    fonctions publiques du module (extraire_dimensions_brut,
    extraire_maillage_filaire, analyser_features_3d) travaillent donc
    automatiquement dans le meme repere partage, sans transformation
    redondante ni risque de divergence entre elles.

    Note : trimesh centre parfois le repere du solide sur son propre milieu
    (ex. Z de -15 a +15 pour une plaque de 30mm) selon l'origine definie dans
    le fichier STEP d'origine -- ce recalage est donc necessaire et ne doit
    pas dependre de l'origine choisie par le logiciel de CAO source.

    La tessellation est resserree explicitement (STEP_TOL_LINEAR /
    STEP_TOL_ANGULAR) par rapport aux defauts de cascadio, qui ne garantissent
    qu'une dizaine de segments par cercle complet -- une marge trop faible
    au regard de MIN_SOMMETS_CERCLE, risquant de faire echouer silencieusement
    la detection d'un percage sur certains fichiers.
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"Fichier STEP introuvable : {filepath}")

    try:
        geometrie = trimesh.load(
            str(filepath), force="mesh",
            tol_linear=STEP_TOL_LINEAR, tol_angular=STEP_TOL_ANGULAR,
        )
    except Exception as exc:
        raise ValueError(f"Impossible de lire le fichier STEP : {exc}") from exc

    if not isinstance(geometrie, trimesh.Trimesh):
        raise ValueError(
            "Le fichier STEP n'a pas produit un maillage exploitable "
            f"(type recu : {type(geometrie).__name__})"
        )
    if geometrie.is_empty:
        raise ValueError("Le fichier STEP ne contient aucune geometrie.")

    _convertir_en_millimetres(geometrie)

    min_corner, max_corner = geometrie.bounds
    decalage = (-float(min_corner[0]), -float(min_corner[1]), -float(max_corner[2]))
    geometrie.apply_translation(decalage)

    return geometrie


def _convertir_en_millimetres(mesh: trimesh.Trimesh) -> None:
    """Ramene le maillage en millimetres, l'unite de travail du moteur FAO.

    CRUCIAL : cascadio (backend STEP de trimesh) convertit TOUJOURS la
    geometrie en METRES lors du passage par le format GLB intermediaire
    (la spec glTF impose le metre), quelle que soit l'unite declaree dans
    le fichier STEP source (mm, pouce, ...). Sans cette reconversion, un
    brut de 100mm arrive dans le pipeline avec une etendue de 0.1 :
      - tous les seuils de detection (MIN_SOMMETS_CERCLE, tolerances en mm)
        rejettent les percages, silencieusement ;
      - le contournage trace un offset d'outil (ex: rayon 3mm) autour d'un
        brut quasi-nul -> trajectoires absurdes de quelques mm ;
      - le viewer se fige sur une boite minuscule.

    Detection : trimesh renseigne mesh.units ('meters') depuis le GLB.
    Si l'attribut est absent (versions futures), une heuristique de repli
    detecte un maillage anormalement petit (< 1.0 unite dans toutes les
    dimensions : aucune piece fraisable ne fait moins de 1mm hors tout,
    alors qu'une piece en metres fait toujours moins de 1m sur cette table).
    """
    unites = getattr(mesh, "units", None)

    if unites in ("meters", "m", "meter"):
        mesh.apply_scale(1000.0)
        mesh.units = "millimeters"
        print("[step_parser] unites : maillage recu en metres (GLB) -> converti en mm (x1000)")
        return

    if unites in ("millimeters", "mm", "millimeter"):
        return   # deja en mm, rien a faire

    # Unite inconnue/absente : heuristique de repli.
    etendue_max = float(mesh.extents.max())
    if etendue_max < 1.0:
        mesh.apply_scale(1000.0)
        mesh.units = "millimeters"
        print(
            f"[step_parser] unites : maillage anormalement petit "
            f"(etendue max {etendue_max:.4f}, unite non declaree) -- "
            f"suppose en metres, converti en mm (x1000)"
        )
    else:
        print(
            f"[step_parser] unites : non declarees, etendue max {etendue_max:.1f} "
            f"-- supposees deja en mm"
        )


# ---------------------------------------------------------------------------
# Fonctions internes -- faces planes horizontales
# ---------------------------------------------------------------------------

def _detecter_faces_planes(mesh: trimesh.Trimesh) -> list[FacePlane]:
    """Detecte les faces planes usinables par le dessus (fraiseuse 3 axes).

    Une face n'est une poche candidate que si TOUTES ces conditions tiennent :

    1. Normale STRICTEMENT orientee vers le haut (Nz ~= +1). Une facette
       horizontale a normale vers le bas est un PLAFOND (dessous du brut,
       haut interieur d'un alesage horizontal...) : l'outil, qui descend
       verticalement, ne peut pas l'usiner -- la traiter comme une poche
       fraiserait a travers la matiere du dessus. Ce filtre elimine
       notamment les bandes tangentes superieures des percages horizontaux
       de flanc, qui generaient des trajectoires parasites sur le cote.
    2. Pas la face superieure du brut (Z=0) : rien a usiner.
    3. Accessible par le dessus (_face_accessible_par_dessus) : une face
       orientee vers le haut mais ENFOUIE sous de la matiere (ex : fond
       interieur d'un alesage horizontal, dont la bande basse regarde vers
       le haut) est hors d'atteinte d'un outil vertical. Verifie par lancer
       de rayons verticaux : si un rayon parti de la face vers +Z rencontre
       le maillage, la face est couverte -> exclue.

    Seules les coordonnees X/Y des sommets servent a decrire le contour 2D
    (min_xy/max_xy) ; le Z de la face ne sert QUE de profondeur de passe.
    """
    candidates: list[FacePlane] = []

    for face_indices, normale, aire in zip(
        mesh.facets, mesh.facets_normal, mesh.facets_area
    ):
        if normale[2] < 1.0 - TOLERANCE_PLAN:
            continue  # paroi verticale OU plafond (normale vers le bas)

        vertex_ids = np.unique(mesh.faces[face_indices])
        points = mesh.vertices[vertex_ids]
        z = float(points[:, 2].mean())

        if abs(z) < TOLERANCE_Z_GROUPE:
            continue  # face superieure du brut (Z=0), pas une poche

        if not _face_accessible_par_dessus(mesh, face_indices):
            print(
                f"[step_parser] face Z={z:.2f} (aire {aire:.0f}mm2) IGNOREE : "
                f"enfouie sous la matiere, inaccessible par le dessus (3 axes)"
            )
            continue

        candidates.append({
            "z": z,
            "aire": float(aire),
            "centre": (float(points[:, 0].mean()), float(points[:, 1].mean())),
            "min_xy": (float(points[:, 0].min()), float(points[:, 1].min())),
            "max_xy": (float(points[:, 0].max()), float(points[:, 1].max())),
            "contours": _contours_face(mesh, face_indices),
        })

    return _fusionner_faces_par_hauteur(candidates)


def _contours_face(
    mesh: trimesh.Trimesh, face_indices: np.ndarray,
) -> list[list[tuple[float, float]]]:
    """Extrait les boucles frontieres d'un groupe de triangles coplanaires,
    projetees dans le plan XY.

    Une arete appartient a la frontiere si elle n'est partagee que par UN
    triangle du groupe. Les aretes frontieres sont ensuite chainees en
    boucles fermees (contour exterieur + eventuels ilots). C'est la VRAIE
    forme de la face -- contrairement a la bbox, une poche en L ou avec
    ilot central est restituee fidele.
    """
    triangles = mesh.faces[face_indices]

    compte_aretes: dict[tuple[int, int], int] = {}
    for tri in triangles:
        for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            cle = (min(int(a), int(b)), max(int(a), int(b)))
            compte_aretes[cle] = compte_aretes.get(cle, 0) + 1

    # Adjacence des sommets frontiere (chaque sommet d'une frontiere
    # manifold a exactement 2 voisins frontiere)
    voisins: dict[int, list[int]] = {}
    for (a, b), n in compte_aretes.items():
        if n == 1:
            voisins.setdefault(a, []).append(b)
            voisins.setdefault(b, []).append(a)

    boucles: list[list[tuple[float, float]]] = []
    visites: set[int] = set()
    for depart in voisins:
        if depart in visites or len(voisins[depart]) != 2:
            continue
        boucle_ids = [depart]
        visites.add(depart)
        precedent, courant = None, depart
        while True:
            suivant = next(
                (v for v in voisins[courant] if v != precedent), None
            )
            if suivant is None or suivant == depart:
                break
            if suivant in visites:
                break   # pincement non-manifold : boucle abandonnee proprement
            boucle_ids.append(suivant)
            visites.add(suivant)
            precedent, courant = courant, suivant

        if len(boucle_ids) >= 3:
            pts = mesh.vertices[boucle_ids]
            boucles.append([(float(p[0]), float(p[1])) for p in pts])

    return boucles


def _face_accessible_par_dessus(
    mesh: trimesh.Trimesh,
    face_indices: np.ndarray,
    nb_rayons_max: int = 8,
    decalage_z: float = 0.05,
) -> bool:
    """True si la face est atteignable par un outil descendant verticalement.

    Lance des rayons +Z depuis quelques centroides de triangles de la face
    (legerement decolles de la surface pour ne pas se re-intersecter avec
    elle-meme) : si UN rayon rencontre le maillage, il y a de la matiere
    au-dessus -- la face est couverte, donc inusinable en 3 axes.

    Strict volontairement (un seul rayon bloque suffit a exclure) : exclure
    une face douteuse laisse de la matiere (sur), l'inclure a tort fait
    fraiser a travers la piece (dangereux).
    """
    triangles = mesh.triangles[face_indices]
    centroides = triangles.mean(axis=1)

    if centroides.shape[0] > nb_rayons_max:
        pas = centroides.shape[0] // nb_rayons_max
        centroides = centroides[::pas][:nb_rayons_max]

    origines = centroides + np.array([0.0, 0.0, decalage_z])
    directions = np.tile([0.0, 0.0, 1.0], (origines.shape[0], 1))

    impacts = mesh.ray.intersects_any(
        ray_origins=origines, ray_directions=directions
    )
    return not bool(np.any(impacts))


def _fusionner_faces_par_hauteur(faces: list[FacePlane]) -> list[FacePlane]:
    """Regroupe les facettes coplanaires disjointes qui partagent la meme
    hauteur (ex : fond de poche compose de plusieurs triangles/facettes).
    """
    if not faces:
        return []

    faces = sorted(faces, key=lambda f: f["z"], reverse=True)
    groupes: list[list[FacePlane]] = [[faces[0]]]

    for face in faces[1:]:
        if abs(face["z"] - groupes[-1][-1]["z"]) <= TOLERANCE_Z_GROUPE:
            groupes[-1].append(face)
        else:
            groupes.append([face])

    return [_moyenne_ponderee_aire(groupe) for groupe in groupes]


def _moyenne_ponderee_aire(groupe: list[FacePlane]) -> FacePlane:
    aire_totale = sum(f["aire"] for f in groupe)
    min_x = min(f["min_xy"][0] for f in groupe)
    min_y = min(f["min_xy"][1] for f in groupe)
    max_x = max(f["max_xy"][0] for f in groupe)
    max_y = max(f["max_xy"][1] for f in groupe)

    if aire_totale <= 0:
        z, cx, cy = groupe[0]["z"], groupe[0]["centre"][0], groupe[0]["centre"][1]
    else:
        z = sum(f["z"] * f["aire"] for f in groupe) / aire_totale
        cx = sum(f["centre"][0] * f["aire"] for f in groupe) / aire_totale
        cy = sum(f["centre"][1] * f["aire"] for f in groupe) / aire_totale

    contours: list[list[tuple[float, float]]] = []
    for f in groupe:
        contours.extend(f.get("contours", []))

    return {
        "z": round(z, 4),
        "aire": round(aire_totale, 2),
        "centre": (round(cx, 4), round(cy, 4)),
        "min_xy": (round(min_x, 4), round(min_y, 4)),
        "max_xy": (round(max_x, 4), round(max_y, 4)),
        "contours": contours,
    }


# ---------------------------------------------------------------------------
# Fonctions internes -- percages cylindriques
# ---------------------------------------------------------------------------

def _detecter_percages(mesh: trimesh.Trimesh) -> list[Percage]:
    """Detecte les percages cylindriques VERTICAUX et les convertit en
    entites Cercle parfaites (centre X/Y, rayon, profondeur), en 4 temps :

    1. Isole les triangles de paroi quasi-horizontale (normale ~ perpendiculaire
       a Z) : `mesh.face_normals` decoupe un cylindre en dizaines de micro-
       facettes, ce ne sont jamais de vrais arcs de cercle, seulement des
       triangles plats. Ce filtre par normale locale laisse aussi passer des
       PORTIONS de cylindres non-verticaux (ex: un trou horizontal d'axe X a,
       a certains points de sa circonference, une normale locale sans
       composante Z) -- d'ou l'etape 2.
    2. Regroupe ces triangles par composantes connexes adjacentes, PUIS
       filtre chaque groupe par orientation d'axe (_axe_cylindre_groupe) :
       une fraiseuse 3 axes ne descend qu'a la verticale (Z), donc seuls les
       groupes dont l'axe est quasi-vertical sont conserves. Sans ce filtre,
       un trou horizontal (axe X ou Y) produit des fragments de paroi dont
       la projection XY est presque une droite -- un fit de cercle sur des
       points quasi-alignes degenere vers un rayon et un centre absurdement
       grands (des dizaines de milliers de mm), qui passent A TORT le
       controle de residu relatif (residu/rayon reste petit meme si le fit
       n'a physiquement aucun sens) et le controle de bord (les points bruts,
       eux, restent dans le brut). Filtrer par axe AVANT le fit de cercle
       elimine ce risque a la racine plutot que d'esperer le rattraper apres.
       La tessellation reelle d'un STEP (cascadio) peut aussi fragmenter la
       paroi d'un MEME trou vertical en plusieurs sous-groupes disjoints
       (facettes limites qui ratent de peu le seuil de tolerance), d'ou
       l'etape de fusion ci-dessous.
    3. Ajuste un cercle preliminaire par moindres carres (scipy.optimize.
       least_squares, cf. _ajuster_cercle_2d) sur chaque groupe vertical
       retenu, puis fusionne (_fusionner_fragments_cercle) les fragments
       dont le cercle coincide -- reconstituant une unique entite Cercle par
       percage physique reel, meme si le maillage source est fragmente.
    4. Filtre final (bord du brut, nombre de sommets, residu) sur chaque
       paroi fusionnee, puis construit la liste complete des Percage
       valides -- TOUS les trous verticaux detectes, pas seulement le premier.
    """
    normales = mesh.face_normals
    masque_parois = np.abs(normales[:, 2]) < TOLERANCE_PAROI_VERTICALE
    indices_parois = np.nonzero(masque_parois)[0]
    print(f"[step_parser] percages : {indices_parois.size} triangle(s) de paroi candidat(s)")

    if indices_parois.size == 0:
        return []

    groupes = _grouper_faces_adjacentes(mesh, indices_parois)

    # --- Filtrage par axe + fit preliminaire par groupe ---
    fragments: list[dict] = []
    nb_rejetes_axe = 0
    for groupe in groupes:
        if len(groupe) >= MIN_TRIANGLES_AXE:
            axe = _axe_cylindre_groupe(mesh, groupe)
            if abs(axe[2]) < 1.0 - TOLERANCE_AXE_VERTICAL:
                nb_rejetes_axe += 1
                continue   # cylindre non-vertical (axe X/Y) : hors capacite 3 axes
        # groupes trop petits pour juger l'axe de facon fiable : laisses
        # passer, geres par le fit/fusion et les filtres habituels en aval.

        vertex_ids = np.unique(mesh.faces[groupe])
        points = mesh.vertices[vertex_ids]
        if points.shape[0] < MIN_POINTS_ARC:
            continue   # trop peu de points pour contraindre un cercle
        cx, cy, rayon, _ = _ajuster_cercle_2d(points[:, :2])
        if rayon <= 0:
            continue
        fragments.append({"cx": cx, "cy": cy, "rayon": rayon, "points": points})

    if nb_rejetes_axe:
        print(
            f"[step_parser] percages : {nb_rejetes_axe} groupe(s) ignore(s) "
            f"(axe non-vertical -- trou horizontal/oblique, hors capacite 3 axes)"
        )

    fusions = _fusionner_fragments_cercle(fragments)
    print(
        f"[step_parser] percages : {len(groupes)} groupe(s) de facettes -> "
        f"{len(fragments)} fragment(s) exploitable(s) -> {len(fusions)} paroi(s) fusionnee(s)"
    )

    (x_min, y_min, _), (x_max, y_max, _) = mesh.bounds
    marge = max(TOLERANCE_Z_GROUPE * 10, 0.5)

    percages: list[Percage] = []
    for i, points in enumerate(fusions):
        xy_uniques = np.unique(np.round(points[:, :2], 3), axis=0)
        cx, cy, rayon, residu = _ajuster_cercle_2d(points[:, :2])

        touche_bord = (
            points[:, 0].min() <= x_min + marge
            or points[:, 0].max() >= x_max - marge
            or points[:, 1].min() <= y_min + marge
            or points[:, 1].max() >= y_max - marge
        )
        if touche_bord:
            print(
                f"[step_parser]   paroi {i}: REJETEE (touche le bord du brut) -- "
                f"centre=({cx:.2f},{cy:.2f}) rayon={rayon:.2f}"
            )
            continue  # paroi exterieure du brut, pas un percage interieur

        if xy_uniques.shape[0] < MIN_SOMMETS_CERCLE:
            print(
                f"[step_parser]   paroi {i}: REJETEE (seulement {xy_uniques.shape[0]} "
                f"sommet(s) distinct(s), minimum {MIN_SOMMETS_CERCLE}) -- "
                f"centre=({cx:.2f},{cy:.2f}) rayon={rayon:.2f}"
            )
            continue  # trop peu de sommets : polygone (ex. poche rectangulaire),
                       # pas une surface cylindrique tessellee

        if rayon <= 0 or residu > TOLERANCE_CERCLE_RATIO * max(rayon, 1e-6):
            print(
                f"[step_parser]   paroi {i}: REJETEE (residu de fit {residu:.4f} trop "
                f"eleve pour rayon={rayon:.2f}, forme non circulaire)"
            )
            continue  # forme non circulaire, on ignore

        z_haut = float(points[:, 2].max())
        z_bas = float(points[:, 2].min())

        # Occlusion : le trou doit etre OUVERT par le dessus. Un percage dont
        # l'entree (z_haut) est enfouie sous la matiere (ex : trou de flanc
        # vu depuis la face opposee) est hors d'atteinte d'un outil vertical.
        if z_haut < -TOLERANCE_Z_GROUPE:
            origine = np.array([[cx, cy, z_haut + 0.05]])
            direction = np.array([[0.0, 0.0, 1.0]])
            if bool(np.any(mesh.ray.intersects_any(
                ray_origins=origine, ray_directions=direction
            ))):
                print(
                    f"[step_parser]   paroi {i}: REJETEE (entree du trou enfouie "
                    f"sous la matiere, z_haut={z_haut:.2f}) -- "
                    f"centre=({cx:.2f},{cy:.2f}) rayon={rayon:.2f}"
                )
                continue

        print(
            f"[step_parser]   paroi {i}: ACCEPTEE -- centre=({cx:.2f},{cy:.2f}) "
            f"rayon={rayon:.2f} z=[{z_bas:.2f},{z_haut:.2f}]"
        )
        percages.append({
            "x": round(cx, 4),
            "y": round(cy, 4),
            "rayon": round(rayon, 4),
            "profondeur": round(z_bas - z_haut, 4),
            "z_depart": round(z_haut, 4),
        })

    return percages


def _fusionner_fragments_cercle(fragments: list[dict]) -> list[np.ndarray]:
    """Fusionne les fragments de paroi dont le cercle ajuste (centre + rayon)
    coincide, pour reconstituer une paroi cylindrique complete a partir de
    plusieurs sous-groupes de facettes disjoints issus d'une tessellation
    fragmentee.

    Composantes connexes sur le graphe de similarite "meme cercle" (deux
    fragments sont relies si _memes_cercle les juge compatibles).

    Args:
        fragments: Liste de dicts {"cx", "cy", "rayon", "points"}, un par
            groupe de facettes issu de _grouper_faces_adjacentes.

    Returns:
        Liste de tableaux de points (Nx3), un par percage physique
        reconstitue (union des points de tous ses fragments).
    """
    n = len(fragments)
    if n == 0:
        return []

    visites = [False] * n
    fusions: list[np.ndarray] = []

    for i in range(n):
        if visites[i]:
            continue
        visites[i] = True
        membres = [i]
        pile = [i]
        while pile:
            a = pile.pop()
            for b in range(n):
                if not visites[b] and _memes_cercle(fragments[a], fragments[b]):
                    visites[b] = True
                    pile.append(b)
                    membres.append(b)

        fusions.append(np.concatenate([fragments[m]["points"] for m in membres], axis=0))

    return fusions


def _memes_cercle(a: dict, b: dict) -> bool:
    """True si deux fragments de paroi partagent (a peu pres) le meme centre
    et le meme rayon -- signe qu'ils appartiennent au meme percage physique,
    simplement separes par la fragmentation du maillage.
    """
    distance_centres = math.hypot(a["cx"] - b["cx"], a["cy"] - b["cy"])
    return (
        distance_centres <= TOLERANCE_FUSION_CERCLE
        and abs(a["rayon"] - b["rayon"]) <= TOLERANCE_FUSION_CERCLE
    )


def _axe_cylindre_groupe(mesh: trimesh.Trimesh, groupe: np.ndarray) -> np.ndarray:
    """Estime la direction de l'axe central d'un groupe de facettes cylindriques.

    Pour un cylindre parfait, chaque normale de paroi est perpendiculaire a
    l'axe (elle pointe radialement, jamais le long de l'axe). L'axe est donc
    la direction de variance MINIMALE parmi les normales du groupe : le
    vecteur propre associe a la plus petite valeur propre de leur matrice de
    covariance (les normales n'ont, en theorie, aucune composante le long de
    l'axe -- variance nulle dans cette direction).

    Args:
        mesh:   Maillage source.
        groupe: Indices de triangles (meme groupe que _grouper_faces_adjacentes).

    Returns:
        Vecteur unitaire (3,) donnant la direction de l'axe (signe arbitraire :
        un cylindre n'a pas de "haut", seul |axe . Z| importe pour juger de
        la verticalite).
    """
    normales_groupe = mesh.face_normals[groupe]
    if normales_groupe.shape[0] < 2:
        return np.array([0.0, 0.0, 1.0])   # groupe degenere, defaut neutre (vertical)

    matrice_cov = np.cov(normales_groupe.T)
    valeurs_propres, vecteurs_propres = np.linalg.eigh(matrice_cov)
    axe = vecteurs_propres[:, 0]   # eigh trie croissant -> plus petite valeur propre en 1er

    norme = np.linalg.norm(axe)
    return axe / norme if norme > 1e-9 else np.array([0.0, 0.0, 1.0])


def _grouper_faces_adjacentes(
    mesh: trimesh.Trimesh, indices_candidats: np.ndarray
) -> list[np.ndarray]:
    """Regroupe par composantes connexes les triangles candidats (parois
    quasi-verticales) qui sont adjacents entre eux, afin de separer chaque
    paroi cylindrique individuelle.
    """
    candidats = set(indices_candidats.tolist())
    adjacence = mesh.face_adjacency

    paires_valides = [
        (a, b) for a, b in adjacence if a in candidats and b in candidats
    ]

    n_faces = len(mesh.faces)
    if paires_valides:
        lignes, colonnes = zip(*paires_valides)
    else:
        lignes, colonnes = (), ()

    matrice = coo_matrix(
        (np.ones(len(lignes)), (lignes, colonnes)),
        shape=(n_faces, n_faces),
    )
    _, labels = connected_components(matrice, directed=False)

    groupes: dict[int, list[int]] = {}
    for idx in indices_candidats:
        groupes.setdefault(int(labels[idx]), []).append(int(idx))

    return [np.array(g) for g in groupes.values()]


def _ajuster_cercle_2d(points_xy: np.ndarray) -> tuple[float, float, float, float]:
    """Ajuste un cercle 2D par moindres carres geometriques (methode classique
    scipy : minimise l'ecart-type des rayons au centre estime).

    Returns:
        (centre_x, centre_y, rayon, residu) ou residu est l'ecart-type des
        distances au centre (mesure de qualite du fit -- proche de 0 pour
        un vrai cylindre).
    """
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    centre_initial = (float(x.mean()), float(y.mean()))

    def residus(centre: np.ndarray) -> np.ndarray:
        rayons = np.sqrt((x - centre[0]) ** 2 + (y - centre[1]) ** 2)
        return rayons - rayons.mean()

    resultat = least_squares(residus, x0=centre_initial)
    cx, cy = resultat.x
    rayons = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)

    return float(cx), float(cy), float(rayons.mean()), float(rayons.std())
