"""
dxf_parser.py — Lecture et extraction des entités géométriques DXF.

Responsabilité unique : ouvrir un fichier DXF et retourner des
structures DxfEntity normalisées, sans aucune logique d'usinage.
"""

from __future__ import annotations

from pathlib import Path

try:
    import ezdxf
    from ezdxf.document import Drawing
except ImportError as exc:
    raise ImportError("ezdxf est requis : pip install ezdxf") from exc

from models import DxfEntity

SUPPORTED_ENTITY_TYPES = {"LINE", "ARC", "CIRCLE", "LWPOLYLINE", "SPLINE"}


# ---------------------------------------------------------------------------
# API publique
# ---------------------------------------------------------------------------

def test_import_dxf(filepath: str) -> None:
    """Fonction de test appelée par le bouton 'Importer DXF' de l'UI.

    Args:
        filepath: Chemin vers le fichier DXF sélectionné.
    """
    print(f"[dxf_parser] Fichier reçu : {filepath}")
    try:
        document = load_dxf_document(filepath)
        layers = list_layers(document)
        entities = extract_all_entities(document)
        print(f"[dxf_parser] Calques trouvés : {layers}")
        print(f"[dxf_parser] Entités extraites : {len(entities)}")
        for entity in entities[:5]:            # apercu des 5 premieres
            print(f"  |-- {entity.entity_type} sur calque '{entity.layer}'")
        if len(entities) > 5:
            print(f"  |-- ... et {len(entities) - 5} entite(s) de plus.")
    except Exception as exc:
        print(f"[dxf_parser] ERREUR : {exc}")


def extraire_geometrie(chemin_fichier: str) -> list[tuple]:
    """Ouvre un fichier DXF et extrait les coordonnées de toutes les lignes (LINE).

    Parcourt l'espace objet (modelspace) et retourne uniquement les entités
    de type LINE sous forme de tuples de coordonnées 2D.

    Args:
        chemin_fichier: Chemin absolu ou relatif vers le fichier .dxf

    Returns:
        Liste de tuples (x1, y1, x2, y2) pour chaque ligne trouvée.
        Retourne une liste vide si aucune ligne n'est présente.

    Raises:
        FileNotFoundError: loguée en console, retourne [].
        ezdxf.DXFStructureError: loguée en console, retourne [].
    """
    try:
        document = load_dxf_document(chemin_fichier)
    except FileNotFoundError:
        print(f"[dxf_parser] Fichier introuvable : {chemin_fichier}")
        return []
    except ezdxf.DXFStructureError as exc:
        print(f"[dxf_parser] Fichier DXF corrompu : {exc}")
        return []
    except Exception as exc:
        print(f"[dxf_parser] Erreur inattendue a l'ouverture : {exc}")
        return []

    lignes_extraites: list[tuple] = []
    modelspace = document.modelspace()

    for entite in modelspace:
        if entite.dxftype() != "LINE":
            continue
        x1, y1, _ = entite.dxf.start   # on ignore Z (travail 2D)
        x2, y2, _ = entite.dxf.end
        lignes_extraites.append((x1, y1, x2, y2))

    print(f"[dxf_parser] extraire_geometrie : {len(lignes_extraites)} ligne(s) extraite(s)")
    return lignes_extraites


def load_dxf_document(filepath: str | Path) -> Drawing:
    """Ouvre et retourne le document DXF sans filtrage.

    Args:
        filepath: Chemin absolu ou relatif vers le fichier .dxf

    Returns:
        Objet Drawing ezdxf.

    Raises:
        FileNotFoundError: si le fichier n'existe pas.
        ezdxf.DXFStructureError: si le fichier est corrompu.
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"Fichier DXF introuvable : {filepath}")
    return ezdxf.readfile(str(filepath))


def extract_entities_from_layer(
    document: Drawing,
    layer_name: str,
) -> list[DxfEntity]:
    """Extrait toutes les entités géométriques d'un calque donné.

    Args:
        document: Document DXF chargé via load_dxf_document.
        layer_name: Nom exact du calque à filtrer (sensible à la casse).

    Returns:
        Liste de DxfEntity pour les types supportés uniquement.
    """
    modelspace = document.modelspace()
    return [
        _normalize_entity(e, e.dxftype(), layer_name)
        for e in modelspace
        if e.dxf.layer == layer_name and e.dxftype() in SUPPORTED_ENTITY_TYPES
    ]


def extract_all_entities(document: Drawing) -> list[DxfEntity]:
    """Extrait toutes les entités géométriques de tous les calques.

    Args:
        document: Document DXF chargé via load_dxf_document.

    Returns:
        Liste complète de DxfEntity pour les types supportés.
    """
    modelspace = document.modelspace()
    return [
        _normalize_entity(e, e.dxftype(), e.dxf.layer)
        for e in modelspace
        if e.dxftype() in SUPPORTED_ENTITY_TYPES
    ]


def list_layers(document: Drawing) -> list[str]:
    """Retourne la liste des noms de calques présents dans le document.

    Args:
        document: Document DXF chargé.

    Returns:
        Liste triée des noms de calques.
    """
    return sorted(layer.dxf.name for layer in document.layers)


# ---------------------------------------------------------------------------
# Fonctions internes
# ---------------------------------------------------------------------------

def _normalize_entity(entity, entity_type: str, layer: str) -> DxfEntity:
    extractors = {
        "LINE": _extract_line_data,
        "ARC": _extract_arc_data,
        "CIRCLE": _extract_circle_data,
        "LWPOLYLINE": _extract_lwpolyline_data,
        "SPLINE": _extract_spline_data,
    }
    raw_data = extractors.get(entity_type, lambda e: {})(entity)
    return DxfEntity(entity_type=entity_type, layer=layer, raw_data=raw_data)


def _extract_line_data(entity) -> dict:
    return {"start": tuple(entity.dxf.start), "end": tuple(entity.dxf.end)}


def _extract_arc_data(entity) -> dict:
    return {
        "center": tuple(entity.dxf.center),
        "radius": entity.dxf.radius,
        "start_angle": entity.dxf.start_angle,
        "end_angle": entity.dxf.end_angle,
    }


def _extract_circle_data(entity) -> dict:
    return {"center": tuple(entity.dxf.center), "radius": entity.dxf.radius}


def _extract_lwpolyline_data(entity) -> dict:
    return {"points": list(entity.get_points()), "closed": entity.closed}


def _extract_spline_data(entity) -> dict:
    return {
        "control_points": [tuple(pt) for pt in entity.control_points],
        "degree": entity.dxf.degree,
    }
