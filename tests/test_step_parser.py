"""Tests du parseur STEP : fit de cercles, filtres, unites, et regression
complete sur un fichier STEP reel (tests/data/featuretype.STEP, piece NIST
de 5x2.5x1.375 pouces avec 29 percages verticaux et un trou lateral)."""

import math
import pathlib

import numpy as np
import pytest
import trimesh

import step_parser as sp

FICHIER_REEL = pathlib.Path(__file__).parent / "data" / "featuretype.STEP"


# ---------------------------------------------------------------------------
# Unites (cascadio convertit toujours en metres -> reconversion mm)
# ---------------------------------------------------------------------------

class TestConversionUnites:
    def test_metres_convertis_en_mm(self):
        mesh = trimesh.creation.box(extents=[0.1, 0.08, 0.02])
        mesh.units = "meters"
        sp._convertir_en_millimetres(mesh)
        assert mesh.extents[0] == pytest.approx(100.0)

    def test_mm_non_rescales(self):
        mesh = trimesh.creation.box(extents=[100, 80, 20])
        mesh.units = "millimeters"
        sp._convertir_en_millimetres(mesh)
        assert mesh.extents[0] == pytest.approx(100.0)

    def test_heuristique_sans_unite_petit_maillage(self):
        mesh = trimesh.creation.box(extents=[0.1, 0.08, 0.02])
        mesh.units = None
        sp._convertir_en_millimetres(mesh)
        assert mesh.extents[0] == pytest.approx(100.0)

    def test_heuristique_sans_unite_maillage_normal(self):
        mesh = trimesh.creation.box(extents=[100, 80, 20])
        mesh.units = None
        sp._convertir_en_millimetres(mesh)
        assert mesh.extents[0] == pytest.approx(100.0)


# ---------------------------------------------------------------------------
# Ajustement de cercle et fusion de fragments
# ---------------------------------------------------------------------------

class TestFitCercle:
    def test_cercle_parfait(self):
        angles = np.linspace(0, 2 * math.pi, 48, endpoint=False)
        pts = np.column_stack([50 + 10 * np.cos(angles), 40 + 10 * np.sin(angles)])
        cx, cy, r, residu = sp._ajuster_cercle_2d(pts)
        assert (cx, cy, r) == (pytest.approx(50), pytest.approx(40), pytest.approx(10))
        assert residu < 1e-9

    def test_arc_partiel(self):
        angles = np.linspace(0, math.pi / 2, 20)   # quart de cercle seulement
        pts = np.column_stack([50 + 10 * np.cos(angles), 40 + 10 * np.sin(angles)])
        cx, cy, r, _ = sp._ajuster_cercle_2d(pts)
        assert r == pytest.approx(10, abs=0.01)

    def test_memes_cercle(self):
        a = {"cx": 50.0, "cy": 40.0, "rayon": 10.0}
        b = {"cx": 50.2, "cy": 40.1, "rayon": 10.1}
        c = {"cx": 70.0, "cy": 40.0, "rayon": 10.0}
        assert sp._memes_cercle(a, b)
        assert not sp._memes_cercle(a, c)


class TestFacesCouvertesParPercages:
    def test_fond_de_trou_retire(self):
        percage = {"x": 50.0, "y": 40.0, "rayon": 5.0,
                   "profondeur": -10.0, "z_depart": 0.0}
        fond = {"z": -10.0, "aire": 78.0, "centre": (50.0, 40.0),
                "min_xy": (45.1, 35.1), "max_xy": (54.9, 44.9), "contours": []}
        assert sp._retirer_faces_couvertes_par_percages([fond], [percage]) == []

    def test_face_distincte_conservee(self):
        percage = {"x": 50.0, "y": 40.0, "rayon": 5.0,
                   "profondeur": -10.0, "z_depart": 0.0}
        poche = {"z": -10.0, "aire": 600.0, "centre": (20.0, 20.0),
                 "min_xy": (10.0, 10.0), "max_xy": (30.0, 30.0), "contours": []}
        assert sp._retirer_faces_couvertes_par_percages([poche], [percage]) == [poche]


# ---------------------------------------------------------------------------
# Regression sur fichier STEP reel
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def features_reelles():
    return sp.analyser_features_3d(str(FICHIER_REEL))


class TestFichierReel:
    def test_dimensions_en_mm(self):
        dims = sp.extraire_dimensions_brut(str(FICHIER_REEL))
        # Piece de 5 x 2.5 x 1.375 pouces : la conversion metres->mm doit
        # restituer les cotes reelles, pas des valeurs 1000x trop petites.
        assert dims["x"] == pytest.approx(127.0, abs=0.1)
        assert dims["y"] == pytest.approx(63.5, abs=0.1)
        assert dims["z"] == pytest.approx(34.925, abs=0.1)

    def test_repere_recale(self):
        dims = sp.extraire_dimensions_brut(str(FICHIER_REEL))
        assert dims["min_corner"][0] == pytest.approx(0.0, abs=1e-6)
        assert dims["min_corner"][1] == pytest.approx(0.0, abs=1e-6)
        assert dims["max_corner"][2] == pytest.approx(0.0, abs=1e-6)

    def test_nombre_de_percages(self, features_reelles):
        assert len(features_reelles["percages"]) == 29

    def test_percages_verticaux_seulement(self, features_reelles):
        # Tous ouverts par le dessus, jamais enfouis
        for p in features_reelles["percages"]:
            assert p["z_depart"] <= 0.0
            assert p["profondeur"] < 0.0

    def test_faces_planes_machinables(self, features_reelles):
        faces = features_reelles["faces_planes"]
        assert len(faces) == 6
        for f in faces:
            assert f["z"] < 0.0                      # jamais la face superieure
            assert f["z"] > -34.9                     # jamais le dessous du brut
            assert len(f.get("contours", [])) >= 1    # contour reel extrait

    def test_silhouette_reelle(self, features_reelles):
        s = features_reelles["silhouette"]
        assert s is not None and len(s) >= 4
        xs = [p[0] for p in s]
        ys = [p[1] for p in s]
        assert max(xs) - min(xs) == pytest.approx(127.0, abs=0.2)
        assert max(ys) - min(ys) == pytest.approx(63.5, abs=0.2)


@pytest.fixture(scope="module")
def multi():
    return sp.analyser_features_multi_faces(str(FICHIER_REEL))


class TestMultiFaces:
    def test_orientation_principale_presente(self, multi):
        assert "Z_PLUS" in multi
        assert len(multi["Z_PLUS"]["percages"]) == 29

    def test_trou_lateral_detecte(self, multi):
        # Le trou lateral D9.9 traversant en Y doit apparaitre dans UNE
        # orientation laterale (Y_PLUS prioritaire), une seule fois.
        lateraux = [
            p for nom in ("Y_PLUS", "Y_MINUS") if nom in multi
            for p in multi[nom]["percages"]
        ]
        assert len(lateraux) == 1
        assert 2 * lateraux[0]["rayon"] == pytest.approx(9.9, abs=0.1)

    def test_deduplication_inter_faces(self, multi):
        # Les lamages vus du dessous (Z_MINUS) sont les memes cylindres que
        # ceux usines depuis le dessus : l'orientation doit disparaitre.
        assert "Z_MINUS" not in multi

    def test_dimensions_tournees(self, multi):
        if "Y_PLUS" in multi:
            dims = multi["Y_PLUS"]["dimensions"]
            assert dims["z"] == pytest.approx(63.5, abs=0.1)   # profondeur = ancien Y
