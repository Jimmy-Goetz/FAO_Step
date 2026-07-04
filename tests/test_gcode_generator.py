"""Tests du generateur G-code : Bouclier de Securite, selection automatique
d'outils, arcs G02 pour les percages, generation multi-faces et post-
processeurs."""

import pytest

import gcode_generator as gg
from models import MachiningConfig, ToolMagazine, ToolParams


def _config(outils=None, **surcharges):
    mag = ToolMagazine()
    for t in outils or [
        ToolParams(1, 6.0, 18000, 1200, 300, tool_length=40.0),
        ToolParams(2, 4.0, 20000, 800, 150, tool_length=40.0),
    ]:
        mag.add(t)
    defauts = dict(
        magazine=mag, tool_number_contour=1, tool_number_poche=1,
        tool_number_drill=1, machine_type="ISO_Standard",
        cnc_limit_x=600.0, cnc_limit_y=400.0,
    )
    defauts.update(surcharges)
    return MachiningConfig(**defauts)


def _features(percages=None, faces=None, silhouette=None):
    return {
        "faces_planes": faces or [],
        "percages": percages or [],
        "silhouette": silhouette,
    }


TROU_D8 = {"x": 50.0, "y": 40.0, "rayon": 4.0, "profondeur": -10.0, "z_depart": 0.0}


# ---------------------------------------------------------------------------
# Bouclier de Securite
# ---------------------------------------------------------------------------

class TestBouclierSecurite:
    def test_brut_trop_grand_pour_la_table(self):
        cfg = _config(cnc_limit_x=100.0)
        cfg.definir_brut_depuis_step({"x": 200.0, "y": 80.0, "z": 20.0})
        with pytest.raises(gg.SecritePhysiqueError, match="X"):
            gg.generer_gcode_depuis_step(_features(), cfg)

    def test_outil_trop_court_pour_le_brut(self):
        outils = [ToolParams(1, 6.0, 18000, 1200, 300, tool_length=10.0)]
        cfg = _config(outils=outils)
        cfg.definir_brut_depuis_step({"x": 100.0, "y": 80.0, "z": 30.0})
        with pytest.raises(gg.SecritePhysiqueError, match="mandrin"):
            gg.generer_gcode_depuis_step(_features(), cfg)

    def test_stock_absent_leve_valueerror(self):
        with pytest.raises(ValueError, match="stock_dimensions"):
            gg.generer_gcode_depuis_step(_features(), _config())

    def test_percage_auto_trop_profond(self):
        outils = [ToolParams(1, 6.0, 18000, 1200, 300, tool_length=40.0)]
        cfg = _config(outils=outils)
        cfg.definir_brut_depuis_step({"x": 100.0, "y": 80.0, "z": 30.0})
        trou_profond = dict(TROU_D8, profondeur=-60.0)
        # profondeur du trou (60) > longueur utile (40), brut lui-meme OK
        cfg2 = _config(outils=[ToolParams(1, 6.0, 18000, 1200, 300, tool_length=35.0)])
        cfg2.definir_brut_depuis_step({"x": 100.0, "y": 80.0, "z": 30.0})
        with pytest.raises(gg.SecritePhysiqueError):
            gg.generer_gcode_depuis_step(
                _features(percages=[dict(TROU_D8, profondeur=-38.0)]), cfg2,
            )


# ---------------------------------------------------------------------------
# Selection automatique d'outil
# ---------------------------------------------------------------------------

class TestSelectionOutil:
    def test_correspondance_exacte(self):
        cfg = _config()
        outil, raison = gg._choisir_outil_forme(4.0, cfg)   # T2 = D4.0
        assert outil.tool_number == 2
        assert "exacte" in raison

    def test_plus_grand_sous_la_forme(self):
        cfg = _config()
        outil, _ = gg._choisir_outil_forme(10.0, cfg)   # pas d'exact -> D6
        assert outil.tool_diameter == 6.0

    def test_repli_outil_poche(self):
        cfg = _config()
        outil, raison = gg._choisir_outil_forme(1.0, cfg)   # rien sous 1mm
        assert "repli" in raison

    def test_contournage_plus_grand_diametre(self):
        cfg = _config()
        outil, _ = gg._choisir_outil_contour_auto(cfg)
        assert outil.tool_diameter == 6.0


# ---------------------------------------------------------------------------
# Generation STEP : arcs, cycles, formes reelles
# ---------------------------------------------------------------------------

class TestGenerationStep:
    def _programme(self, percages=None, faces=None, silhouette=None, **cfg_kw):
        cfg = _config(**cfg_kw)
        cfg.definir_brut_depuis_step({"x": 100.0, "y": 80.0, "z": 20.0})
        return gg.generer_gcode_depuis_step(
            _features(percages, faces, silhouette), cfg,
        )

    def test_trou_fraise_en_arcs_g02(self):
        prog = self._programme(percages=[TROU_D8])
        assert "G02" in prog
        assert "FRAISAGE CIRCULAIRE" in prog
        # jamais de plongee verticale seche : chaque G02 de descente porte X/Y
        for ligne in prog.splitlines():
            if ligne.startswith("G01 Z") or ligne.startswith("G02 Z"):
                pytest.fail(f"plongee verticale detectee : {ligne}")

    def test_outil_au_diametre_percage_g83(self):
        outils = [
            ToolParams(1, 6.0, 18000, 1200, 300, tool_length=40.0),
            ToolParams(2, 8.0, 12000, 400, 120, tool_length=40.0),   # = D du trou
        ]
        prog = self._programme(percages=[TROU_D8], outils=outils)
        assert "G83" in prog          # helice impossible -> cycle de percage
        assert "PERCAGE CYCLE" in prog

    def test_silhouette_reelle_utilisee(self):
        # silhouette en L : le contournage doit suivre ses sommets, pas la bbox
        silhouette = [(0, 0), (100, 0), (100, 40), (50, 40), (50, 80), (0, 80)]
        prog = self._programme(silhouette=silhouette)
        assert "X103.000" in prog          # offset exterieur du coin (100,*)
        assert "X143.000" not in prog      # pas la bbox du brut

    def test_face_contour_reel(self):
        face = {
            "z": -5.0, "aire": 600.0, "centre": (30.0, 25.0),
            "min_xy": (10.0, 10.0), "max_xy": (50.0, 40.0),
            "contours": [[(10, 10), (50, 10), (50, 40), (10, 40)]],
        }
        prog = self._programme(faces=[face])
        assert "POCKETING" in prog

    def test_commentaire_retournement(self):
        cfg = _config()
        cfg.definir_brut_depuis_step({"x": 100.0, "y": 80.0, "z": 20.0})
        prog = gg.generer_gcode_depuis_step(
            _features(percages=[TROU_D8]), cfg,
            commentaires_entete=["OPERATION RETOURNEMENT PIECE : face X_PLUS"],
        )
        assert "RETOURNEMENT" in prog


# ---------------------------------------------------------------------------
# Multi-faces et 4 axes
# ---------------------------------------------------------------------------

def _features_multi():
    return {
        "Z_PLUS": {
            "faces_planes": [], "percages": [TROU_D8],
            "dimensions": {"x": 100.0, "y": 80.0, "z": 20.0},
            "silhouette": None,
        },
        "X_PLUS": {
            "faces_planes": [], "percages": [dict(TROU_D8, x=10.0, y=40.0)],
            "dimensions": {"x": 20.0, "y": 80.0, "z": 100.0},
        },
    }


class TestMultiFaces:
    def test_3_axes_un_fichier_par_face(self):
        gcodes, erreurs = gg.generer_gcodes_multi_faces(
            _features_multi(), _config(), mode="3_axes",
        )
        assert set(gcodes) == {"Z_PLUS", "X_PLUS"}
        assert erreurs == {}
        assert "RETOURNEMENT" in gcodes["X_PLUS"]
        assert "RETOURNEMENT" not in gcodes["Z_PLUS"]
        assert "CONTOURNAGE" in gcodes["Z_PLUS"]
        assert "CONTOURNAGE" not in gcodes["X_PLUS"]   # detourage 1 seule fois

    def test_3_axes_face_impossible_isolee(self):
        features = _features_multi()
        features["X_PLUS"]["percages"][0]["profondeur"] = -90.0   # > tout outil
        gcodes, erreurs = gg.generer_gcodes_multi_faces(features, _config(), mode="3_axes")
        assert "Z_PLUS" in gcodes          # les autres faces sortent quand meme
        assert "X_PLUS" in erreurs

    def test_4_axes_fichier_unique_avec_rotations(self):
        gcodes, erreurs = gg.generer_gcodes_multi_faces(
            _features_multi(), _config(), mode="4_axes",
        )
        assert list(gcodes) == ["4_AXES_A"]
        prog = gcodes["4_AXES_A"]
        assert "G00 A0" in prog
        assert "M00" in prog               # X_PLUS hors axe A -> pause manuelle
        assert prog.count("M30") == 1      # un seul pied de programme


# ---------------------------------------------------------------------------
# Post-processeurs
# ---------------------------------------------------------------------------

class TestPostProcesseurs:
    def _programme(self, machine_type):
        cfg = _config(machine_type=machine_type)
        cfg.definir_brut_depuis_step({"x": 100.0, "y": 80.0, "z": 20.0})
        return gg.generer_gcode_depuis_step(_features(percages=[TROU_D8]), cfg)

    def test_haas_marqueurs_bande(self):
        prog = self._programme("Haas_Fanuc")
        lignes = prog.splitlines()
        assert lignes[0] == "%" and lignes[-1] == "%"
        assert "O0001" in prog and "G43 H" in prog

    def test_grbl_changement_manuel(self):
        prog = self._programme("GRBL_CNC")
        assert "M00" in prog and "M06" not in prog

    def test_iso_g17_pour_les_arcs(self):
        prog = self._programme("ISO_Standard")
        assert "G17" in prog and "M30" in prog
