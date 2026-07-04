"""Tests du moteur geometrique (offsets, pocketing concentrique, ilots)."""

import math

import pytest

import geometry


def _rectangle(largeur, hauteur, x0=0.0, y0=0.0):
    x1, y1 = x0 + largeur, y0 + hauteur
    return [
        (x0, y0, x1, y0),
        (x1, y0, x1, y1),
        (x1, y1, x0, y1),
        (x0, y1, x0, y0),
    ]


def _cercle(cx, cy, r, n=64):
    pts = [
        (cx + r * math.cos(2 * math.pi * i / n),
         cy + r * math.sin(2 * math.pi * i / n))
        for i in range(n + 1)
    ]
    return [
        (pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
        for i in range(n)
    ]


class TestOffsetContour:
    def test_offset_exterieur_agrandit(self):
        segs = geometry.calculer_offset_contour(_rectangle(50, 30), 3.0, "exterieur")
        xs = [s[0] for s in segs] + [s[2] for s in segs]
        assert min(xs) == pytest.approx(-3.0, abs=0.01)
        assert max(xs) == pytest.approx(53.0, abs=0.01)

    def test_offset_interieur_retrecit(self):
        segs = geometry.calculer_offset_contour(_rectangle(50, 30), 3.0, "interieur")
        xs = [s[0] for s in segs] + [s[2] for s in segs]
        assert min(xs) == pytest.approx(3.0, abs=0.01)
        assert max(xs) == pytest.approx(47.0, abs=0.01)

    def test_rayon_nul_retourne_inchange(self):
        contour = _rectangle(50, 30)
        assert geometry.calculer_offset_contour(contour, 0.0, "exterieur") == contour

    def test_rayon_negatif_leve(self):
        with pytest.raises(ValueError):
            geometry.calculer_offset_contour(_rectangle(50, 30), -1.0, "exterieur")

    def test_cote_invalide_leve(self):
        with pytest.raises(ValueError):
            geometry.calculer_offset_contour(_rectangle(50, 30), 1.0, "dessous")


class TestTrajectoiresPoche:
    def test_poche_rectangulaire_anneaux(self):
        anneaux = geometry.calculer_trajectoires_poche(_rectangle(60, 50), 3.0, 0.5)
        assert len(anneaux) > 3
        # premier anneau : retrait du rayon outil par rapport au contour
        xs = [s[0] for s in anneaux[0]] + [s[2] for s in anneaux[0]]
        assert min(xs) == pytest.approx(3.0, abs=0.01)
        assert max(xs) == pytest.approx(57.0, abs=0.01)

    def test_poche_trop_petite_vide(self):
        assert geometry.calculer_trajectoires_poche(_rectangle(4, 4), 3.0, 0.5) == []

    def test_stepover_invalide_leve(self):
        with pytest.raises(ValueError):
            geometry.calculer_trajectoires_poche(_rectangle(60, 50), 3.0, 1.5)

    def test_ilot_preserve(self):
        """Un contour interieur est un ilot : aucun anneau ne doit le raser."""
        contour = _rectangle(100, 80)
        ilot = _rectangle(20, 20, 40, 30)   # ilot central 20x20 en (40,30)
        anneaux = geometry.calculer_trajectoires_poche(contour + ilot, 3.0, 0.5)
        assert anneaux, "la poche avec ilot doit produire des trajectoires"
        # Aucune extremite de segment ne doit se trouver DANS l'ilot
        # (marge = rayon outil : le centre outil doit rester a >= 3mm du bord)
        for anneau in anneaux:
            for x1, y1, x2, y2 in anneau:
                for px, py in ((x1, y1), (x2, y2)):
                    dedans = (40 + 2.9 < px < 60 - 2.9) and (30 + 2.9 < py < 50 - 2.9)
                    assert not dedans, f"centre outil dans l'ilot : ({px:.2f},{py:.2f})"

    def test_anneaux_separes_pas_de_pont(self):
        """Chaque anneau retourne est une boucle continue : pas de saut entre
        deux zones disjointes a l'interieur d'un meme anneau."""
        anneaux = geometry.calculer_trajectoires_poche(_cercle(30, 30, 20), 3.0, 0.5)
        for anneau in anneaux:
            for i in range(1, len(anneau)):
                x_prec, y_prec = anneau[i - 1][2], anneau[i - 1][3]
                x_suiv, y_suiv = anneau[i][0], anneau[i][1]
                saut = math.hypot(x_suiv - x_prec, y_suiv - y_prec)
                assert saut < 5.0, f"pont de {saut:.1f}mm au sein d'un anneau"
