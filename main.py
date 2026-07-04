"""
main.py — Point d'entrée du logiciel FAO.

Lance uniquement la fenêtre CustomTkinter. Toute la logique métier
est dans les modules spécialisés (ui, dxf_parser, step_parser, geometry,
gcode_generator, models).

Le logiciel prend en charge deux flux d'import équivalents, tous deux
menant au même moteur de génération G-code et au même Bouclier de Sécurité :
  - DXF  (2D) : ui.FaoMainWindow._on_import_dxf  -> _on_generate_gcode_dxf
  - STEP (3D) : ui.FaoMainWindow._on_import_step -> _on_generate_gcode_step
                (step_parser extrait dimensions/features, traduits par
                gcode_generator.generer_gcode_depuis_step)
Le bouton "Générer G-code" de l'UI distribue automatiquement vers le bon
flux selon le dernier fichier importé -- aucune branche a ajouter ici.
"""

from ui import FaoMainWindow


def main() -> None:
    """Initialise et lance la boucle principale de l'application."""
    app = FaoMainWindow()
    try:
        app.mainloop()
    except KeyboardInterrupt:
        # Python 3.14 laisse remonter KeyboardInterrupt depuis mainloop()
        # au lieu de l'avaler. On ferme proprement sans traceback.
        try:
            app.destroy()
        except Exception:
            pass


if __name__ == "__main__":
    main()
