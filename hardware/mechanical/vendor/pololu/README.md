# Pololu S13V20F5 vendor model (optional, not included)

Pololu publishes a STEP model of the 5 V, 2 A S13V20F5 step-up/step-down
regulator, item 4085, in the archive linked from the product resources page:

<https://www.pololu.com/product/4085/resources>

That file is Pololu's and is not redistributed in this repository. Nothing here
needs it: `generate_component_models.py` builds an envelope model with the same
outline and height, and the bracer designs use that.

To have the manufacturer's model available for local CAD work, download it and
save it in this directory as `s13v20f5_item4085.step`. The generator will then
also write `models/n203_pololu_s13v20f5_item4085_official.*`, recentred in X/Y
only. Both the downloaded file and those outputs are ignored by git. Review
Pololu's terms before redistributing them.
