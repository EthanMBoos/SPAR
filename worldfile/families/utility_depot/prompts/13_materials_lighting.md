# 13 — Materials and lighting

Make no asset-search or download calls. Do not use Poly Haven, image textures,
an HDRI, a photographic background, or a surrounding terrain plane.

In the execute call, preserve existing geometry and transforms. Create and
assign simple coherent procedural materials by semantic-root ancestry and
object purpose: neutral compacted-dirt color for the physical ground,
weathered metal, galvanized steel, wood, cable, pipe, restrained plastic, and
safety markings. Use Principled BSDF values only; do not create or load image
nodes. Set both node colors and diffuse colors. Keep the inspection drum as the
only saturated-red object. Preserve the existing Blender World, its node tree,
background, color, strength, and film settings without modification. Add
exactly one sun for scene lighting. Do not add presentation geometry or
scene-level external-asset metadata.

Finish in Material Preview with scene lights and world, and save
`<REPO>/artifacts/worldgen/<WORLD>/materials.blend`.
