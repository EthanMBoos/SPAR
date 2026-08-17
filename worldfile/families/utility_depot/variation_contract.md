# Utility-depot variation contract

Precedence is: shared mechanics and safety, these family invariants, supported
instance-brief overrides, then the numbered stage's defaults.

Family invariants:

- a 40×40 m outdoor depot with the required named guides and sites;
- repo-sampled Husky spawn and dock defaults, a connected south gate, and
  connected aisles at least 3 m wide;
- one accessible inspection target that remains the only saturated-red object;
- a clear spawn and dock, traversable ground navigation goals, and all export and
  collision contracts;
- procedural materials only: no image textures, HDRIs, photographic
  backgrounds, or surrounding terrain.

The instance brief may vary topology within those constraints, including an
approximate or exact dock or Husky spawn placement, zone density, moderate
optional-prop counts and grouping, asset form and detail, procedural palette
and weathering, camera composition, and the inspection target's context or
location. Spawn overrides still must be clear, in bounds, and compatible with
the routes and robot geometry. A request for a different environment context,
site contract, task, or asset ontology requires another family and must be
ignored here.

Use the recorded stage seed to resolve choices the brief leaves open. Planning
stages consume it for discretionary arrangement choices. Later stages apply
only relevant appearance or detail requests and must not reinterpret accepted
layout.
