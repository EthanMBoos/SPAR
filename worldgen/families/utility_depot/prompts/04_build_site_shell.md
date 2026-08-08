# 04 — Build the site shell

Build only anchors whose `spar_plan_type` is `ground`, `fence`, or `gate`.

For every matching anchor, create one semantic root at the anchor and build a
recognizable blockout in its local coordinates:

- build the ground only with
  `spar.add_ground_from_boundary(root, spar.guide("PLAN_SiteBoundary"))`; this
  reads the complete 40×40 m size directly and must not be divided or scaled;
- each fence segment uses local posts, two rails, and a thin fence panel along
  its local length;
- the gate has two posts and visibly open gate leaves, leaving the planned
  vehicle opening clear.

Use restrained placeholder material colors. Keep component density low and do
not add chain-link detail yet. Finish in Solid Material Color mode and save
`<REPO>/artifacts/worldgen/<WORLD>/04_site_shell.blend`.
