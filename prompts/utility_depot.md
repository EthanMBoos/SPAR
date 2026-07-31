# Utility depot world prompts

Replace `<REPO>` with the absolute repository path and `<WORLD>` with a
lowercase world name. Open a new Blender scene, connect BlenderMCP, then send
these three prompts in order as separate messages.

## 1. Layout

```text
Build the complete spatial layout for a realistic outdoor utility depot in
Blender. You own every object position, rotation, grouping, aisle, and
sightline. Do not ask me for coordinates and do not use a fixed region grid.

Make exactly one BlenderMCP execute_blender_code call. Its Python program must
create every major object, apply transforms, set the cameras, and save the
result. Do not call an external asset service during this step.

Scene contract:

- Work in meters on an exactly 40 m by 40 m gravel site centered at the world
  origin. Keep real object scale: expand the layout and asset count, not the
  size of racks, barrels, pallets, or fencing components.
- Create a coherent, asymmetric industrial layout with a wide connected
  drive/patrol aisle and a clear central staging area.
- Include a perimeter chain-link fence with an open vehicle gate, at least
  five loaded storage racks, at least sixteen industrial barrels, pallets,
  cable spools, utility cabinets, a small shed or covered work area, and
  appropriate pipes, crates, bollards, or cones.
- Include exactly one conspicuous red inspection drum. Nothing else may be
  saturated red.
- Use procedural Blender geometry with recognizable silhouettes. This is a
  complete blockout, not final surfacing.
- Create collections named SITE, PROPS, DETAILS, CAMERAS, LIGHTS, and
  SPAR_SITES.
- In SPAR_SITES create arrow empties named SITE_Dock and SITE_HuskySpawn at
  (0, 0, 0), and SITE_X2Spawn at (-2, -1, 0). Give them spar_site values dock,
  husky_spawn, and x2_spawn respectively.
- Keep connected, at least 1.5 m wide drivable aisles between the dock and
  every major storage zone. Do not reserve a fixed patrol route; the waypoint
  pass will inspect the completed layout.
- Give every major object a unique descriptive name and a custom string
  property named spar_type. Use these values where applicable: ground, fence,
  gate, rack, barrel, pallet, cable_spool, utility_cabinet, structure, and
  inspection_target. Add spar_role="anomaly" only to the red inspection drum.
- Create cameras named CAM_Overview, CAM_Ground, and CAM_Detail with useful
  compositions. Set CAM_Overview active.
- Use Eevee, 1920 by 1080 output, AgX-style color management, and reasonable
  viewport performance.
- Apply object transforms, purge unused default data, and save the file to
  <REPO>/artifacts/worldgen/<WORLD>/layout.blend.

At the end, print the saved path, object count, count of each spar_type, and
the inspection target's name and location. Do not add final textures, download
assets, render images, or revise the layout in this call.
```

## 2. Detail and render

```text
Inspect the current Blender scene and preserve the accepted layout. Do not
move, rotate, add, or remove any major object carrying spar_type. Finish it as
a convincing real utility depot.

Use only Poly Haven CC0 downloads plus procedural Blender modeling. Find and
apply a real gravel PBR texture to the ground and a neutral outdoor industrial
HDRI. Do not use generated third-party models. Record the exact Poly Haven
asset IDs.

Improve the existing procedural assets in place: open steel racks with loaded
shelves, ribbed drums, chain-link fencing and posts, pallet slats, cable on
spools, cabinet doors and vents, believable shed construction, pipes, crates,
bollards, and restrained clutter. Add bevels, weathered metal, galvanized
steel, wood, plastic, dirt variation, tire marks, and small imperfections.
Keep geometry moderate enough for interactive Eevee use. Nothing except the
inspection drum may be saturated red.

Establish natural daylight using the HDRI plus a sun, refine the three cameras
without changing the layout, and render:

- <REPO>/artifacts/worldgen/<WORLD>/overview.png from CAM_Overview
- <REPO>/artifacts/worldgen/<WORLD>/ground.png from CAM_Ground
- <REPO>/artifacts/worldgen/<WORLD>/detail.png from CAM_Detail

Pack external resources into the blend file and save the final scene to
<REPO>/artifacts/worldgen/<WORLD>/final.blend. Write scene_manifest.json beside
it containing the Blender version, render engine, site dimensions, every
spar_type object's name, transform, dimensions, materials and custom
properties, camera transforms, and Poly Haven asset IDs.

Inspect the three renders yourself. Fix obvious intersections, floating parts,
broken materials, poor framing, implausible scale, or an empty composition.
Make no more than two repair passes and do not redesign the major layout.
Report the saved files, asset IDs, repairs, and known shortcomings.
```

## 3. Ground and air waypoints

This prompt is for the current explicit-waypoint ground and air behaviors.
Future behaviors that dynamically plan routes can skip it.

```text
Inspect the completed depot and add interesting, achievable ground and air
patrol routes. Do not move or change any visual object. Use one
execute_blender_code call.

In SPAR_SITES, create six to eight arrow empties named SITE_Patrol_01 onward.
Place them in order along connected drivable aisles so the Husky visits several
distinct storage zones and returns naturally toward the dock. Keep each marker
at least 0.75 m from visible obstacles and leave at least 1.5 m aisle width.
Orient each empty's local X axis in the desired arrival heading. Give each a
spar_site string beginning with patrol_, such as patrol_01, and a unique
integer spar_patrol_order starting at 0. Ensure the red inspection drum is
visible from within 8 m of at least one route marker.

Also create six to eight arrow empties named SITE_AirPatrol_01 onward. Give
them spar_site strings beginning with air_patrol_ and unique integer
spar_patrol_order values starting at 0. These are full 3D flight targets:
place them over several distinct depot zones, orient local X in the desired
camera heading, and keep every marker within the site boundary.

Use one common flight altitude that is at least 0.75 m above the tallest
visible obstacle and no higher than 10 m. The straight segments between air
markers, the closing segment, and direct returns to SITE_X2Spawn must remain
clear at that altitude. Keep a clear 1.2 m diameter vertical takeoff and
landing cylinder above SITE_X2Spawn. Keep a 2 m radius inspection orbit around
the red drum clear at the same altitude, and put at least one air marker within
12 m 3D distance of the drum with its heading facing the drum.

Save the result to <REPO>/artifacts/worldgen/<WORLD>/waypoints.blend. Print the
ordered ground and air waypoint names, positions, yaw angles, flight altitude,
and nearest visible obstacle. Do not add collision proxies, render, or revise
the visual scene.
```
