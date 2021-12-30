﻿# Custom Navmesh
Custom implementation of a Navigation Mesh using Unreal Engine as editor


## Setup (in progress)

- Add a supported agent in the project settings with the *Nav Data Class* as **NNNavMesh**
- Place a NNNavMesh in the world
  - It should be the only NavData placed in the world
- You character movement component should have the NNavMesh as the preferred NavData. There are many ways to do so.
  - Make you AI character inherit from NNCharacter
  - Make your movement component's character AI inherit from NNAvMovementComponent
  - Override the prefered Navdata from your movement comonent's character AI yourself

## TO DO
- [X] Voxelization
  - [X] Gather Geometry
  - [X] Footprint the geometry
  - [X] Implement heightfields
  - [X] Identify walkable spans
  - [X] Implement an open heightfield
    - [X] Create the open spans with the solid spans
    - [X] Collect the open spans walkable neighbours
- [X] Region Generation
  - [X] Implement the water shed algorithm
  - [X] Filter small regions
  - [X] Clear null region borders
- [X] Contour Generation
  - [X] Implement contour generation
  - [X] Simplify the contour
- [ ] Convex Polygon Generation
  - [X] Triangulate contours
  - [ ] Merge triangles to form convex polygons
- [ ] Detail Mesh Generation
- [ ] Implement pathfinding

## Improves
- [ ] Refactor the distance field generation
- [ ] Profile
