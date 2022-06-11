# Custom Navmesh
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
- [X] Convex Polygon Generation
  - [X] Triangulate contours
  - [X] Merge triangles to form convex polygons
- [ ] Detail Mesh Generation
- [ ] Implement pathfinding
  - [X] Graph generation
  - [X] A*
  - [X] Implement simple point projection
  - [ ] Implement path smoothing
- [ ] Bake results
- [ ] Combine multiple navmesh bounds

## Improves
- [X] Refactor the distance field generation
- [ ] Replace unreal triangulation with custom one without using ear clipping
- [ ] Profile
- [X] Make navmesh generation asynchronous
- [X] Rebuild only the dirty area and not all the navmesh
- [ ] Store the polygons in an Octree
- [ ] Find proper way to check nearest point to a 3D polygon
- [ ] Convert debug macros to console variables for proper debugging
