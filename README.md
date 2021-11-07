﻿# Custom Navmesh
Custom implementation of a Navigation Mesh using Unreal Engine as editor


## Setup (in progress)

- Add a supported agent in the project settings with the *Nav Data Class* as **NNNavMesh**
- Place a NNNavMesh in the world
  - It should be the only NavData placed in the world
- You character movement component should have the NNavMesh as the preferred NavData. To are many ways to do so.
  - Make you AI character inherit from NNCharacter
  - Make your movement component's character AI inherit from NNAvMovementComponent

## TO DO
- [ ] Voxelization
  - [X] Gather Geometry
  - [ ] Footprint the geometry
  - [ ] Implement heightfields
  - [ ] Implement open heightfields
- [ ] Region Generation
- [ ] Contour Generation
- [ ] Convex Polygon Generation
- [ ] Detail Mesh Generation
