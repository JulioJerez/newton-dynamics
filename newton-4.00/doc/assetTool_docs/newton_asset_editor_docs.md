# Newton Asset Editor — Code Documentation

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Core Classes](#core-classes)
   - [ndAssetEditor](#ndasseteditor)
   - [ndUndoRedo & ndUndoRedoCommand](#ndundoredo--ndundoredocommand)
   - [ndDebugDisplayRenderPass](#nddebugdisplayrenderpass)
   - [ndEditorCameraFlyby](#ndeditorcameraflyby)
   - [ndMenuRenderPass](#ndmenurenderpass)
4. [Subsystems](#subsystems)
   - [Scene Management](#scene-management)
   - [Selection Model](#selection-model)
   - [Properties Panel](#properties-panel)
   - [Outliner Panel](#outliner-panel)
   - [Collision Editing](#collision-editing)
   - [Joint Editing](#joint-editing)
   - [Loop Joints](#loop-joints)
   - [Colliding Pairs](#colliding-pairs)
   - [Rigid Body Editing](#rigid-body-editing)
   - [Mesh Node Editing](#mesh-node-editing)
5. [Tools](#tools)
   - [ndResizeMesh](#ndresizemesh)
   - [ndRotateMesh / ndRotateBones / ndRotatePivots](#ndrotatemesh--ndrotatebones--ndrotatepivots)
   - [ndNomalizeMassDistribution](#ndnomalizemassdistribution)
6. [File I/O](#file-io)
   - [URDF Import](#urdf-import)
   - [File Browser Utilities](#file-browser-utilities)
7. [Render Pipeline](#render-pipeline)
8. [Data Flow Diagrams](#data-flow-diagrams)

---

## Overview

The Newton Asset Editor is a desktop GUI application built on top of the Newton Game Dynamics physics engine and a custom OpenGL renderer. It allows artists and engineers to author articulated rigid-body models (`.nd` files) that can be loaded directly into Newton physics simulations.

Key capabilities:

- Load/save Newton native `.nd` mesh files; import FBX and URDF formats.
- Inspect and edit node transforms, geometry, rigid bodies, collision shapes, structural joints, loop (closed-chain) joints, and colliding-pair exclusion lists.
- Unlimited undo/redo for all editing operations.
- Fly-by camera with orbit, pan, zoom, and orthographic/perspective toggle.
- Per-node debug overlay: collision shapes, joint gizmos, centre-of-mass, skeleton bones.
- Batch mesh tools: resize, rotate, normalise mass distribution.

The application entry point is `main.cpp`, which constructs an `ndAssetEditor` and calls `Run()`.

---

## Architecture

```
main.cpp
  └── ndAssetEditor          ← application singleton
        ├── ndRender          ← OpenGL window + render pipeline
        │     ├── ndRenderPassShadows
        │     ├── ndRenderPassColor
        │     ├── ndRenderPassEnvironment
        │     ├── ndDebugDisplayRenderPass   ← debug overlays & gizmos
        │     └── ndMenuRenderPass           ← Dear ImGui UI
        ├── ndEditorCameraFlyby              ← camera controller
        ├── ndMesh (scene graph root)        ← logical model data
        ├── ndRenderSceneNode (entity root)  ← visual scene graph
        └── ndUndoRedo                       ← command history
```

The logical model (`ndMesh` tree) and the visual scene graph (`ndRenderSceneNode` tree) are kept in sync manually. Every structural mutation pushes a pair of `ndUndoRedoCommand` snapshots (before and after) onto the undo stack.

---

## Core Classes

### ndAssetEditor

**File:** `ndAssetEditor.h` / `ndAssetEditor.cpp`

The central application class. Owns all top-level state and orchestrates the render loop.

#### Key Members

| Member | Type | Purpose |
|--------|------|---------|
| `m_mesh` | `ndSharedPtr<ndMesh>` | Root of the active logical scene graph |
| `m_entity` | `ndSharedPtr<ndRenderSceneNode>` | Root of the active visual scene graph |
| `m_renderer` | `ndSharedPtr<ndRender>` | OpenGL renderer |
| `m_currentSelection` | `ndWeakPtr<ndMesh>` | Currently selected mesh node |
| `m_currentSubSelection` | `ndWeakPtr<ndMesh>` | Secondary selection (loop joint target, colliding pair partner, etc.) |
| `m_subSelection` | `ndSubSelectionMode` | Current secondary-selection mode |
| `m_undoRedo` | `ndUndoRedo` | Undo/redo command stack |
| `m_currentTool` | `ndSharedPtr<ndAssetTool>` | Active modal tool (resize, rotate, …) |
| `m_renderMode` | `ndInt32` | `m_shaded`, `m_wireframe`, or `m_hiddenSurface` |
| `m_initCamera` | `bool` | When `true`, auto-fit camera on next scene load |

#### Enumerations

```cpp
enum ndCameraMode    { m_free, m_backView, m_frontView, m_sideLeftView, m_sideRrightView };
enum ndRenderModes   { m_shaded, m_wireframe, m_hiddenSurface };
enum ndSubSelectionMode { m_none, m_loopJoint, m_collidingPair, m_alignToTarget, m_transformModifier };
```

#### Render Loop

```
Run()
  while (!ShouldFinish)
    PollEvents()
    → if new mesh pending  → swap in m_newMesh / m_newSceneMesh
    RenderScene()
      camera->TickUpdate()
      BegingRender()
      ConfigureDockSpace()   ← full-screen ImGui docking frame
      Render()               ← executes all render passes
        ndMenuRenderPass::RenderScene()
          RenderLayout()
            ShowMainMenuBar()
            ShowMainToolbar()
            ShowOutlierPanel()
            ShowPropertiesPanel()
            m_currentTool->Execute()   (if active)
      EndRender() / Present()
```

#### SetVisualScene

```cpp
void SetVisualScene(const ndSharedPtr<ndMesh>& mesh,
                    const ndSharedPtr<ndRenderSceneNode>& renderMesh);
```

Stages a new scene for the next frame. Resets all selections and sub-selections. Counts bones to decide whether bone-raycast or mesh-raycast selection mode should be active.

#### SelectCurrentNode

```cpp
void SelectCurrentNode(ndSharedPtr<ndMesh> node);
```

Routes a node click to the correct handler based on `m_subSelection`:

| `m_subSelection` | Action |
|-----------------|--------|
| `m_none` | Sets `m_currentSelection` |
| `m_loopJoint` | Calls `SetLoopJointSelection()` |
| `m_collidingPair` | Calls `SetCollidingSubSelection()` |
| `m_transformModifier` | Calls `SetModifierSubSelection()` |
| `m_alignToTarget` | Sets `m_currentSubSelection` |

#### Inner Classes

**`ndRenderCallback`** — bridges the renderer's input callbacks to `ndAssetEditor` member functions (key, char, cursor, scroll, mouse button).

**`ndAssetTool`** — abstract base for modal tools. Subclasses implement `Execute()`, which is called once per frame while the tool is active. The tool sets `m_toolActive = false` to dismiss itself.

---

### ndUndoRedo & ndUndoRedoCommand

**Files:** `ndUndoRedo.h` / `ndUndoRedo.cpp`

#### ndUndoRedoCommand (abstract)

Base class for all undoable operations.

| Member | Purpose |
|--------|---------|
| `m_selectedNodeName` | Name of the node this command was recorded for |
| `m_selectedNode` | Weak pointer to that node |
| `m_editor` | Weak pointer back to the editor |
| `Undo()` | Restore state to what this snapshot captured |
| `operator!=()` | Equality check — used to deduplicate consecutive identical commands |

Concrete subclasses (all defined in their respective `.cpp` files):

| Class | What it captures |
|-------|-----------------|
| `ndUndoRedoMeshNode` | Full clone of the entire mesh tree (used for transform, rename, add/delete node, tool operations) |
| `ndUndoRedoRigidBody` | Duplicate of a single `ndMeshBody` |
| `ndUndoRedoShape` | Copy of a single `ndMeshShapeInstance` |
| `ndUndoRedoStructuralJoint` | Duplicate of a single `ndMeshJoint` |
| `ndUndoRedoLoopJoint` | Snapshot of the entire loop-joint list |
| `ndUndoRedoCollidingPairs` | Snapshot of the entire colliding-pair list |

#### ndUndoRedo

A doubly-linked list of `ndSharedPtr<ndUndoRedoCommand>` with a cursor (`m_currentCommand`).

```cpp
void Push(const ndSharedPtr<ndUndoRedoCommand>& command);
void Undo(ndAssetEditor* const owner);
void Redo(ndAssetEditor* const owner);
void Clear();
```

**Push pattern** — every editable operation pushes two commands: one immediately before the mutation and one immediately after. The `operator!=` check prevents duplicate entries when the value did not actually change.

```cpp
m_undoRedo.Push(before_snapshot);
// ... mutate data ...
m_undoRedo.Push(after_snapshot);
```

**Undo/Redo** step the cursor and call `Undo()` on the command at the new cursor position, then restore `m_currentSelection` by name lookup.

---

### ndDebugDisplayRenderPass

**Files:** `ndDebugDisplayRenderPass.h` / `ndDebugDisplayRenderPass.cpp`

Inherits from `ndRenderPassDebug`. Renders all editor overlays on top of the 3D scene.

#### Responsibilities

- Wireframe and hidden-surface overlays for all mesh nodes.
- Collision shape wireframes (yellow by default, red/blue for colliding pairs).
- Joint gizmos (coordinate-frame arrows at joint pivot locations).
- Skeleton bone geometry (octahedral diamond shapes).
- Close-loop joint frames.
- Colliding-pair highlights.
- Centre-of-mass and shape-pivot markers.

#### Key Methods

| Method | Description |
|--------|-------------|
| `ResetScene()` | Rebuilds `m_debugMesh` list from scratch after a scene change |
| `RebuildDebugCollision()` | Refreshes the collision-shape render primitive for the currently selected node after shape parameter changes |
| `RebuildVisualDebugMesh()` | Refreshes the visual wireframe primitive for the selected node after geometry rebuild |
| `RenderScene()` | Entry point; dispatches to `RenderBoneSelection()` or `RenderMeshSelection()` |
| `DrawFrame()` | Draws an RGB axis gizmo at a given matrix position |
| `DrawBone()` / `DrawSelectedBone()` | Generates and draws octahedral bone geometry |

#### ndDebugMesh

Per-node render data cached in `m_debugMesh`:

```cpp
struct ndDebugMesh {
    ndWeakPtr<ndRenderSceneNode> m_parent;
    ndSharedPtr<ndRenderPrimitive> m_zBufferMesh;    // hidden-surface pass mesh
    ndSharedPtr<ndRenderPrimitive> m_zBufferShape;   // hidden-surface pass collision
    ndSharedPtr<ndRenderPrimitive> m_wireFrameMesh;  // wireframe mesh
    ndSharedPtr<ndRenderPrimitive> m_flatShadedMesh; // flat-shaded mesh
    ndSharedPtr<ndRenderPrimitive> m_wireFrameShape; // wireframe collision
};
```

---

### ndEditorCameraFlyby

**Files:** `ndEditorCameraFlyby.h` / `ndEditorCameraFlyby.cpp`

A first-person style camera controller attached as the scene camera.

#### Controls

| Mouse button | Action |
|-------------|--------|
| Left drag | Orbit (yaw / pitch) — also triggers node selection on first click |
| Right drag | Pan (Y / Z) |
| Middle drag | Zoom (field-of-view scale) |
| Scroll wheel | (handled by ImGui, not this class) |

Camera orientation is stored as separate `m_yaw` and `m_pitch` angles to avoid quaternion gimbal lock. `CalculateCameraMatrix()` reconstructs the full matrix and also updates the directional light to follow the camera.

#### SetView

```cpp
void SetView(ndAssetEditor::ndCameraMode mode);
```

Snaps the camera to one of the orthographic preset orientations (back, front, left side, right side).

#### MouseSelection

Fires a ray from screen-space mouse coordinates into world space. In bone mode, does a ray-to-ray closest-point test against each bone segment. In mesh mode, does a per-geometry triangle raycast. Calls `SelectCurrentNode()` with the hit node.

---

### ndMenuRenderPass

**Files:** `ndMenuRenderPass.h` / `ndMenuRenderPass.cpp`

Thin wrapper around `ndRenderPassGui`. Its `RenderScene()` calls `ndAssetEditor::RenderLayout()`, which drives all ImGui panels.

---

## Subsystems

### Scene Management

The editor maintains two parallel trees at all times:

- **Logical tree** (`ndMesh`) — contains physics data: rigid bodies, joints, collision shapes, loop constraints, colliding pairs, geometry.
- **Visual tree** (`ndRenderSceneNode`) — contains render primitives and transform matrices for OpenGL.

Both trees have the same node names, which is how the editor maps between them (e.g. `m_entity->FindByName(m_currentSelection->GetName())`).

Scene changes follow this sequence:

1. Load/create data → call `SetVisualScene(mesh, renderMesh)` → stores in `m_newMesh` / `m_newSceneMesh`.
2. Next frame in `Run()`: old entity is removed from renderer; new mesh and entity are committed; debug display is reset; camera is auto-fit if `m_initCamera` is true.

---

### Selection Model

`m_currentSelection` is a weak pointer to the primary selected `ndMesh` node.

`m_currentSubSelection` is used for operations that require picking a second node (adding a loop joint, adding a colliding pair, align-to-target, transform modifier target).

`m_subSelection` (an `ndSubSelectionMode` enum) governs how clicks are interpreted while a secondary-selection mode is active. It is reset to `m_none` when the operation completes or the user exits the mode.

`m_lockSelection` prevents accidental selection changes during precise editing.

---

### Properties Panel

**File:** `ndPropertiesPanel.cpp`

`ShowPropertiesPanel()` is the top-level dispatch. It checks the type of `m_currentSelection` and calls the appropriate sub-panel:

```
ShowPropertiesPanel()
  if GetAsMesh()
    ShowPropertiesMeshInfo()
    if GetRigidBody()
      if GetJoint() → ShowPropertiesJointInfo()
      ShowPropertiesCollisionInfo()
      ShowPropertiesRigidBodyInfo()
  else if GetAsCloseLoopConstraints()
    ShowPropertiesJointsLoopInfo()
  else if GetAsCollidingPairs()
    ShowPropertiesCollidingPairs()
```

---

### Outliner Panel

**File:** `ndOutlierPanel.cpp`

`ShowOutlierPanel()` renders the scene tree using recursive `ShowOutlierExplorer()`. Each node displays its name; clicking it calls `SelectCurrentNode()`.

Special handling:
- Nodes typed as `ndCloseLoopConstraints` expand via `ShowOutlierExplorerCloseLoop()`, listing loop joint names.
- Nodes typed as `ndCollidingPairs` expand via `ShowOutlierExplorerCollidindPairs()`, listing pair names.
- Regular mesh nodes with a rigid body show `joint`, `geometry`, and `rigidBody` sub-items as leaf bullets.

---

### Collision Editing

**File:** `ndPropertiesPanelCollision.cpp`

`ShowPropertiesCollisionInfo()` presents a combo box to switch the collision shape type and delegates to a shape-specific editor:

| Shape class | Editor function |
|-------------|----------------|
| `ndShapeNull` | (no parameters) |
| `ndShapeBox` | `EditCollisionBox()` — x, y, z half-extents |
| `ndShapeSphere` | `EditCollisionSphere()` — radius |
| `ndShapeCapsule` | `EditCollisionCapsule()` — radius0, radius1, height |
| `ndShapeCylinder` | `EditCollisionCylinder()` — radius0, radius1, height |
| `ndShapeChamferCylinder` | `EditCollisionChamferCylinder()` — radius, height |
| `ndShapeWheel` | `EditCollisionWheel()` — (no extra params currently) |
| `ndShapeConvexHull` | `EditCollisionConvexHull()` — tolerance, max points |
| `ndShapeCompound` | `EditCollisionCompound()` — sub-shape count (read-only display) |

`EditShapeTransform()` exposes the shape's local offset matrix (position, rotation, scale) in either absolute (`m_parentSpaceTransform = true`) or delta (`false`) mode.

`MakeVisualGeometry()` provides a "build visual mesh" button that generates render geometry from the current collision shape and replaces the node's visual primitive.

All changes call `GetDebugDisplay()->RebuildDebugCollision()` to refresh the debug overlay immediately.

---

### Joint Editing

**File:** `ndPropertiesPanelJoints.cpp`

`ShowPropertiesJointInfo()` presents a combo to switch the structural joint type and delegates to:

| Joint class | Editor function | Editable parameters |
|-------------|----------------|---------------------|
| `ndJointFix6dof` | `EditFix6dofJoint()` | softness, max force, max torque |
| `ndJointHinge` | `EditHingeJoint()` | spring, damper, regularizer, limits on/off, min/max limit |
| `ndJointSlider` | `EditSliderJoint()` | spring, damper, regularizer, limits on/off, min/max limit |
| `ndJointPlane` | `EditPlaneJoint()` | lock rotation on/off |
| `ndJointRoller` | `EditRollerJoint()` | linear + angular actuator params and limits |
| `ndJointCylinder` | `EditCylinderJoint()` | linear + angular actuator params and limits |
| `ndJointDoubleHinge` | `EditDoubleHingeJoint()` | two independent axis params and limits |
| `ndJointWheel` | `EditWheelJoint()` | spring, damper, regularizer, upper/lower stop, steering angle, brake/handbrake torque |
| `ndJointSpherical` | `EditSphericalJoint()` | twist spring/damper/regularizer, twist limits, cone limits, max cone angle |

`EditJointGlobalMatrix()` provides relative position and rotation inputs that adjust both `m_localFrame0` (child frame) and `m_localFrame1` (parent frame) simultaneously to keep the joint in global alignment.

---

### Loop Joints

**File:** `ndPropertiesPanelJointsLoopInfo.cpp`

Loop joints represent closed kinematic chains (e.g. gears, differentials, IK effectors). They are stored in a `ndCloseLoopConstraints` node that lives as a child of the scene root.

`ShowPropertiesJointsLoopInfo()` displays a list box of all loops, a joint-type combo, and delegates to per-type editors mirroring the structural joint editors. Additional loop-only types:

| Joint class | Editor function | Notes |
|-------------|----------------|-------|
| `ndJointGear` | `EditGearLoopJoint()` | gear ratio |
| `ndMultiBodyVehicleDifferentialAxle` | `EditDifferentialAxleLoopJoint()` | gear ratio |
| `ndIkSwivelPositionEffector` | `EditSwivelPositionEffectorLoopJoint()` | linear/angular spring-damper, work space radii, swivel mode, rotation order |

Loop joints use either `EditLoopJointLocalMatrix()` (each frame edited independently in its body's local space) or `EditLoopJointGlobalMatrix()` (frames kept in sync via global matrix math) depending on joint type.

`AddLoopJoint()` creates a new `ndMeshLoopJoint` with a default spherical joint between `m_currentSelection` and `m_currentSubSelection` and pushes undo snapshots.

---

### Colliding Pairs

**File:** `ndPropertiesPanelCollidingPairsInfo.cpp`

Colliding pairs are stored in a `ndCollidingPairs` node as a list of `ndMeshCollidingPair` structs (parent node + child node).

`ShowPropertiesCollidingPairs()` shows a list box of all pairs and a "remove colliding pair" button.

`SetCollidingSubSelection()` validates that the proposed second node forms a new, non-duplicate pair before accepting it as `m_currentSubSelection`.

`AddCollidingPair()` appends a new pair and pushes undo snapshots.

---

### Rigid Body Editing

**File:** `ndPropertiesPanelRigidBody.cpp`

`ShowPropertiesRigidBodyInfo()` exposes:

- **mass** — recalculates principal inertia from shape geometry when changed.
- **mass volume weight** — per-body density multiplier for `ndNomalizeMassDistribution`.
- **linear step / angle step** — CCD integration limits.
- **linear damp / angular damp** — intrinsic damping.
- **centre of mass** — local offset vector.
- **principal inertia** — direct edit of Ixx, Iyy, Izz.
- **inertia axis** — Euler angles of the principal-axis frame.
- Buttons to enter/exit **loop joint** and **colliding pair** sub-selection modes.

`AddRigidBody()` creates a `ndMeshBodyDynamic` with a null shape and, if a parent rigid body exists, a default `ndJointFix6dof`.

---

### Mesh Node Editing

**File:** `ndPropertiesPanelMeshNode.cpp`

`ShowPropertiesMeshInfo()` exposes:

- **Name** — rename with automatic uniqueness suffix.
- **parent space transform** / **transform pivot only** checkboxes.
- **Add node** / **Clone node** / **Delete node** (delete is a stub).
- **Add body** / **Delete body**.
- **Node transform** — position and rotation in either absolute or delta mode.
- **Geometry transform** — local offset of the visual mesh relative to the node pivot.
- **Modifier type** — dropdown to attach `ndMeshTransformModifierLookAt`, `ndMeshTransformModifierTwoLinksIK`, or `ndMeshTransformModifierUserDefined`.
- **Align to target** — two-step workflow: enter mode, pick target node, apply.

`ApplyNodeTransform()` is the central transform mutation function. It updates the node matrix, the visual scene node, and (when `m_transformPivotOnly` is true) compensates child transforms and the collision shape local matrix so the geometry stays in place while only the pivot moves.

---

## Tools

Tools are modal operations activated from the **Tools** menu. While active they render their own floating ImGui window and call `Execute()` each frame. Closing the window sets `m_toolActive = false`, which causes `RenderLayout()` to destroy the tool.

All tools wrap their mutations in `ndUndoRedoMeshNode` snapshot pairs.

### ndResizeMesh

**Files:** `ndResizeMesh.h` / `ndResizeMesh.cpp`

Applies a uniform scale to the entire scene. Shows the current AABB dimensions for reference.

`ApplyScale()`:
1. Builds a diagonal scale matrix.
2. Calls `ndMesh::ApplyTransform()` on the geometry.
3. Iterates all nodes, scaling rigid-body centre of mass, inertia tensor, and collision shape.
4. Rebuilds the visual scene from scratch via `SetVisualScene()`.

### ndRotateMesh / ndRotateBones / ndRotatePivots

**Files:** `ndRotateMesh.h` / `ndRotateMesh.cpp`

Three related tools sharing the same Euler-angle input UI.

| Tool | Scope | Physics update |
|------|-------|----------------|
| `ndRotateMesh` | All geometry + coordinate system | Currently commented out |
| `ndRotateBones` | Bone nodes only | Updates CoM, inertia tensor, shape local matrix, joint frames |
| `ndRotatePivots` | Node pivot matrices | No physics update |

### ndNomalizeMassDistribution

**Files:** `ndNomalizeMassDistribution.h` / `ndNomalizeMassDistribution.cpp`

Redistributes mass across all rigid bodies proportionally to their collision volume multiplied by their individual `m_massVolumeWeigh` factor, such that the total mass equals a user-specified value. Also enforces a minimum principal-inertia ratio to prevent degenerate thin-slab inertia tensors.

---

## File I/O

### URDF Import

**Files:** `ndUrdfFile.h` / `ndUrdfFile.cpp`

`ndUrdfMeshLoader` (subclass of `ndRenderMeshLoader`) parses ROS URDF XML files.

`Import()` pipeline:

1. Parse all `<link>` nodes into a `ndTree<Hierarchy>` map.
2. Parse `<joint>` nodes to build parent–child relationships.
3. Find the root link (the one with no parent joint).
4. Parse global `<material>` definitions.
5. Walk the tree breadth-first: create `ndBodyDynamic` per link via `ImportLink()`, connect with `ndJointBilateralConstraint` via `ImportJoint()`.
6. Construct a default `ndModelArticulation`, then call `CreateDefaultMesh()`.
7. Post-process: strip a static root body if it has exactly one dynamic child with a fixed joint.
8. Replace default visuals by calling `ImportVisual()` per node.

`ImportJoint()` maps URDF joint types to Newton joints:

| URDF type | Newton joint |
|-----------|-------------|
| `fixed` | `ndJointFix6dof` |
| `continuous` | `ndJointHinge` (unlimited) |
| `revolute` | `ndJointHinge` (with limits) |
| `prismatic` | `ndJointSlider` |
| `floating` | `ndIkJointSpherical` (via `newton` extension) |

`ImportVisual()` builds `ndMeshEffect` geometry from URDF `<visual>` elements. Supports sphere, cylinder, capsule, box, and mesh (`.obj` and `.stl`) shapes.

`ImportCollision()` builds `ndShapeInstance` from `<collision>` elements. Multiple collision elements per link produce a `ndShapeCompound`.

### File Browser Utilities

**Files:** `ndFileBrowser.h` / `ndFileBrowser.cpp`

Platform-native open/save dialogs (Win32 `GetOpenFileName` / `GetSaveFileName`). On non-Windows platforms these functions return `false` immediately (not yet implemented).

| Function | Purpose |
|----------|---------|
| `dGetLoadNdFileName()` | Open `.nd` file |
| `dGetSaveNdFileName()` | Save `.nd` file |
| `dGetImportFbxFileName()` | Open `.fbx` file |
| `dGetImportUrdfFileName()` | Open `.urdf` file |
| `dGetWorkingFileName()` | Recursive directory search for a file by name within a base path |

---

## Render Pipeline

Render passes execute in this order every frame:

| Pass | Class | Purpose |
|------|-------|---------|
| 1 | `ndRenderPassShadows` | Shadow map generation |
| 2 | `ndRenderPassColor` | Lit colour pass (active only in `m_shaded` mode) |
| 3 | `ndRenderPassEnvironment` | Skybox / cube-map environment |
| 4 | `ndDebugDisplayRenderPass` | Editor debug overlays |
| 5 | `ndMenuRenderPass` | Dear ImGui UI |

The `ndMenuRenderPass` is last so the UI renders on top of everything.

`ndRenderPassColor` is toggled active/inactive depending on render mode — wireframe and hidden-surface modes skip the full lit pass.

---

## Data Flow Diagrams

### Undo/Redo Push

```
User edits a value
  │
  ├── Push(snapshot_before)   ← captures current state
  │
  ├── mutate data
  │
  └── Push(snapshot_after)    ← captures new state
        │
        └── ndUndoRedo::Push()
              if snapshot_after != snapshot_before
                → append to list, advance cursor
              else
                → discard (no-op)
```

### Node Selection via Viewport Click

```
Mouse left-button down (not captured by ImGui)
  │
  └── ndEditorCameraFlyby::MouseSelection()
        │
        ├── [bone mode] ray-to-segment closest point test
        └── [mesh mode] per-geometry triangle raycast
              │
              └── ndAssetEditor::SelectCurrentNode(hitNode)
                    │
                    ├── m_none         → m_currentSelection = hitNode
                    ├── m_loopJoint    → SetLoopJointSelection()
                    ├── m_collidingPair→ SetCollidingSubSelection()
                    ├── m_alignToTarget→ m_currentSubSelection = hitNode
                    └── m_transformModifier → SetModifierSubSelection()
```

### Scene Load

```
File → ndRenderMeshLoader::LoadMesh() / ImportFbx() / ndUrdfMeshLoader::Import()
  │
  └── SetVisualScene(mesh, renderMesh)
        │   stores in m_newMesh / m_newSceneMesh
        │
        └── [next frame] ndAssetEditor::Run()
              removes old entity from renderer
              m_mesh = m_newMesh
              m_entity = m_newSceneMesh
              AddSceneNode(m_entity)
              ndDebugDisplayRenderPass::ResetScene()
              auto-fit camera (if m_initCamera)
```
