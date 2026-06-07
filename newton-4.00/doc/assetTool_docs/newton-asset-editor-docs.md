# Newton Asset Editor — User Documentation

## Overview

The Newton Asset Editor is a desktop tool for creating and editing physics-ready 3D assets for the Newton Game Dynamics engine. It allows you to import meshes, configure rigid bodies, define collision shapes, set up joints, manage loop constraints, and export assets in Newton's native format (`.nd`).

The editor uses an ImGui-based interface with dockable panels. The main window is divided into:

- **Main Menu Bar** — file operations, tools, and display options
- **Main Toolbar** — undo/redo, selection lock, camera views, and projection mode
- **Outliner Panel** — scene hierarchy tree
- **Properties Panel** — context-sensitive properties for the selected node

---

## File Operations

### New
**File → New** creates a blank scene with a single root mesh node named `root`. Any unsaved changes to the current scene are discarded.

### Load
**File → Load** opens a native file dialog to load a `.nd` file. The scene replaces the current one and the undo/redo history is cleared.

### Save / Save As
**File → Save** writes the current scene back to the path it was loaded from (or last saved to).

**File → Save As** lets you choose a new path. The scene's current path is updated accordingly.

### Import FBX
**File → Import fbx** imports an FBX file. Node name suffixes control automatic physics setup:

| Suffix | Effect |
|---|---|
| `-rb` | Creates a dynamic rigid body with default mass 1.0 |
| `-box`, `-sphere`, `-capsule`, `-cylinder`, `-chamferedCylinder`, `-tire`, `-convexHull`, `-mesh`, `-vhacd` | Assigns the named collision shape |
| `-hinge`, `-slider`, `-roller`, `-plane`, `-wheel`, `-spherical`, `-cylindrical` | Creates the named joint with default parameters |
| `-hidden` | Marks the node as invisible at runtime |

### Import URDF
**File → Import urdf** imports a URDF robot description. Visual geometry is generated from `.obj` or `.stl` mesh files found on the search path. Collision shapes and joint types are read directly from the URDF XML.

---

## Toolbar

| Control | Function |
|---|---|
| **Undo / Redo** | Step through the edit history |
| **Lock / Unlock Selection** | Prevents mouse clicks in the viewport from changing the selected node |
| **Camera combo box** | Switch between Free, Back, Front, Left Side, and Right Side views |
| **Perspective / Orthographic** | Toggle the camera projection mode |

---

## Viewport Navigation

Mouse controls when the cursor is not over an ImGui panel:

| Action | Effect |
|---|---|
| Left-drag | Orbit camera (yaw and pitch) |
| Right-drag | Pan camera (up/down, left/right) |
| Middle-drag | Zoom in/out |
| Left-click | Select a mesh node or bone via ray cast |

The camera pitch is clamped to ±80 degrees. Left-click selection is disabled while **Lock Selection** is active.

---

## Outliner Panel

Displays the scene as a collapsible tree. Clicking a node selects it and populates the Properties Panel.

Special tree entries:

- **Loop Joints node** — shows all close-loop constraints defined for the scene. Clicking an entry sets the active loop joint index used by the Properties Panel.
- **Colliding Pairs node** — shows all pairs of bodies allowed to collide. Clicking an entry sets the active pair index.
- Nodes with a rigid body show **joint**, **geometry**, and **rigidBody** sub-items as visual indicators.

---

## Properties Panel

The Properties Panel is context-sensitive. Its contents depend on what is selected in the Outliner.

### Mesh Node

Shown when a regular mesh node is selected. Sections include:

**Identity**
- Rename the node. If the new name already exists in the scene, a suffix `_1` is appended automatically.
- Toggle **parent space transform** — when on, position and rotation fields show and accept absolute local values; when off, they accept incremental deltas.
- Toggle **transform pivot only** — moves only the node's pivot (and adjusts child positions, geometry offset, and collision offset to compensate, keeping visual geometry in place).
- Toggle **visible** — controls runtime and editor visibility.

**Node Management**
- **Add node** — creates a new unnamed child node.
- **Delete node** — (placeholder, not yet implemented).
- **Clone node** — duplicates the node and its full subtree with unique names.
- **Add body / Delete body** — attaches or removes a dynamic rigid body. Adding a body also adds a Fix6dof joint if a parent body exists.

**Node Transform** — position and rotation (Euler angles in degrees) relative to parent.

**Geometry Transform** — offset of the visual mesh relative to the node pivot.

**Modifier** — assigns a transform modifier to the node (only available on nodes without a rigid body):
- **none** — no modifier.
- **LookAt** — node continuously orients toward a target node.
- **TwoLinksIK** — two-link inverse kinematics solver. Requires selecting a target and a child link, plus a solution sign (+1 or -1).
- **UserDefined** — stores a constructor string that runtime code can use to instantiate a custom modifier.

**Align to Target** — click **align to target**, then click another node in the viewport; clicking **pick to target** rotates the pivot so its X axis points toward the target.

---

### Rigid Body

Shown below the Mesh Node section when a body exists.

| Field | Description |
|---|---|
| **mass** | Total mass in kg. Automatically recomputes principal inertia from the collision shape. |
| **mass weigh** | Per-body weight factor used by the Normalize Mass Distribution tool. |
| **linear step** | Maximum linear integration sub-step size (0.1 – 30). |
| **angle step** | Maximum angular integration sub-step size in degrees (10 – 180). |
| **linear damp** | Intrinsic linear damping coefficient (0 – 1). |
| **angular damp** | Per-axis intrinsic angular damping (xyz vector). |
| **com** | Local centre-of-mass offset. |
| **principal inertia** | Diagonal elements of the inertia tensor in the principal frame. |
| **inertia axis** | Euler angles (degrees) of the principal inertia frame relative to the node. |

**Loop Joint editing** — click **enter loop joint** to enter selection mode; left-click a second body in the viewport; click **add loop joint** to create a spherical loop joint between them. Click **exit loop joints** to leave this mode.

**Colliding Pairs editing** — click **enter colliding pairs** to enter selection mode; left-click another body; click **add pair** to register a permitted collision. Click **exit colliding pairs** to leave this mode.

---

### Collision Shape

Shown when a rigid body exists. A dropdown lets you change the shape type. Available shapes:

| Shape | Editable Parameters |
|---|---|
| **Null** | None — body has no collision |
| **Box** | x, y, z extents |
| **Sphere** | radius |
| **Capsule** | radius0, radius1, height |
| **Cylinder** | radius0, radius1, height |
| **ChamferCylinder** | radius, height |
| **Wheel** | (no editable parameters) |
| **ConvexHull** | tolerance, max point count |
| **Compound** | number of sub-shapes (read-only display) |

**Shape Transform** — position, rotation, and scale of the collision shape relative to the node pivot. The **parent space transform** toggle controls whether inputs are absolute or incremental.

**Build visual mesh** — generates a new visual mesh from the current collision shape geometry, replacing the existing one.

---

### Constraint Joint

Shown when the selected node has both a rigid body and a joint. A dropdown selects the joint type. All joint types share:

- **rel position / rel rotation** — adjust the joint frame incrementally in world space.

Per-type parameters (for types with linear or angular axes):

| Parameter | Description |
|---|---|
| **spring const** | Spring stiffness of the actuator |
| **damper const** | Damping coefficient of the actuator |
| **regularizer** | Numerical regularization factor |
| **limits on** | Enable/disable travel limits |
| **min limit / max limit** | Travel limits (linear in meters, angular in radians) |

Additional parameters for specific types:

- **Fix6dof** — softness, max force, max torque
- **Wheel** — spring/damper, upper/lower stop, steering angle, brake torque, hand-brake torque
- **Spherical** — twist limits (min/max in degrees), cone angle limit and enable toggle
- **DoubleHinge / Roller / Cylinder** — separate parameter blocks for each axis

---

### Loop Joint

Shown when the **Loop Joints** node is selected. A list box shows all defined loop joints. For the selected joint:

- **Remove selected** — deletes the joint.
- **Joint type** dropdown — change the joint type. Available types include Fix6dof, Hinge, Slider, Plane, Roller, Cylinder, DoubleHinge, Wheel, Spherical, Gear, DifferentialAxle, and IkSwivelPositionEffector.

Editing the joint frame:
- **Global frame** mode — position and rotation inputs move both child and parent frames together, keeping their relative alignment in world space.
- **Local frame** mode — edit child (`localFrame0`) and parent (`localFrame1`) frames independently.

Additional fields per type follow the same conventions as structural joints. The **Gear** and **DifferentialAxle** types expose a single **gear ratio** field. The **IkSwivelPositionEffector** type exposes linear and angular actuator parameters, workspace radius limits, rotation order, and swivel control enable/disable.

---

### Colliding Pairs

Shown when the **Colliding Pairs** node is selected. A list box shows all registered pairs. The **remove colliding pair** button deletes the currently selected pair.

---

### Materials

Shown when the selected node has visual geometry. A list box selects the active material. Editable fields:

- material name
- ambient, diffuse, specular, reflection (RGB vectors)
- opacity, shiness (floats)
- texture (filename string)

Changing any value immediately rebuilds the visual render scene.

---

### Custom Properties

Each node can carry a list of typed custom properties (float, string or refrence to othe node). 
Use **new property** to add one and **delete selected** to remove it. The property name and value are editable inline.

---

## Tools Menu

Tools are modal windows that stay open until you close them (click the X or set the checkbox to false).

### Resize Mesh
Uniformly scales all geometry, collision shapes, joint frames, and centre-of-mass values by a single scalar. The current bounding box size is shown for reference. Enter a scale value and click **execute**.

### Rotate Mesh
Rotates all geometry around the world origin by Euler angles (degrees). Enter pitch, yaw, roll, and click **execute**.

### Rotate Pivots
Rotates only node pivots and joint frames, leaving geometry in place.

### Rotate Bones
Rotates bones and their associated physics data (inertia, collision shape local matrix, joint frames).

### Normalize Mass Distribution
Redistributes mass across all dynamic bodies so that the total equals **total mass** (kg). The **principal inertia ratio** clamps the minimum principal inertia to `ratio × max_inertia`, preventing numerically degenerate bodies. Click **execute** to apply.

---

## Options Menu

| Option | Description |
|---|---|
| **Render mode** | Shaded, Wireframe, or Hidden Surface |
| **Gizmo scale** | Size of the axis frame gizmos drawn in the viewport |
| **Show node** | Draw the selection highlight |
| **Show node pivot** | Draw the node's local coordinate frame |
| **Show geometry pivot** | Draw the visual mesh offset frame |
| **Show collision pivot** | Draw the collision shape local frame |
| **Show center of mass** | Draw the centre-of-mass frame |
| **Show Joints** | Draw the joint constraint frames |
| **Show collision** | Draw collision shape outlines |
| **Show mesh skeleton** | Switch between bone selection mode and mesh selection mode |

---

## Undo / Redo

All editing operations push snapshot commands onto an undo stack. The toolbar **undo** and **redo** buttons step through the history. The history is cleared when a new file is loaded or created.

---

## Keyboard & Mouse Summary

| Input | Context | Action |
|---|---|---|
| Left-click (viewport) | Mesh mode | Select mesh node |
| Left-click (viewport) | Bone mode | Select nearest bone |
| Left-drag | Viewport | Orbit camera |
| Right-drag | Viewport | Pan camera |
| Middle-drag | Viewport | Zoom |

---

## Supported File Formats

| Format | Read | Write |
|---|---|---|
| `.nd` (Newton native) | ✓ | ✓ |
| `.fbx` | ✓ | — |
| `.urdf` | ✓ | — |
| `.stl` (referenced by URDF) | ✓ | — |
| `.obj` (referenced by URDF) | ✓ | — |
