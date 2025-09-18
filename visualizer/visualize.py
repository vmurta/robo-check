import trimesh
import numpy as np
from pythreejs import *
from ipywidgets import VBox, HTML
from IPython.display import display
from pythreejs import EdgesGeometry, LineSegments, LineBasicMaterial

# Load the .obj file (replace with your filename)
mesh = trimesh.load_mesh('../robot_OBB_BVH.obj', process=False, maintain_order=True, group_material=True)
# print(mesh.body_count)
# If the mesh is a Scene (multiple parts), get the geometries
if isinstance(mesh, trimesh.Scene):
    print("Scene with multiple geometries")
    geometries = [g for g in mesh.geometry.values()]
else:
    geometries = mesh.split()

# Create Mesh objects for pythreejs
three_meshes = []
for i, g in enumerate(geometries):
    geometry = BufferGeometry(
        attributes={
            'position': BufferAttribute(np.array(g.vertices, dtype=np.float32)),
            'index': BufferAttribute(np.array(g.faces.flatten(), dtype=np.uint32)),
        }
    )
    # print(i * 60 % 360)
    material = MeshLambertMaterial(color=f"hsl({(i*60)%360}, 80%, 60%)", side='DoubleSide')
    # material = MeshLambertMaterial(color="hsl(180, 80%, 60%)", side='DoubleSide')
    three_mesh = Mesh(geometry=geometry, material=material, name=f"part_{i}")
    # Add black edge outlines
    # edge_geom = EdgesGeometry(geometry=geometry)
    # edge_mat = LineBasicMaterial(color='black', linewidth=2)
    # edge_lines = LineSegments(edge_geom, edge_mat)
    # group = Group(children=[three_mesh, edge_lines], name=f"part_{i}_group")
    three_meshes.append(three_mesh)

print(f"Loaded {len(three_meshes)} parts from the model.")
# print(three_meshes)
scene = Scene(children=three_meshes + [AmbientLight(intensity=0.8)])
camera = PerspectiveCamera(position=[0, 0, 5], up=[0, 0, 1], aspect=1)
controller = OrbitControls(controlling=camera)
renderer = Renderer(camera=camera, scene=scene, controls=[controller], width=600, height=400)

print("Renderer set up.")
# zoom to fit 
def zoom_to_fit():
    all_vertices = np.vstack([g.vertices for g in geometries])
    min_bounds = all_vertices.min(axis=0)
    max_bounds = all_vertices.max(axis=0)
    center = (min_bounds + max_bounds) / 2
    size = np.linalg.norm(max_bounds - min_bounds)
    
    camera.position = [center[0], center[1], center[2] + size * 1.5]
    camera.lookAt(center.tolist())
    camera.near = size * 0.1
    camera.far = size * 10
    # camera.updateProjectionMatrix()

zoom_to_fit()
# Info display
info = HTML("Click on a part to toggle its visibility.")

# Picking logic
def on_pick(change):
    picked = change['new']
    if picked is not None and hasattr(picked, 'object'):
        mesh = picked.object
        mesh.visible = not mesh.visible
        info.value = f"Toggled visibility of {mesh.name}"

renderer.observe(on_pick, names=['picked'])

print("Ready for interaction.")
display(VBox([renderer, info]))