# libmv blender camera track export; do not edit
import Blender
from Blender import Camera, Object, Scene, NMesh
from Blender import Mathutils
from Blender.Mathutils import *
cur = Scene.GetCurrent()
# Cameras
c0000 = Camera.New('persp')
c0000.lens = 32
c0000.setDrawSize(0.05)
o0000 = Object.New('Camera')
o0000.name = 'libmv_cam0000'
o0000.setMatrix(Mathutils.Matrix([0.815973,-0.574267,0.06637,0.0],[-0.578079,-0.811264,0.0876121,0.0],[0.00353086,-0.109856,-0.993941,0.0],[-0.00592824,-0.0366872,0.0291193,1.0]))
o0000.link(c0000)

cur.link(o0000)

c0001 = Camera.New('persp')
c0001.lens = 32
c0001.setDrawSize(0.05)
o0001 = Object.New('Camera')
o0001.name = 'libmv_cam0001'
o0001.setMatrix(Mathutils.Matrix([-0.984259,0.170147,0.0477925,0.0],[-0.17112,-0.849889,-0.498404,0.0],[-0.0441837,-0.498737,0.865627,0.0],[-0.061511,-0.217209,0.863137,1.0]))
o0001.link(c0001)

cur.link(o0001)

# Point cloud
ob=Object.New('Mesh','libmv_point_cloud')
ob.setLocation(0.0,0.0,0.0)
mesh=ob.getData()
cur.link(ob)
v = NMesh.Vert(-0.0615092,-0.217207,0.863136)
mesh.verts.append(v)
v = NMesh.Vert(-0.00592745,-0.0366893,0.0291257)
mesh.verts.append(v)
v = NMesh.Vert(0.13474,-0.0523115,0.437725)
mesh.verts.append(v)
v = NMesh.Vert(0.083099,0.10457,0.481497)
mesh.verts.append(v)
v = NMesh.Vert(-0.0335118,0.0323226,0.285559)
mesh.verts.append(v)
v = NMesh.Vert(-0.145027,0.149563,0.435428)
mesh.verts.append(v)
mesh.update()
cur.update()

# Add a helper object to help manipulating joined camera andpoints
scene_dummy = Object.New('Empty','libmv_scene')
scene_dummy.setLocation(0.0,0.0,0.0)
cur.link(scene_dummy)
scene_dummy.makeParent([ob])
scene_dummy.makeParent([o0000])
scene_dummy.makeParent([o0001])

scene_dummy.SizeX=1.0
scene_dummy.SizeY=1.0
scene_dummy.SizeZ=1.0
