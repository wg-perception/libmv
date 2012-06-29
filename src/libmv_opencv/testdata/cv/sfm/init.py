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
o0000.setMatrix(Mathutils.Matrix([1,0,0,0.0],[-0,-1,-0,0.0],[-0,-0,-1,0.0],[-0,-0,-0,1.0]))
o0000.link(c0000)

cur.link(o0000)

c0001 = Camera.New('persp')
c0001.lens = 32
c0001.setDrawSize(0.05)
o0001 = Object.New('Camera')
o0001.name = 'libmv_cam0001'
o0001.setMatrix(Mathutils.Matrix([-0.873598,0.248169,-0.418616,0.0],[-0.155169,-0.957349,-0.24373,0.0],[-0.461248,-0.147966,0.874846,0.0],[-0.406157,-0.0316833,0.913254,1.0]))
o0001.link(c0001)

cur.link(o0001)

# Point cloud
ob=Object.New('Mesh','libmv_point_cloud')
ob.setLocation(0.0,0.0,0.0)
mesh=ob.getData()
cur.link(ob)
v = NMesh.Vert(0.0454348,-0.061863,0.688192)
mesh.verts.append(v)
v = NMesh.Vert(0.0272438,-0.0581996,0.104082)
mesh.verts.append(v)
v = NMesh.Vert(0.120066,-0.0332311,0.482559)
mesh.verts.append(v)
v = NMesh.Vert(0.0239618,0.163182,0.409482)
mesh.verts.append(v)
v = NMesh.Vert(-0.0457249,0.00479279,0.264034)
mesh.verts.append(v)
v = NMesh.Vert(-0.327636,0.055537,0.696841)
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
