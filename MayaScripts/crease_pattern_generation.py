from maya import cmds
import maya.api.OpenMaya as om
from maya.api.OpenMaya import MVector
from maya.api.OpenMaya import MFnMesh, MDagPath
import pymel.core.nodetypes as nodetypes
from maya.api.OpenMaya import MGlobal as omg

# create a poly plane
# valley and mountain define
# make multiple cuts on the plane 

def polySplitExample():
	# create a new face
	newFace = cmds.polyCreateFacet( p=[(0, 2, 0), (0, -2, 0), (4, -2, 0), (4, 2, 0)])
	# create 
	cmds.polySplit( ip=[(2, 0.1), (3, 0.5), (0, 2, -1, 0.0), (0, 0.9)] )

def testLine():
	pts = []
	pt1 = (0,0.5,0.0,0.0)
	pt2 = (0,0.5,0.0,0.5)
	pts.append(pt1)
	pts.append(pt2)
	return pts

def cubicBezierCurve(p0,p1,p2,p3,segments =4, include_p0 = False, include_p3 = False):
	'''
	Return a list of points as the beizer curve 
	'''
	pts = []
	if(include_p0):
		pts.append(p0)

	# polulate spline curves
	for i in range(segments):
		t = (i+ 0.5) / segments
		pt = (1-t)*(1-t)*(1-t)*p0 + 3*(1-t)*(1-t)*t*p1 + 3*(1-t)*t*t*p2 + t*t*t*p3
		pts.append(pt)

	if(include_p3):
		pts.append(p3)

	return pts

def createBeizerCrease(mesh,start, end, segments):
	ip = []
	ip.append(epStart)

# some short hand wrappers to make the code more readable
def getVertName(n, i):
	return n + '.vtx[' + str(i) + ']'

def getMeshName(v):
	return v.split('.')[0]

def getVertextFaceName(n, i,j):
	return n + '.vtxFace[' + str(i) + '][' + str(j) + ']'

def getEdgeName(n, i):
	return n + '.e[' + str(i) + ']'

def getFaceName(n, i):
	return n + '.f[' + str(i) + ']'

def getDag(name):
	sel_list = omg.getSelectionListByName(name)
	print(sel_list)
	#
	'''
	iterator = om.MItSelectionList(sel_list, om.Mfn.kDagNode)
	dagPath = om.MDagPath()
	result = []
	if not iterator.isDone():
		iterator.getDagPath(dagPath)
		result = [dagPath]
'''
	return result

def dotProduct(v0,v1):
	return v0[0] * v1[0] + v0[1] * v1[1]+v0[2] * v1[2]

def getNormalized(v):
	'''
	get a normalized vector from this vector
	'''
	s = 0
	#print('a')
	# get all vert
	for i in v:
		s += i*i

	#print('b')
	s = s**0.5
	res = []
	for i in range(len(v)):
		res.append(v[i] / s)
	#print('c')
	return res

def findClosesVertexOnMesh(mesh, point):
	'''
	Try to find the closest vertex on the mesh
	'''
	# the core here here is to use http://download.autodesk.com/us/maya/2011help/CommandsPython/polyInfo.html
	# the polyInfo command is like the iterator in openmesh
	# or a guide here http://polycount.com/discussion/77642/maya-select-all-by-componets-verts-edges-faces
	print(__name__ + ": mesh is " + str(mesh))
	# how do I give them Mfn
	'''
	d = getDag(mesh)
	# cmds.ls(mesh).getDagPath(0,d)
	print(d)
	if len(d) > 0:
		dagPath = d[0]
'''
	# print("open maya = " + str(mfnmesh))
	vtxCount = cmds.polyEvaluate(mesh, v = True)
	distance = 9999.0
	closest_idx = -1
	for i in range(vtxCount):
		vert = getVertName(mesh,i)
		vert_p = cmds.pointPosition(vert, w = True)
		dist_sqrt = (vert_p[0] - point[0])**2 + (vert_p[1] - point[1])**2 + (vert_p[2] - point[2])**2
		if dist_sqrt < distance:
			distance = dist_sqrt
			closest_idx = i

	if closest_idx != -1:
		return getVertName(mesh, closest_idx)
	else:
		return ""

def findBestEdgePointToCrease(vertex, crease_direction):
	'''
	Try to find the best edge to start the crease based on the crease direction
	return the edge index and edge time in (int, float) form
	'''
	print("crease dir" + str(crease_direction))
	edges = cmds.polyInfo(vertex, ve = True)
	faces = cmds.polyInfo(vertex, vf = True)
	# we skip VERTEX and 0:
	skip = 2
	e_is = edges[0].split()
	# print("e_is len is " + str(len(e_is)))
	best_idx_min = -1
	closeset_dot_min = 0
	best_idx_max = -1
	closeset_dot_max = 0
	for i in range(skip,len(e_is)):
		e_n = getEdgeName(getMeshName(vertex),e_is[i]);
		# print("e_n is " + str(e_n))
		ev_s = cmds.polyInfo(e_n, ev = True)
		from_idx = ev_s[0].split()[2]
		to_idx = ev_s[0].split()[3]
		# what is the Hard edge and soft edge?
		# print(ev_s)
		from_v_name = getVertName(getMeshName(vertex),from_idx)
		to_v_name = getVertName(getMeshName(vertex),to_idx)

		from_p = cmds.pointPosition(from_v_name, w = True)
		to_p = cmds.pointPosition(to_v_name, w =True)
		
		cur_dir = (to_p[0] - from_p[0],to_p[1] - from_p[1],to_p[2] - from_p[2]) 
		# print(cur_dir)

		normalized_dir = getNormalized(cur_dir)
		# print(normalized_dir)
		
		dot = dotProduct(normalized_dir,crease_direction)
		# print(dot)

		if dot > closeset_dot_max:
			closeset_dot_max = dot
			best_idx_max = int(e_is[i])

		if dot < closeset_dot_min:
			closeset_dot_min = dot
			best_idx_min = int(e_is[i])

	# check if min or max is our best choice
	# print("e_is = " + str(e_is))
	print(closeset_dot_max)
	print(best_idx_max)
	print(closeset_dot_min)
	print(best_idx_min)
	if abs(closeset_dot_max) > abs(closeset_dot_min):
		return (best_idx_max, 0.0)
	else:
		return (best_idx_min, 1.0)


def createSingleCrease(mesh,p):
	# create a single crease out of 4 points p
	print(mesh)
	print("crease b points " + str(p))
	closest_vert_0 = findClosesVertexOnMesh(mesh,p[0])
	print(closest_vert_0)
	dir_01 = p[1] - p[0]
	best_edge_0 = findBestEdgePointToCrease(closest_vert_0, dir_01)
	print('best_edge 0: ' + str(best_edge_0))
	# p3
	closest_vert_1 = findClosesVertexOnMesh(mesh,p[3])
	print(closest_vert_1)
	dir_32 = p[2] - p[3]
	best_edge_1 = findBestEdgePointToCrease(closest_vert_1, dir_32)
	print('best_edge 1: ' + str(best_edge_1))


	# hey, now we have the best_edge_0, and _1, let's do the creassing
	epStart = best_edge_0
	epEnd = best_edge_1
	ip = []
	# start edge point
	ip.append(epStart)
	# intermediate points
	
	pts = cubicBezierCurve(p[0],p[1], p[2],p[3], segments = 10)
	for i in range(len(pts)):
		face_idx = 0
		if(i%10 == 0):
			print(pts[i])
		pt = pts[i]
		ip.append((face_idx,pt[0],pt[1],pt[2]))
		
	# end edge point
	ip.append(epEnd)
	cmds.ls(mesh)
	print(ip)

	# split the polygons
	cmds.polySplit(mesh,ip = ip)
	
	# poly subdivide?


def polySplitCrease():
	# create a uniform polygon first
	newFace = cmds.polyCreateFacet(n='myPaperFace', p = [(-1, 0 , -1), (-1, 0, 1), (1, 0, 1), (1, 0, -1)], s = 4)
    newFace = cmds.polyPlane(n='myPaperPlane', sx=10, sy=15, w=2, h=2)

	sel_list = om.MSelectionList()
	verticePos = [(-1, 0 , -1,1), (-1, 0, 1,1), (1, 0, 1,1), (1, 0, -1,1)]
	polygonCounts = [4,4,4,4]
	vertices = om.MPointArray()
	for p in verticePos:
		mPoint = om.MPoint()
		mPoint.x = p[0]
		mPoint.y = p[1]
		mPoint.z = p[2]
		vertices.append(mPoint)
	'''
	mfnMesh = om.MFnMesh()
	mfnMesh.create(vertices, polygonCounts)
	mfnMesh.updateSurface()
	cmds.sets(mfnMesh.fullPathName(), e = 1, fe = 'initialShadingGroup')
	mfnMesh.updateSurface()
	'''
	# mfnMesh = om.mfnMesh()
	# mfnmesh.create()
	# now need to find edge index from point
	# A example test beizer curve, later convert this to our curve
	
	'''
	-1 1   						1 1
			-0.5,0.5	0.5,0.5
			-0.5,-0.5	0.5,-0.5
	-1 -1  						1 -1 

	'''

	# this should be replaced by .cp file modifications
	p =  [MVector(-1.0,0.0,-1.0),MVector(-0.5,0.0,-0.5),MVector(-0.5,0.0,0.5), MVector(-1.0,0.0,1.0)]
	c2 = [MVector(-1.0,0.0,-1.0),MVector(-0.5,0.0,-0.5),MVector(0.5,0.0,-0.5), MVector(1.0,0.0,-1.0)]
	c3 = [MVector(-1.0,0.0,1.0),MVector(-0.5,0.0,0.5),MVector(0.5,0.0,0.5), MVector(1.0,0.0,1.0)]
	c4 = [MVector(1.0,0.0,-1.0),MVector(0.5,0.0,-0.5),MVector(0.5,0.0,0.5), MVector(1.0,0.0,1.0)]
	createSingleCrease(newFace[0], p)
	createSingleCrease(newFace[0], c2)
	createSingleCrease(newFace[0], c3)
	createSingleCrease(newFace[0], c4)

	# subdivide the faces with polySubdivideFacet


polySplitCrease()
