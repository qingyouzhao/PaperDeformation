from maya import cmds
import maya.api.OpenMaya as om
from maya.api.OpenMaya import MVector
import pymel.core.nodetypes as nodetypes

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

def getVertextFaceName(n, i,j):
	return n + '.vtxFace[' + str(i) + '][' + str(j) + ']'

def getEdgeName(n, i):
	return n + '.e[' + str(i) + ']'

def getFaceName(n, i):
	return n + '.f[' + str(i) + ']'

def findClosesVertexOnMesh(mesh, point):
	'''
	Try to find the closest vertex on the mesh
	'''
	# the core here here is to use http://download.autodesk.com/us/maya/2011help/CommandsPython/polyInfo.html
	# the polyInfo command is like the iterator in openmesh
	# or a guide here http://polycount.com/discussion/77642/maya-select-all-by-componets-verts-edges-faces
	print("mesh is " + str(mesh))
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

def findBestEdgeToCrease(vertex, crease_direction):
	

def polySplitCrease():
	# create a uniform polygon first
	newFace = cmds.polyCreateFacet(n='myPaper', p = [(-1, 0 , -1), (-1, 0, 1), (1, 0, 1), (1, 0, -1)])

	# now need to find edge index from point
	p0 = MVector(-1.0,0.0,-1.0)
	p1 = MVector(-1,0,1)
	closest_vert_0 = findClosesVertexOnMesh(newFace[0],p0)
	print(closest_vert_0)
	closest_vert_1 = findClosesVertexOnMesh(newFace[0],p1)
	print(closest_vert_1)

	epStart = (0,0.0)
	epEnd = (0,1.0)
	ip = []
	# start edge point
	ip.append(epStart)
	# intermediate points
	
	creaseStart = MVector(-1,0,-1)
	creaseEnd = MVector(-1,0,1)
	
	pts = cubicBezierCurve(creaseStart,MVector(0,0,-1), MVector(0,0,1),creaseEnd, segments = 100)
	for p in pts:
		face_idx = 0
		ip.append((face_idx,p[0],p[1],p[2]))
		
	# end edge point
	ip.append(epEnd)
	cmds.polySplit( ip = ip)



polySplitCrease()
