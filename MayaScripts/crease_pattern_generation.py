from maya import cmds
import maya.api.OpenMaya as om
from maya.api.OpenMaya import MVector

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


def polySplitCrease():
	# create a uniform polygon first
	newFace = cmds.polyCreateFacet(n='myPaper', p = [(-1, 0 , -1), (-1, 0, 1), (1, 0, 1), (1, 0, -1)])
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
