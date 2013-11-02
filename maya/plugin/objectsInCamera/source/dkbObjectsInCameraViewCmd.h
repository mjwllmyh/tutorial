#ifndef _dkbObjectsInFrustum
#define _dkbObjectsInFrustum 1

#include <maya/MPxCommand.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MBoundingBox.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MFloatMatrix.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MFn.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnCamera.h>
#include <maya/MGlobal.h>
#include <maya/MItDag.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MSelectionList.h>
#include <maya/MSyntax.h>
#include <maya/MVector.h>
#include <assert.h>

#define CHK( x, y ) x; if (stat.error()) { MGlobal::displayError(MString("In function ")+y+" "+stat.errorString()); }

class Plane {
public:
    // members
    MVector normal;
    double distance;
    enum ptRelation { INFRONT, BEHIND, ON };

    // functions
    ptRelation relativeToPlane(const MFloatVector &point);
};

class Frustum {
public:
    // members
    unsigned int numPlanes;
    Plane planes[6];
    enum ptRelation { INSIDE, OUTSIDE, INTERSECTS };

    // functions
    Frustum() : numPlanes(0) { }
    void init(const MFnCamera &fnCam);
    ptRelation relativeToFrustum(const MFloatVectorArray &points);
};

class dkbObjectsInCameraView : public MPxCommand {
public:
    dkbObjectsInCameraView() {
        camInvWorldMtx.setToIdentity();
    }
    virtual	~dkbObjectsInCameraView() { }
	MStatus doIt(const MArgList& args);
    bool isUndoable() const { return false; }
    static void *creator() { return new dkbObjectsInCameraView; }
    static MSyntax newSyntax();

private:
    // members
    Frustum camFrustum;
    MMatrix camInvWorldMtx;

    // functions
    MStatus parseArgs(const MArgList &args);
    void processNode(const MDagPath &dagPath);
};

#endif
