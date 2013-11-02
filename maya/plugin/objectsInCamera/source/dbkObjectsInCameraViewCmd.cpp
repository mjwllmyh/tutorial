#include "dkbObjectsInCameraViewCmd.h"

Plane::ptRelation Plane::relativeToPlane(const MFloatVector &point) {
    double val = (this->normal * point) + this->distance;
    if (val > 0.0)
        return INFRONT;
    else if (val < 0.0)
        return BEHIND;
    return ON;
}

Frustum::ptRelation Frustum::relativeToFrustum(const MFloatVectorArray &points) {
    unsigned int numInside = 0;
    unsigned int numPoints = points.length();

    for (unsigned int j = 0; j < 6; j++) {
        int numBehindThisPlane = 0;
        for (unsigned int i = 0; i < numPoints; i++) {
            if (this->planes[j].relativeToPlane(points[i]) == Plane::BEHIND) {
                numBehindThisPlane++;
            }
        }
        if (numBehindThisPlane == numPoints)
            // all points were behind the same plane
            return OUTSIDE;
        else if (!numBehindThisPlane)
            numInside++;
    }
    if (numInside == this->numPlanes)
        return INSIDE;
    return INTERSECTS;
}

// compute the 6 planes which define the camera's viewing frustum
void Frustum::init(const MFnCamera &fnCam) {
    MStatus stat = MS::kSuccess;
    MVector a, b;

    CHK( double nearClip = fnCam.nearClippingPlane(&stat), "MFnCamera::nearClippingPlane()" );
    CHK( double farClip = fnCam.farClippingPlane(&stat), "MFnCamera::farClippingPlane()" );
    double left = 0, right = 0, bottom = 0, top = 0;
    CHK( double aspectRatio = fnCam.aspectRatio(&stat), "MFnCamera::aspectRatio()" );
    CHK( stat = fnCam.getViewingFrustum(aspectRatio, left, right, bottom, top, false, true), "MFnCamera::getViewingFrustum()" );

    // planes[0] = right plane
    a = MVector(right, top, -nearClip);
    b = MVector(right, bottom, -nearClip);
    planes[0].normal = (a ^ b).normal(); // normal of plane = cross product of vectors a and b
    planes[0].distance = 0.0;

    // planes[1] = left plane
    a = MVector(left, bottom, -nearClip);
    b = MVector(left, top, -nearClip);
    planes[1].normal = (a ^ b).normal();
    planes[1].distance = 0.0;

    // planes[2] = bottom plane
    a = MVector(right, bottom, -nearClip);
    b = MVector(left, bottom, -nearClip);
    planes[2].normal = (a ^ b).normal();
    planes[2].distance = 0.0;

    // planes[3] = top plane
    a = MVector(left, top, -nearClip);
    b = MVector(right, top, -nearClip);
    planes[3].normal = (a ^ b).normal();
    planes[3].distance = 0.0;

    // planes[4] = far plane
    planes[4].normal = MVector(0,0,1);
    planes[4].distance = farClip;

    // planes[5] = near plane
    planes[5].normal = MVector(0,0,-1);
    planes[5].distance = nearClip;
}

MSyntax dkbObjectsInCameraView::newSyntax() {
    MSyntax syntax;
    syntax.setObjectType(MSyntax::kStringObjects,1,1);
    syntax.useSelectionAsDefault(true);
    return syntax;
}

MStatus dkbObjectsInCameraView::parseArgs(const MArgList &args) {
    MStatus stat = MS::kSuccess;
    int len = args.length();
    MArgDatabase argData(newSyntax(),args,&stat);
    if (stat.error())
        return stat;

    // get required camera object parameter
    MStringArray objArr;
    MSelectionList objList;
    MDagPath dp, cameraPath;
    MObject obj;
    argData.getObjects(objArr);
    if (objArr.length() != 1) {
        MGlobal::displayError("Must specify exactly one camera node.");
        return MS::kInvalidParameter;
    }
    MGlobal::getSelectionListByName(objArr[0],objList);
    if (objList.length() <= 0) {
        MGlobal::displayError("No valid camera node was specified.");
        return MS::kInvalidParameter;
    } else if (objList.length() > 1) {
        MGlobal::displayError("More than one object matches string '"+objArr[0]+"'.");
        return MS::kInvalidParameter;
    }
    stat = objList.getDagPath(0,dp);
    if (stat.error()) {
        MGlobal::displayError("No valid camera node was specified.");
        return stat;
    }
    // validate object as camera node
    obj = dp.node();
    if (obj.hasFn(MFn::kCamera))
        cameraPath = dp;
    else {
        stat = dp.extendToShape();
        if (stat.error()) {
            MGlobal::displayError(dp.partialPathName()+"is not a valid camera node.");
            return stat;
        }
        obj = dp.node();
        if (obj.hasFn(MFn::kCamera))
            cameraPath = dp;
        else {
            MGlobal::displayError(dp.partialPathName()+"is not a valid camera node.");
            return MS::kInvalidParameter;
        }
    }

    MFnCamera fnCam(cameraPath);
    // initialize the viewing frustum
    this->camFrustum.init(fnCam);

    // store camera's inverse world matrix
    // (for transforming points to camera's local space)
    CHK( this->camInvWorldMtx = cameraPath.inclusiveMatrixInverse(&stat), "MDagPath::inclusiveMatrixInverse()" );

    return stat;
}

void dkbObjectsInCameraView::processNode(const MDagPath &dagPath) {
    MStatus stat = MS::kSuccess;
    MFloatVectorArray points;
    
    CHK( MFnDagNode fnDag(dagPath, &stat), "MFnDagNode::MFnDagNode()" );
    CHK( MObject obj = dagPath.node(&stat), "MDagPath::node()" );

    // get the bounding box of the node, transformed to world space through
    // post-multiplication with the node's parent matrix, and then
    // transformed to the camera's local space through post-multiplication
    // with the camera's inverse world matrix
    CHK( MMatrix dWorldMtx = dagPath.exclusiveMatrix(&stat), "MDagPath::exclusiveMatrix()" );
    CHK( MBoundingBox bbox = fnDag.boundingBox(&stat), "MFnDagNode::boundingBox()" );
    double minX = bbox.min().x, minY = bbox.min().y, minZ = bbox.min().z;
    double maxX = bbox.max().x, maxY = bbox.max().y, maxZ = bbox.max().z;
    points.append(MFloatVector(bbox.min() * dWorldMtx * this->camInvWorldMtx));
    points.append(MFloatVector(MPoint(maxX,minY,minZ) * dWorldMtx * this->camInvWorldMtx));
    points.append(MFloatVector(MPoint(maxX,minY,maxZ) * dWorldMtx * this->camInvWorldMtx));
    points.append(MFloatVector(MPoint(minX,minY,maxZ) * dWorldMtx * this->camInvWorldMtx));
    points.append(MFloatVector(MPoint(minX,maxY,minZ) * dWorldMtx * this->camInvWorldMtx));
    points.append(MFloatVector(MPoint(maxX,maxY,minZ) * dWorldMtx * this->camInvWorldMtx));
    points.append(MFloatVector(bbox.max() * dWorldMtx * this->camInvWorldMtx));
    points.append(MFloatVector(MPoint(minX,maxY,maxZ) * dWorldMtx * this->camInvWorldMtx));

    // check all the points to see if they are within the camera frustum
    Frustum::ptRelation relation = this->camFrustum.relativeToFrustum(points);
    if (relation == Frustum::INSIDE) // bounding box is completely inside the frustum
        // add node to list of objects in frustum
        appendToResult(dagPath.partialPathName());
    else if (relation == Frustum::INTERSECTS) { // bounding box intersects frustum edge
        CHK( unsigned int numChildren = dagPath.childCount(&stat), "MDagPath::childCount()" );

        // if node has children, recursively process them
        if (numChildren) {
            for (unsigned int i = 0; i < numChildren; i++) {
                CHK( MObject objChild = dagPath.child(i, &stat), "MDagPath::child()" );
                MDagPath dagPathChild;
                CHK( stat = MDagPath::getAPathTo(objChild, dagPathChild), "MDagPath::getAPathTo()" );

                if (objChild.hasFn(MFn::kShape))
                    if (objChild.hasFn(MFn::kMesh) ||
                        objChild.hasFn(MFn::kNurbsSurface) ||
                        objChild.hasFn(MFn::kNurbsCurve) ||
                        objChild.hasFn(MFn::kSubdiv) ||
                        objChild.hasFn(MFn::kLattice))
                        if (numChildren == 1)
                            appendToResult(dagPath.partialPathName());
                        else
                            processNode(dagPathChild);
                    else if (numChildren == 1)
                        appendToResult(dagPath.partialPathName());
                    else
                        processNode(dagPathChild);
                else
                    processNode(dagPathChild);
            }
        } else {
            // node has no children, so add it to list since it's partially in view
            appendToResult(dagPath.partialPathName());
        }
    }
}

MStatus dkbObjectsInCameraView::doIt(const MArgList &args) {
	MStatus stat = MS::kSuccess;

    // parse args
    stat = parseArgs(args);
    if (stat.error())
        return stat;

    clearResult();

    // iterate over only top-level dag nodes (processing children recursively),
    // testing their spatial relationship to the camera frustum
    MItDag itDag(MItDag::kBreadthFirst, MFn::kTransform, &stat);
    for (; !itDag.isDone(); itDag.next()) {
        MObject objDag = itDag.item();
        MString path = itDag.partialPathName(&stat);
        CHK( unsigned int depth = itDag.depth(&stat), "MItDag::depth()" );
        if (depth > 1) // done with all top-level nodes, return
            break;
        MDagPath dagPath;
        CHK( stat = itDag.getPath(dagPath), "MItDag::getPath()" );
        MFnDagNode fnDag(dagPath, &stat);
        if (stat.error()) // object is not a DAG node, skip it
            continue;
        if (!fnDag.canBeWritten()) // skip things like manipulators and groundPlane_transform
            continue;
        processNode(dagPath);
    }
    return MS::kSuccess;
}
