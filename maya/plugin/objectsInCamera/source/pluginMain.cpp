//
// Copyright (C) Dane Barney
// 
// File: pluginMain.cpp
//
// Author: Maya Plug-in Wizard 2.0
//

#include "dkbObjectsInCameraViewCmd.h"

#include <maya/MFnPlugin.h>

MStatus initializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is loaded into Maya.  It 
//		registers all of the services that this plug-in provides with 
//		Maya.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	MStatus   status;
	MFnPlugin plugin( obj, "Dane Barney", "2012", "Any");
    status = plugin.registerCommand( "dkbObjectsInCameraView", dkbObjectsInCameraView::creator, dkbObjectsInCameraView::newSyntax );
	if (!status)
		status.perror("registerCommand");
    return status;
}

MStatus uninitializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is unloaded from Maya. It 
//		deregisters all of the services that it was providing.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterCommand( "dkbObjectsInCameraView" );
	if (!status)
		status.perror("deregisterCommand");
	return status;
}
