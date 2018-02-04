package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.control;

import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.control.*;
import org.usfirst.frc4444.TerrifiedCameliansLittleSistem.control.PathBuilder.*;
import stealth_libraries.*;
import java.util.ArrayList;

/**
 * Path from the blue center peg to the blue boiler.
 * 
 * Used in CenterGearThenShootModeBlue
 * 
 * @see CenterGearThenShootModeBlue
 * @see PathContainer
 */
public class CenterGearToShootBlue implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0, 0, 0));
        sWaypoints.add(new Waypoint(200, 0, 0, 400));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA:
    // [{"position":{"x":90,"y":160},"speed":0,"radius":0,"comment":""},{"position":{"x":40,"y":160},"speed":60,"radius":40,"comment":""},{"position":{"x":40,"y":220},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: CenterGearToShootBlue
}