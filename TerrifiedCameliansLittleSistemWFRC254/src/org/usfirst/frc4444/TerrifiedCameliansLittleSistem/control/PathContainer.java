package org.usfirst.frc4444.TerrifiedCameliansLittleSistem.control;

import stealth_libraries.*;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    Path buildPath();

    RigidTransform2d getStartPose();

    boolean isReversed();
}
