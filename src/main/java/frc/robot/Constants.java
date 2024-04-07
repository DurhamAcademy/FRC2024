// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean isInReplayTestMode =
            (System.getenv().getOrDefault("TEST_RUN_MODE", "false").equalsIgnoreCase("true"));
    public static final Mode currentMode = (RobotBase.isReal()) ? Mode.REAL : ((isInReplayTestMode)?Mode.REPLAY:Mode.SIM);

    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static Transform3d[] robotToCam;

    static {
        Transform3d ShootSideCamera = new Transform3d(
                new Translation3d(
                        Meters.of(0.258572),//-0.25009 y
                        Meters.of(0.1796796),//-0.1854 x
                        Meters.of(0.280162)//-0.34316 z
                ),
                new Rotation3d(
                        Rotations.of(.5).in(Radians),
                        Degrees.of(-30).in(Radians),
                        Degrees.of(3).in(Radians)
                )
        );
        robotToCam = new Transform3d[]{
                ShootSideCamera,
                new Transform3d(
                        new Translation3d(
                                Meters.of(0.258572).minus(Inches.of(-0.323914)),
                                Meters.of(0.1796796).minus(Inches.of(14.265874)),
                                Meters.of(0.280162).plus(Inches.of(4.938808))
                        ),
                        new Rotation3d(
                                Rotations.of(.5).in(Radians),
                                Degrees.of(-30).plus(Radians.of(0.1839466536)).in(Radians),
                                Degrees.of(3).plus(Radians.of(-1.500654)).in(Radians)
                        )
                )
        };
    }
}
