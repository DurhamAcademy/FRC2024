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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = (RobotBase.isReal()) ? Mode.REAL : Mode.REPLAY;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

    public static Transform3d[] robotToCam = new Transform3d[]{
            new Transform3d(
                    new Translation3d(
                            0.258572,
                            0.1796796,
                            0.280162
                    ),
                    new Rotation3d(
                            0,
                            degreesToRadians(60 - 90),
                            degreesToRadians(3)
                    )
            ),
            new Transform3d(
                    new Translation3d(
                            0.258572,
                            0.1796796,
                            0.280162
                    ),
                    new Rotation3d(
                            0,
                            degreesToRadians(60 - 90),
                            degreesToRadians(3 + 90)
                    )
            )
    };

}
