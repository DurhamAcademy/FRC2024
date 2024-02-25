package frc.robot.commands;


import static edu.wpi.first.units.Units.*;


import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.drive.Drive;

import java.util.Map;


public class ShooterCommands {
    static InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    Rotation3d speakerAngle = new Rotation3d();
    public double offset;
    Pose3d speakerPos = new Pose3d(0.0, 0.0,2.045, speakerAngle);
    Rotation3d shooterAngle = new Rotation3d(); //i don't know how to use quaternions so this has to be initialized to smth ltr

    /**
     * @param drive
     * @return (hopefully) the angle in which the shooter has to be at to make the shot to speaker 
     */
    private double doTrig(Drive drive){ //the parameter is the robot, idk how to declare it
        Pose3d drivePose = new Pose3d(drive.getPose().getX(), drive.getPose().getY(), offset, shooterAngle);
        double distance = Math.sqrt(Math.pow(drivePose.relativeTo(speakerPos).getX(), 2) + Math.pow(drivePose.relativeTo(speakerPos).getY(), 2));
        return Math.asin((2.045-offset)/distance);
    }
    private void populateITM() { //im making separate methods for this because I am not sure how much adjustments you would have to make
        for(double i = 0.2; i< 2.0; i+=0.2){
            distanceToAngle.put(i, Math.atan(2.045/i));
        }


    }
    public static Command autoaim(){









    return null;
    }
}
