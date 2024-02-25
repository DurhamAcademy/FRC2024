package frc.robot.commands;


import static edu.wpi.first.units.Units.*;


import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;



public class ShooterCommands {
    InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    private double doTrig(double distance){
        return Math.atan(2.045/distance); //angle must not be bigger than 14 deg at max distance 8.2 m
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
