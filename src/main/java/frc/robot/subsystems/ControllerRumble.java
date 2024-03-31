package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerRumble extends SubsystemBase {
    private final XboxController controller;

    public ControllerRumble(int port) {
        controller = new XboxController(port);
    }

    public void setRumbleLight(double rumble) {
        controller.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
    }

    public void setRumbleHeavy(double rumble) {
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
    }
}
