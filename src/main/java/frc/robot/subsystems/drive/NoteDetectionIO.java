package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface NoteDetectionIO {

    String cameraName = "";
    default String getCameraName() {
        return cameraName;
    }
    default void updateInputs(NoteDetectionIOInputs inputs) {
    }

    default void turnOffLEDs() {

    }
    @AutoLog
    class NoteDetectionIOInputs {
        String name = "";
        String jsonDump = "";
        double tx = 0.0;
        double ty = 0.0;
        double ta = 0.0;
        boolean tv = false;
    }

}
