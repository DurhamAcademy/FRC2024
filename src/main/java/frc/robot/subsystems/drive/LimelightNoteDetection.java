package frc.robot.subsystems.drive;

import frc.robot.util.LimelightHelpers;

public class LimelightNoteDetection implements NoteDetectionIO {
    @Override
    public String getCameraName() {
        return "limelight";
    }
    public void turnOffLEDs() {
        LimelightHelpers.setLEDMode_ForceOff(getCameraName());
    }

    @Override
    public void updateInputs(NoteDetectionIO.NoteDetectionIOInputs inputs) {
        inputs.name = getCameraName();
//        inputs.jsonDump = LimelightHelpers.getJSONDump(getCameraName());
        inputs.tx = LimelightHelpers.getTX(getCameraName());
        inputs.ty = LimelightHelpers.getTY(getCameraName());
        inputs.ta = LimelightHelpers.getTA(getCameraName());
        inputs.tv = LimelightHelpers.getTV(getCameraName());
    }
}
