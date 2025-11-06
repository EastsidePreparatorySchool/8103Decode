package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class CameraSubsystem extends SubsystemBase {
    RobotHardware robot;
    public CameraName camera;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal portal;

    public CameraSubsystem() {
        robot = RobotHardware.getInstance();
        camera = robot.hardwareMap.get(WebcamName.class, "Webcam 1");

        portal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();

        setEnabled(false);
    }

    public void setEnabled(boolean enable) {
        portal.setProcessorEnabled(aprilTagProcessor, enable);
    }
}
