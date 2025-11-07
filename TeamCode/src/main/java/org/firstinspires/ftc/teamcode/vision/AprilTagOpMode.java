package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="April Tag Test")
public class AprilTagOpMode extends OpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor myAprilTagProcessor;

    @Override
    public void init() {
//        AprilTagProcessor myAprilTagProcessor;

        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();
    }

    @Override
    public void loop() {
        List<AprilTagDetection> myAprilTagDetections;  // list of all detections
        int myAprilTagIdCode;                           // ID code of current detection, in for() loop

        myAprilTagDetections = myAprilTagProcessor.getDetections();

        for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {
            if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                myAprilTagIdCode = myAprilTagDetection.id;

                double myTagPoseX = myAprilTagDetection.ftcPose.x;
                double myTagPoseY = myAprilTagDetection.ftcPose.y;
                double myTagPoseZ = myAprilTagDetection.ftcPose.z;
//                double myTagPosePitch = myAprilTagDetection.ftcPose.pitch;
//                double myTagPoseRoll = myAprilTagDetection.ftcPose.roll;
//                double myTagPoseYaw = myAprilTagDetection.ftcPose.yaw;
                double myTagPoseRange = myAprilTagDetection.ftcPose.range;
                double myTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
                double myTagPoseElevation = myAprilTagDetection.ftcPose.elevation;

                telemetry.addData("detection x", myTagPoseX);
                telemetry.addData("detection y", myTagPoseY);
                telemetry.addData("detection z", myTagPoseZ);
                telemetry.addData("detection bearing", myTagPoseBearing);
            }
        }
    }


}
