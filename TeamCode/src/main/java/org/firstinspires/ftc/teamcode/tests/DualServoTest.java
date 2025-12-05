package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;

@Config
@Autonomous(name="dual servo test", group = "Tuning")
public class DualServoTest extends OpMode {
    public static double servoPos = 0.5;
    public static double servo2Pos = 0.5;

    Servo servo;
    Servo servo2;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void loop() {
        servo.setPosition(servoPos);
        servo2.setPosition(servo2Pos);

        loopRate.update();
        telemetry.addData("hz", loopRate.getHz());
        telemetry.addData("servoPos", servoPos);
        telemetry.addData("servo2Pos", servo2Pos);
        telemetry.update();
    }
}
