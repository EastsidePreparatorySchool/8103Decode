package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="half servo")
public class HalfServo extends OpMode {
    Servo s;
    @Override
    public void init() {
        s = hardwareMap.get(Servo.class, "transfer");
    }

    @Override
    public void loop() {
        s.setPosition(0.5);
    }
}
