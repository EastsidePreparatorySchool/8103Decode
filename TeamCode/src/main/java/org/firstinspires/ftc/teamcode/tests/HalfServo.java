package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
@Disabled
@Autonomous(name="half servo")
public class HalfServo extends OpMode {
    Servo s;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);
    @Override
    public void init() {
        s = hardwareMap.get(Servo.class, "transfer");
    }

    @Override
    public void loop() {
        s.setPosition(0.5);
        loopRate.update();
        telemetry.addData("hz", loopRate.getHz());
        telemetry.update();
    }
}
