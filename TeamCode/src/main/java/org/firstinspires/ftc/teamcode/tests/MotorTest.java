package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;

@Config
@TeleOp(name = "Motor Test", group = "Tuning")
public class MotorTest extends OpMode {
    public static double intakePower = 0.0;
    public static double turretPower = 0.0;
    public static double flywheelPower = 0.0;
    public static double flywheel2Power = 0.0;

    private DcMotorEx intake;
    private DcMotorEx turret;
    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;

    private MultipleTelemetry multiTelemetry;

    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        // Set all motors to run with encoder for position tracking
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // Set motor powers (all forward direction)
        intake.setPower(intakePower);
        turret.setPower(turretPower);
        flywheel.setPower(flywheelPower);
        flywheel2.setPower(flywheel2Power);
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Report encoder values
        loopRate.update();
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.addLine("--- Powers ---");
        multiTelemetry.addData("intakePower", intakePower);
        multiTelemetry.addData("turretPower", turretPower);
        multiTelemetry.addData("flywheelPower", flywheelPower);
        multiTelemetry.addData("flywheel2Power", flywheel2Power);
        multiTelemetry.addLine("--- Encoder Values ---");
        multiTelemetry.addData("intakeEncoder", intake.getCurrentPosition());
        multiTelemetry.addData("turretEncoder", turret.getCurrentPosition());
        multiTelemetry.addData("flywheelEncoder", flywheel.getCurrentPosition());
        multiTelemetry.addData("flywheel2Encoder", flywheel2.getCurrentPosition());
        multiTelemetry.update();
    }
}
