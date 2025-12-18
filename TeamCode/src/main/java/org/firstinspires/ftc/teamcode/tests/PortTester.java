package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Port Tester - Control every motor and servo individually via FTC Dashboard
 * All motors have FLOAT zero power behavior for easy port identification.
 */
@Config
@TeleOp(name = "Port Tester", group = "Tuning")
public class PortTester extends OpMode {

    // === DASHBOARD VARIABLES ===
    // Control Hub Motors (ports 0-3)
    public static double port0_FL_power = 0.0;
    public static double port1_FR_power = 0.0;
    public static double port2_BL_power = 0.0;
    public static double port3_BR_power = 0.0;

    // Expansion Hub Motors (ports 0-3)
    public static double port0_intake_power = 0.0;
    public static double port1_turret_power = 0.0;
    public static double port2_flywheel2_power = 0.0;
    public static double port3_flywheel_power = 0.0;

    // Servo Hub Servos (ports 0-5)
    public static double port0_hood_pos = 0.5;
    public static double port1_spindexer_pos = 0.5;
    public static double port2_transfer_pos = 0.5;
    public static double port5_hood2_pos = 0.5;

    // Hardware
    private DcMotorEx FL, FR, BL, BR;
    private DcMotorEx intake, turret, flywheel2, flywheel;
    private Servo hood, spindexer, transfer, hood2;

    private MultipleTelemetry telem;

    @Override
    public void init() {
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Control Hub Motors
        FL = initMotor("FL");
        FR = initMotor("FR");
        BL = initMotor("BL");
        BR = initMotor("BR");

        // Expansion Hub Motors
        intake = initMotor("intake");
        turret = initMotor("turret");
        flywheel2 = initMotor("flywheel2");
        flywheel = initMotor("flywheel");

        // Servo Hub Servos
        hood = initServo("hood");
        spindexer = initServo("spindexer");
        transfer = initServo("transfer");
        hood2 = initServo("hood2");

        telem.addLine("Port Tester Ready - Use FTC Dashboard to control");
        telem.update();
    }

    private DcMotorEx initMotor(String name) {
        try {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            return motor;
        } catch (Exception e) {
            return null;
        }
    }

    private Servo initServo(String name) {
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    @Override
    public void loop() {
        // Apply Control Hub motor powers
        if (FL != null) FL.setPower(port0_FL_power);
        if (FR != null) FR.setPower(port1_FR_power);
        if (BL != null) BL.setPower(port2_BL_power);
        if (BR != null) BR.setPower(port3_BR_power);

        // Apply Expansion Hub motor powers
        if (intake != null) intake.setPower(port0_intake_power);
        if (turret != null) turret.setPower(port1_turret_power);
        if (flywheel2 != null) flywheel2.setPower(port2_flywheel2_power);
        if (flywheel != null) flywheel.setPower(port3_flywheel_power);

        // Apply servo positions
        if (hood != null) hood.setPosition(port0_hood_pos);
        if (spindexer != null) spindexer.setPosition(port1_spindexer_pos);
        if (transfer != null) transfer.setPosition(port2_transfer_pos);
        if (hood2 != null) hood2.setPosition(port5_hood2_pos);

        // Telemetry
        telem.addLine("=== CONTROL HUB MOTORS ===");
        telem.addData("Port 0 (FL)", motorStatus(FL, port0_FL_power));
        telem.addData("Port 1 (FR)", motorStatus(FR, port1_FR_power));
        telem.addData("Port 2 (BL)", motorStatus(BL, port2_BL_power));
        telem.addData("Port 3 (BR)", motorStatus(BR, port3_BR_power));

        telem.addLine("=== EXPANSION HUB MOTORS ===");
        telem.addData("Port 0 (intake)", motorStatus(intake, port0_intake_power));
        telem.addData("Port 1 (turret)", motorStatus(turret, port1_turret_power));
        telem.addData("Port 2 (flywheel2)", motorStatus(flywheel2, port2_flywheel2_power));
        telem.addData("Port 3 (flywheel)", motorStatus(flywheel, port3_flywheel_power));

        telem.addLine("=== SERVO HUB SERVOS ===");
        telem.addData("Port 0 (hood)", servoStatus(hood, port0_hood_pos));
        telem.addData("Port 1 (spindexer)", servoStatus(spindexer, port1_spindexer_pos));
        telem.addData("Port 2 (transfer)", servoStatus(transfer, port2_transfer_pos));
        telem.addData("Port 5 (hood2)", servoStatus(hood2, port5_hood2_pos));

        telem.update();
    }

    private String motorStatus(DcMotorEx motor, double power) {
        if (motor == null) return "NOT FOUND";
        return String.format("pwr=%.2f enc=%d", power, motor.getCurrentPosition());
    }

    private String servoStatus(Servo servo, double pos) {
        if (servo == null) return "NOT FOUND";
        return String.format("pos=%.3f", pos);
    }

    @Override
    public void stop() {
        if (FL != null) FL.setPower(0);
        if (FR != null) FR.setPower(0);
        if (BL != null) BL.setPower(0);
        if (BR != null) BR.setPower(0);
        if (intake != null) intake.setPower(0);
        if (turret != null) turret.setPower(0);
        if (flywheel2 != null) flywheel2.setPower(0);
        if (flywheel != null) flywheel.setPower(0);
    }
}
