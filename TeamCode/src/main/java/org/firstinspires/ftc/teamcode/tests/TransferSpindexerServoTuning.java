package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.lib.LoopRateAverager;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

@Config
@TeleOp(name = "TransferSpindexerServoTuning", group = "Tuning")
public class TransferSpindexerServoTuning extends CommandOpMode {
    public static double transferPos = 0.0; // default transfer position should be 0
    public static double spindexerPos = 0.0;

    private final RobotHardware robot = RobotHardware.getInstance();
    private MultipleTelemetry multiTelemetry;
    private final LoopRateAverager loopRate = new LoopRateAverager(50);

    @Override
    public void initialize() {
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, multiTelemetry);
        robot.initLynx();
        // Do NOT register spindexer/transfer subsystems here; we want direct servo control
        // Wrap raw servos in CachingServo to match RobotHardware field types
        robot.spindexer = new CachingServo(hardwareMap.get(Servo.class, "spindexer"));
        robot.transfer = new CachingServo(hardwareMap.get(Servo.class, "transfer"));

        // Initialize hardware to requested positions
        applyPositions();
        publishTelemetry();
    }

    @Override
    public void initialize_loop() {
        robot.periodic();
        applyPositions();
        publishTelemetry();
    }

    @Override
    public void run() {
        robot.periodic();
        applyPositions();
        publishTelemetry();
    }

    private void applyPositions() {
        if (robot.transfer != null) robot.transfer.setPosition(transferPos);
        if (robot.spindexer != null) robot.spindexer.setPosition(spindexerPos);
    }

    private void publishTelemetry() {
        loopRate.update();
        multiTelemetry.addData("transfer/pos", transferPos);
        multiTelemetry.addData("spindexer/pos", spindexerPos);
        multiTelemetry.addData("hz", loopRate.getHz());
        multiTelemetry.update();
    }
}
