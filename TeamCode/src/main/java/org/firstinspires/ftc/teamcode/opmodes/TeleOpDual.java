package nullrobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayDeque;

import nullrobotics.commandbase.groupcommands.ClawToggle;
import nullrobotics.commandbase.groupcommands.ExtendoToggle;
import nullrobotics.commandbase.groupcommands.L2HangRetract;
import nullrobotics.commandbase.groupcommands.LiftToggle;
import nullrobotics.commandbase.groupcommands.OuttakeToggle;
import nullrobotics.commandbase.groupcommands.PivotDepositAndReturn;
import nullrobotics.commandbase.groupcommands.PivotInterIntake;
import nullrobotics.commandbase.subsystemcommands.composedcommands.ClawCommand;
import nullrobotics.commandbase.subsystemcommands.composedcommands.ExtendoCommand;
import nullrobotics.commandbase.subsystemcommands.composedcommands.FollowPathCommand;
import nullrobotics.commandbase.subsystemcommands.composedcommands.LiftCommand;
import nullrobotics.commandbase.subsystemcommands.composedcommands.OuttakeCommand;
import nullrobotics.commandbase.subsystemcommands.composedcommands.PivotCommand;
import nullrobotics.commandbase.subsystemcommands.composedcommands.SpecClawCommand;
import nullrobotics.commandbase.subsystemcommands.statecommands.OuttakeStateCommand;
import nullrobotics.commandbase.subsystemcommands.statecommands.WristPosCommand;
import nullrobotics.commandbase.subsystemcommands.composedcommands.WristTickCommand;
import nullrobotics.lib.AutoPoints;
import nullrobotics.lib.Common;
import nullrobotics.lib.RobotHardware;
import nullrobotics.subsystems.ClawSubsystem;
import nullrobotics.subsystems.ExtendoSubsystem;
import nullrobotics.subsystems.LiftSubsystem;
import nullrobotics.subsystems.OuttakeSubsystem;
import nullrobotics.subsystems.PivotSubsystem;
import nullrobotics.subsystems.SpecClawSubsystem;

@Config
@TeleOp(name="Dual TeleOp")
public class TeleOpDual extends CommandOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    ArrayDeque<Double> hzDeque = new ArrayDeque<>(101);

    double ly;
    double lx;
    double rx;
    double flpower = 0;
    double frpower = 0;
    double blpower = 0;
    double brpower = 0;
    double denominator = 1;
    double multiplier = 1;
    double prevTime = 0;
    double curTime = 0;
    double avgHz;
    double driveCurrent;
    double liftCurrent;
    double otherCurrent;
    double cVoltage;
    double eVoltage;
//    public static boolean toggleLift = false;
//    boolean prevToggle = false;

    Follower follower;
    Pose startPose = AutoPoints.POST_AUTO_POSE;
    public SequentialCommandGroup toBucketGroup;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        // TODO This part may error. Not sure why, I don't have driver hub. Please fix if error!
        //  Adding "Constants.setConstants(FConstants.class, LConstants.class);" fixed it on the other file
        //  But here, it doesn't seem to work, these constants aren't defined
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(AutoPoints.BlueDepositBasket);

        CommandScheduler.getInstance().schedule(
                false,
                new ClawCommand(ClawSubsystem.ClawState.OPEN, robot.clawSubsystem),
                new PivotCommand(PivotSubsystem.PivotState.INTERMEDIATE, robot.liftSubsystem, robot.pivotSubsystem, robot.outtakeSubsystem),
                new ExtendoCommand(ExtendoSubsystem.ExtendoState.RETRACTED, robot.extendoSubsystem),
                new OuttakeCommand(OuttakeSubsystem.OuttakeState.RETRACTED, robot.pivotSubsystem, robot.outtakeSubsystem),
                new LiftCommand(LiftSubsystem.LiftState.RETRACTED, robot.liftSubsystem, robot.pivotSubsystem),
                new WristPosCommand(Common.WRIST_SERVO_NEUTRAL),
                new SpecClawCommand(SpecClawSubsystem.ClawState.OPEN, robot.specClawSubsystem),
                new RunCommand(robot::periodic),
//                new RunCommand(() -> telemetry.addData("follower x", follower.getPose().getX())),
//                new RunCommand(() -> telemetry.addData("follower y", follower.getPose().getY())),
//                new RunCommand(() -> telemetry.addData("follower heading", follower.getPose().getHeading())),
                new RunCommand(() -> telemetry.addData("bucket x", AutoPoints.BlueDepositBasket.getX())),
                new RunCommand(() -> telemetry.addData("bucket y", AutoPoints.BlueDepositBasket.getY())),
                new RunCommand(() -> telemetry.addData("bucket heading", AutoPoints.BlueDepositBasket.getHeading())),
                new RunCommand(telemetry::update)
        );

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(false, new OuttakeToggle(robot.pivotSubsystem, robot.outtakeSubsystem)));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(false, new ExtendoToggle(robot.liftSubsystem, robot.extendoSubsystem, robot.pivotSubsystem, robot.outtakeSubsystem)));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(new InstantCommand(() -> {
                    robot.liftSubsystem.usePID(false);
                    robot.leftLift.setPower(-0.4);
                    robot.rightLift.setPower(-0.4);
                }))
                .whenReleased(new InstantCommand(() -> {
                    robot.leftLift.setPower(0);
                    robot.rightLift.setPower(0);
                    robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.liftSubsystem.usePID(true);
                }));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT) // Calibrate positioning
//                .whenPressed(()-> CommandScheduler.getInstance().schedule(false,
//                        new OuttakeToggle(robot.pivotSubsystem, robot.outtakeSubsystem),
//                        new InstantCommand(follower::update),
//                        new InstantCommand(() -> AutoPoints.BlueDepositBasket.setX(follower.getPose().getX())),
//                        new InstantCommand(() -> AutoPoints.BlueDepositBasket.setY(follower.getPose().getY())),
//                        new InstantCommand(() -> AutoPoints.BlueDepositBasket.setHeading(follower.getPose().getHeading()))
//                ));
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT) // Debugging update
//                .whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(follower::update)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(false, new LiftToggle(LiftSubsystem.LiftState.L2_HANG_EXTENDED, robot.liftSubsystem, robot.pivotSubsystem, robot.outtakeSubsystem)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(false, new L2HangRetract(robot, robot.liftSubsystem, robot.extendoSubsystem, robot.pivotSubsystem, robot.outtakeSubsystem)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(false, new ClawToggle(robot.clawSubsystem)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(false, new PivotInterIntake(robot.liftSubsystem, robot.pivotSubsystem, robot.outtakeSubsystem, robot.clawSubsystem)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(()-> CommandScheduler.getInstance().schedule(false, new WristTickCommand(-1, robot.clawSubsystem)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(()-> CommandScheduler.getInstance().schedule(false, new WristTickCommand(1, robot.clawSubsystem)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(false, new LiftToggle(LiftSubsystem.LiftState.HIGH_BASKET, robot.liftSubsystem, robot.pivotSubsystem, robot.outtakeSubsystem)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(false, new LiftToggle(LiftSubsystem.LiftState.LOW_BASKET, robot.liftSubsystem, robot.pivotSubsystem, robot.outtakeSubsystem)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(false, new PivotDepositAndReturn(robot.liftSubsystem, robot.extendoSubsystem, robot.pivotSubsystem, robot.outtakeSubsystem, robot.clawSubsystem)));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
//        if(toggleLift != prevToggle) {
//            CommandScheduler.getInstance().schedule(new LiftToggle());
//            prevToggle = toggleLift;
//        }
        /*
         * CONTROLLER CONFIGURATION:
         * LS: Drive
         * RS: Turn
         * Dpad up:
         * Dpad right: Tick extendo right
         * Dpad left: Tick extendo left
         * Dpad down: Toggle extendo
         *
         * LB: Wrist rotate right
         * RB: Wrist rotate left
         *
         * LT:
         * RT: Slowmode
         *
         * A: Claw toggle
         * B: Pivot toggle
         * X: Outtake toggle
         * Y: Lift toggle
         */
        ly = Math.pow(-gamepad1.left_stick_y, 3);
        lx = Math.pow(gamepad1.left_stick_x, 3);
        //rx = Math.pow(gamepad1.right_stick_x, 3) / 1.2;

//        if (gamepad1.right_trigger > 0.3) {
//            multiplier = Common.SLOWMODE_MULTIPLIER;
//        } else {
//            multiplier = Common.DEFAULT_DRIVE_SPEED;
//        }
        rx = 0;
        if(gamepad1.right_trigger > 0.02) {
            rx = Math.pow(gamepad1.right_trigger, 3) / 1.65;
        } else if (gamepad1.left_trigger > 0.02) {
            rx = -Math.pow(gamepad1.left_trigger, 3) / 1.65;
        }

        telemetry.addData("lx", lx);
        telemetry.addData("rx", rx);
        // drive motor powers
        multiplier = Common.DEFAULT_DRIVE_SPEED;
        if(robot.extendoSubsystem.state != ExtendoSubsystem.ExtendoState.RETRACTED) {
            multiplier = Common.SLOWMODE_MULTIPLIER;
        }
        denominator = Math.max(multiplier * (Math.abs(ly) + Math.abs(lx) + Math.abs(rx)), 1);
        flpower = multiplier * (ly + lx + rx) / denominator;
        frpower = multiplier * (ly - lx - rx) / denominator;
        blpower = multiplier * (ly - lx + rx) / denominator;
        brpower = multiplier * (ly + lx - rx) / denominator;
        if(!robot.hanging) {
            robot.powerMotors(flpower, frpower, blpower, brpower);
        }
        // telemetry
        switch(robot.clawSubsystem.clawState) {
            case CLOSED:
                telemetry.addData("Claw", "Closed");
                break;
            case OPEN:
                telemetry.addData("Claw", "Open");
        }

        telemetry.addData("Wrist", robot.clawSubsystem.wristPos);
        switch(robot.extendoSubsystem.state) {
            case RETRACTED:
                telemetry.addData("Extendo", "Retracted");
                break;
            case EXTENDED:
                telemetry.addData("Extendo", "Extended");
                break;
            case PARTIAL:
                telemetry.addData("Extendo", "Partial");
        }

        switch(robot.pivotSubsystem.state) {
            case INTAKE:
                telemetry.addData("Pivot", "Intake");
                break;
            case OUTTAKE:
                telemetry.addData("Pivot", "Outtake");
                break;
            case INTERMEDIATE:
                telemetry.addData("Pivot", "Intermediate");
                break;
        }
        switch(robot.outtakeSubsystem.state) {
            case RETRACTED:
                telemetry.addData("Outtake", "Intake");
                break;
            case EXTENDED:
                telemetry.addData("Outtake", "Outtake");
                break;
        }
        switch(robot.liftSubsystem.state) {
            case HIGH_BASKET:
                telemetry.addData("Lift", "High Basket");
                break;
            case LOW_BASKET:
                telemetry.addData("Lift", "Low Basket");
                break;
            case RETRACTED:
                telemetry.addData("Lift", "Retracted");
                break;
        }
//        driveCurrent = robot.dtFL.getCurrent(CurrentUnit.AMPS) + robot.dtFR.getCurrent(CurrentUnit.AMPS) + robot.dtBL.getCurrent(CurrentUnit.AMPS) + robot.dtBR.getCurrent(CurrentUnit.AMPS);
//        liftCurrent = robot.leftLift.getCurrent(CurrentUnit.AMPS) + robot.rightLift.getCurrent(CurrentUnit.AMPS);
//        otherCurrent = robot.CONTROL_HUB.getCurrent(CurrentUnit.AMPS) + robot.EXPANSION_HUB.getCurrent(CurrentUnit.AMPS) - driveCurrent - liftCurrent;
//        cVoltage = robot.CONTROL_HUB.getInputVoltage(VoltageUnit.VOLTS);
//        eVoltage = robot.EXPANSION_HUB.getInputVoltage(VoltageUnit.VOLTS);
        curTime = System.nanoTime();
        hzDeque.addFirst(1000000000 / (curTime - prevTime));
        int len = 0;
        avgHz = 0;
        for(double hz : hzDeque) {
            avgHz += hz;
            len++;
        }
        if(len > 100) {
            hzDeque.removeLast();
        }
        avgHz /= len;
        telemetry.addData("rolling avg hz", avgHz);
//        telemetry.addData("driveCurrent", driveCurrent);
//        telemetry.addData("liftCurrent", liftCurrent);
//        telemetry.addData("otherCurrent", otherCurrent);
//        telemetry.addData("cVoltage", cVoltage);
//        telemetry.addData("eVoltage", eVoltage);
        prevTime = curTime;
        telemetry.update();
    }
}