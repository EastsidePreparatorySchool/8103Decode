package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class RobotHardware {
    private static RobotHardware instance;
    public ElapsedTime chassisElapsedTime = new ElapsedTime();

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public CachingDcMotorEx dtFL;
    public CachingDcMotorEx dtFR;
    public CachingDcMotorEx dtBL;
    public CachingDcMotorEx dtBR;
    // shooter
    public CachingDcMotorEx flywheel;
    public Servo hood;
    // turret
    public CachingDcMotorEx turret;
    // intake
    public CachingDcMotorEx intake;
    // transfer
    public Servo transfer;
    // spindexer
    public CRServo spindexer;
    public AnalogInput spindexerAnalog;
    // odometry
    public GoBildaPinpointDriver pinpoint;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;
    public LynxModule EXPANSION_HUB;

    public void init(HardwareMap hwMap, Telemetry tele) {
        this.hardwareMap = hwMap;
        this.telemetry = tele;
    }

    public void initDrivetrain() {
        dtFL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "FL"));
        dtFR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "FR"));
        dtBL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "BL"));
        dtBR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "BR"));
        dtFL.setDirection(DcMotorSimple.Direction.REVERSE);
        dtFR.setDirection(DcMotorSimple.Direction.FORWARD);
        dtBL.setDirection(DcMotorSimple.Direction.REVERSE);
        dtBR.setDirection(DcMotorSimple.Direction.FORWARD);
        dtFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dtFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dtBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dtBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dtFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumSubsystem = new MecanumSubsystem();
    }

    public void initTurret() {
        turret = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "turret"));
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretSubsystem = new TurretSubsystem();
    }

    public void initSpindexer() {
        spindexer = new CachingCRServo(hardwareMap.get(CRServo.class, "spindexer"));
        spindexerAnalog = hardwareMap.get(AnalogInput.class, "spindexerAnalog");
        spindexerSubsystem = new SpindexerSubsystem();
    }

    public void initPinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Common.PINPOINT_HARDWARE_NAME);
        configurePinpoint();
        Pose2D startPose = new Pose2D(
                DistanceUnit.INCH,
                Common.PINPOINT_START_X_IN,
                Common.PINPOINT_START_Y_IN,
                AngleUnit.DEGREES,
                Common.PINPOINT_START_HEADING_DEG
        );
        pinpoint.setPosition(startPose);
        if (Common.PINPOINT_RESET_IMU_ON_INIT) {
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(startPose);
        }
        pinpointSubsystem = new PinpointSubsystem();
    }

    private void configurePinpoint() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.setOffsets(Common.PINPOINT_X_OFFSET_MM, Common.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        if (Common.PINPOINT_ENCODER_RESOLUTION_MM_PER_TICK > 0) {
            pinpoint.setEncoderResolution(Common.PINPOINT_ENCODER_RESOLUTION_MM_PER_TICK, DistanceUnit.MM);
        } else if (Common.PINPOINT_POD_TYPE != null) {
            pinpoint.setEncoderResolution(Common.PINPOINT_POD_TYPE);
        }
        pinpoint.setEncoderDirections(Common.PINPOINT_X_DIRECTION, Common.PINPOINT_Y_DIRECTION);
    }

    // subsystems
    public IntakeSubsystem intakeSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public SpindexerSubsystem spindexerSubsystem;
    public TransferSubsystem transferSubsystem;
    public TurretSubsystem turretSubsystem;
    public MecanumSubsystem mecanumSubsystem;
    public PinpointSubsystem pinpointSubsystem;

    public void initLynx() {
        modules = hardwareMap.getAll(LynxModule.class);
        if (modules.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(modules.get(0).getSerialNumber())) {
            CONTROL_HUB = modules.get(0);
            EXPANSION_HUB = modules.get(1);
        } else {
            CONTROL_HUB = modules.get(1);
            EXPANSION_HUB = modules.get(0);
        }

        CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void periodic() {
        if (CONTROL_HUB != null) {
            CONTROL_HUB.clearBulkCache();
        }
        if (EXPANSION_HUB != null) {
            EXPANSION_HUB.clearBulkCache();
        }
    }

    public void powerMotors(double powerFL, double powerFR, double powerBL, double powerBR) {
        dtFL.setPower(powerFL);
        dtFR.setPower(powerFR);
        dtBL.setPower(powerBL);
        dtBR.setPower(powerBR);
    }
}
