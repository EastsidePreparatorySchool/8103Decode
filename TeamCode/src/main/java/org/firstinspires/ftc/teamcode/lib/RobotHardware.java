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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.seattlesolvers.solverslib.command.CommandScheduler;

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
    // Battery voltage
    public java.util.List<VoltageSensor> voltageSensors;
    private ElapsedTime voltageTimer;
    private double cachedBatteryVoltage = 12.0;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;
    public LynxModule EXPANSION_HUB;

    public void init(HardwareMap hwMap, Telemetry tele) {
        this.hardwareMap = hwMap;
        this.telemetry = tele;
        // Prepare voltage sensors cache
        this.voltageSensors = hardwareMap.getAll(VoltageSensor.class);
        this.voltageTimer = new ElapsedTime();
        this.cachedBatteryVoltage = readBatteryVoltageInstant();
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
        CommandScheduler.getInstance().registerSubsystem(mecanumSubsystem);
    }

    public void initTurret() {
        turret = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "turret"));
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretSubsystem = new TurretSubsystem();
        CommandScheduler.getInstance().registerSubsystem(turretSubsystem);
    }

    public void initShooter() {
        flywheel = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel"));
        hood = hardwareMap.get(Servo.class, "hood");

        // Shooter must run forward
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // Use manual control; we compute velocity ourselves
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Let flywheel coast
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterSubsystem = new ShooterSubsystem();
        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);

        // Start hood at 0.5
        hood.setPosition(Common.HOOD_INITIAL_POS);
    }

    public void initSpindexer() {
        spindexer = new CachingCRServo(hardwareMap.get(CRServo.class, "spindexer"));
        spindexerAnalog = hardwareMap.get(AnalogInput.class, "spindexerAnalog");
        spindexerSubsystem = new SpindexerSubsystem();
        CommandScheduler.getInstance().registerSubsystem(spindexerSubsystem);
    }

    public void initPinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Common.PINPOINT_HARDWARE_NAME);
        configurePinpoint();
        if (Common.PINPOINT_RESET_IMU_ON_INIT) {
            pinpoint.resetPosAndIMU();
        }
        pinpointSubsystem = new PinpointSubsystem();
        CommandScheduler.getInstance().registerSubsystem(pinpointSubsystem);
        pinpointSubsystem.initializePose(0.0, 0.0, 0.0);
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(Common.PINPOINT_X_OFFSET_MM, Common.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(Common.PINPOINT_POD_TYPE);
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
        CONTROL_HUB.clearBulkCache();
        EXPANSION_HUB.clearBulkCache();
    }

    // Battery voltage helpers (cached)
    private double readBatteryVoltageInstant() {
        double best = 0.0;
        if (voltageSensors != null) {
            for (VoltageSensor sensor : voltageSensors) {
                double v = sensor.getVoltage();
                if (v > best && v < 20.0) {
                    best = v;
                }
            }
        }
        return (best > 0.0) ? best : Common.NOMINAL_BATTERY_VOLTAGE;
    }

    public double getBatteryVoltage() {
        if (voltageTimer == null) {
            voltageTimer = new ElapsedTime();
        }
        double now = voltageTimer.seconds();
        if (now >= Common.BATTERY_VOLTAGE_SAMPLE_PERIOD_SEC) {
            cachedBatteryVoltage = readBatteryVoltageInstant();
            voltageTimer.reset();
        }
        return cachedBatteryVoltage;
    }

    public void powerMotors(double powerFL, double powerFR, double powerBL, double powerBR) {
        dtFL.setPower(powerFL);
        dtFR.setPower(powerFR);
        dtBL.setPower(powerBL);
        dtBR.setPower(powerBR);
    }
}
