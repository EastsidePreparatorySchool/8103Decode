package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class RobotHardware {
    private static RobotHardware instance;

    public ElapsedTime chassisElapsedTime = new ElapsedTime();

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }

     // subsystems
    public IntakeSubsystem intakeSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public HoodSubsystem hoodSubsystem;
    public SpindexerSubsystem spindexerSubsystem;
    public TransferSubsystem transferSubsystem;
    public TurretSubsystem turretSubsystem;
    public MecanumSubsystem mecanumSubsystem;
    public PinpointSubsystem pinpointSubsystem;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public CachingDcMotorEx dtFL;
    public CachingDcMotorEx dtFR;
    public CachingDcMotorEx dtBL;
    public CachingDcMotorEx dtBR;
    // shooter
    public CachingDcMotorEx flywheel;
    public CachingDcMotorEx flywheel2;
    public CachingServo hood;
    public CachingServo hood2;
    // turret
    public CachingDcMotorEx turret;
    // intake
    public CachingDcMotorEx intake;
    // transfer
    public CachingServo transfer;
    // spindexer (standard servo mode)
    public CachingServo spindexer;
    // odometry
    public GoBildaPinpointDriver pinpoint;
    // color sensor (REV Color Sensor V3) - used for color detection and distance
    public ColorRangeSensor colorSensor;
    // Battery voltage
    public java.util.List<VoltageSensor> voltageSensors;
    private ElapsedTime voltageTimer;
    private double cachedBatteryVoltage = 12.0;

    public List<LynxModule> modules;

    public double robotX;
    public double robotY;
    public double robotHeadingDeg;

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
        // CCW-positive convention (viewed from above). With an open belt (38->108),
        // FORWARD makes CCW turret rotation yield increasing encoder counts.
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretSubsystem = new TurretSubsystem();
        CommandScheduler.getInstance().registerSubsystem(turretSubsystem);
    }

    public void initShooter() {
        flywheel = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel"));
        flywheel2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel2"));

        // Shooter must run forward
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // Second flywheel runs opposite direction of the first
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        // Use manual control; we compute velocity ourselves
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Let flywheel coast
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterSubsystem = new ShooterSubsystem();
        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
    }

    public void initHood() {
        hood = new CachingServo(hardwareMap.get(Servo.class, "hood"));
        hood2 = new CachingServo(hardwareMap.get(Servo.class, "hood2"));
        hoodSubsystem = new HoodSubsystem();
        CommandScheduler.getInstance().registerSubsystem(hoodSubsystem);
        hoodSubsystem.setHoodPosition(Common.HOOD_INITIAL_POS);
    }
    
    public void initIntake() {
        intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"));
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSubsystem = new IntakeSubsystem();
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
    }

    public void initSpindexer() {
        spindexer = new CachingServo(hardwareMap.get(Servo.class, "spindexer"));
        spindexerSubsystem = new SpindexerSubsystem();
        CommandScheduler.getInstance().registerSubsystem(spindexerSubsystem);
    }

    public void initTransfer() {
        transfer = new CachingServo(hardwareMap.get(Servo.class, "transfer"));
        transferSubsystem = new TransferSubsystem();
        CommandScheduler.getInstance().registerSubsystem(transferSubsystem);
    }

    public void initPinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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

    public void initColorSensor() {
        // REV Color Sensor V3 configured with name "color"
        // ColorRangeSensor provides both color sensing and distance measurement
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "color");
    }

    public void initLynx() {
        modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : modules) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void periodic() {
        for (LynxModule hub : modules) {
            hub.clearBulkCache();
        }
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
