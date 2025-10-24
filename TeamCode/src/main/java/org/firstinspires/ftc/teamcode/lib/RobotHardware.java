package org.firstinspires.ftc.teamcode.lib;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class RobotHardware {
    private static RobotHardware instance;
    public ElapsedTime chassisElapsedTime = new ElapsedTime();
    // drivetrain
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }

    public HardwareMap hardwareMap;;
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
    public Servo spindexer;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;
    public LynxModule EXPANSION_HUB;

    public void init(HardwareMap hwMap, Telemetry tele) {
        this.hardwareMap = hwMap;
        this.telemetry = tele;
        initDrivetrain();
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
    }

    public void initLynx() {
        modules = hardwareMap.getAll(LynxModule.class);
        if(modules.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(modules.get(0).getSerialNumber())) {
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

    public void powerMotors(double powerFL, double powerFR, double powerBL, double powerBR) {
        dtFL.setPower(powerFL);
        dtFR.setPower(powerFR);
        dtBL.setPower(powerBL);
        dtBR.setPower(powerBR);
    }
}
