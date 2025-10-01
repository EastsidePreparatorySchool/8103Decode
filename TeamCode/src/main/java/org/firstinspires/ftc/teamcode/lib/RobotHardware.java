package org.firstinspires.ftc.teamcode.lib;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class RobotHardware {
    private static RobotHardware instance;
    // drivetrain
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }
    public CachingDcMotorEx dtFL;
    public CachingDcMotorEx dtFR;
    public CachingDcMotorEx dtBL;
    public CachingDcMotorEx dtBR;
    // shooter
    public CachingDcMotorEx flywheel;
    public Servo hood;
    // turret
    public DcMotorEx turret;
    // intake
    public DcMotorEx intake;
    // transfer
    public Servo transfer;
    // spindexer
    public Servo spindexer;

