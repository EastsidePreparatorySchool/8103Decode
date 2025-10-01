package org.firstinspires.ftc.teamcode.lib;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    private static RobotHardware instance;
    // drivetrain
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }
    public DcMotorEx dtFL;
    public DcMotorEx dtFR;
    public DcMotorEx dtBL;
    public DcMotorEx dtBR;
    // shooter
    public DcMotorEx flywheel;
    public Servo hood;
    // turret
    public DcMotorEx turret;
    // intake
    public DcMotorEx intake;
    // transfer
    public Servo transfer;
    // spindexer
    public Servo spindexer;

