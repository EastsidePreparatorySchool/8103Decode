package org.firstinspires.ftc.teamcode.lib;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

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

    // subsystems
    public IntakeSubsystem intakeSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public SpindexerSubsystem spindexerSubsystem;
    public TransferSubsystem transferSubsystem;
    public TurretSubsystem turretSubsystem;
}