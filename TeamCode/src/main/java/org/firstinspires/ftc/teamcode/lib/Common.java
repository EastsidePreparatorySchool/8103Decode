package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Config
public class Common {
    // Turret PIDF
    public static double TURRET_KP = 0.035;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.002;
    public static double TURRET_KF = 0.0;
    // Turret geometry
    public static double TURRET_FULL_ROTATION_DEGREES = 360.0;
    public static double TURRET_TICKS_PER_MOTOR_REV = 28.0;
    public static double TURRET_GEARBOX_RATIO = 13.7; // goBILDA 435 rpm (13.7:1)
    public static double TURRET_DRIVING_PULLEY_TEETH = 38.0;
    public static double TURRET_DRIVEN_PULLEY_TEETH = 108.0;
    public static double TURRET_TOLERANCE_TICKS = 10.0;
    public static double TURRET_WIRE_WRAP_LIMIT_DEGREES = 90.0;
    // Drivetrain
    public static double INTAKE_FORWARD_POWER = 1.0;
    public static double INTAKE_BACKWARD_POWER = -1.0;
    public static double SLOWMODE_MULTIPLIER = 0.4;
    // Pinpoint odometry configuration
    public static double PINPOINT_X_OFFSET_MM = -84.0;
    public static double PINPOINT_Y_OFFSET_MM = -168.0;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static boolean PINPOINT_RESET_IMU_ON_INIT = true;
    // Spindexer targets defined as servo positions [0..1]
    public static double SPINDEXER_INTAKE_ONE = 0.0;
    public static double SPINDEXER_INTAKE_TWO = 0.35;
    public static double SPINDEXER_INTAKE_THREE = 0.73;
    public static double SPINDEXER_OUTTAKE_ONE = 0.55;
    public static double SPINDEXER_OUTTAKE_TWO = 0.9;
    public static double SPINDEXER_OUTTAKE_THREE = 0.16;
    // Transfer / shooter
    public static double TRANSFER_DOWN = 0.5;
    public static double TRANSFER_UP = 0.27;

    // Shooter velocity controller (RPM-based)
    // Motor/geometry
    public static double SHOOTER_TICKS_PER_MOTOR_REV = 28.0; // Built-in encoder CPR
    public static double SHOOTER_GEAR_RATIO = 1.0;            // Motor:flywheel
    public static double SHOOTER_MAX_RPM = 6000.0;            // Bare motor free speed
    // Feedforward + feedback coefficients (power units)
    public static double SHOOTER_KS = 0.13; // static friction offset (power)
    public static double SHOOTER_KV = 0.00018; // power per RPM
    public static double SHOOTER_KA = 0.0; // power per (RPM/s)
    public static double SHOOTER_KP = 0.00015; // P on velocity error (RPM)
    // Hood initial position
    public static double HOOD_INITIAL_POS = 0.47;
    // Voltage normalization
    public static double NOMINAL_BATTERY_VOLTAGE = 12.0;
    public static double BATTERY_VOLTAGE_SAMPLE_PERIOD_SEC = 0.5;
}
