package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.util.InterpLUT;

@Config
public class Common {
    // Turret PIDF
    public static double TURRET_KP = 0.023;
    public static double TURRET_KI = 0.02;
    public static double TURRET_KD = 0.0002;
    public static double TURRET_KF = 0.07;
    // Turret geometry
    public static double TURRET_FULL_ROTATION_DEGREES = 360.0;
    public static double TURRET_TICKS_PER_MOTOR_REV = 28.0;
    public static double TURRET_GEARBOX_RATIO = 13.7; // goBILDA 435 rpm (13.7:1)
    public static double TURRET_DRIVING_PULLEY_TEETH = 38.0;
    public static double TURRET_DRIVEN_PULLEY_TEETH = 108.0;
    public static double TURRET_TOLERANCE_TICKS = 5.0;
    public static double TURRET_WIRE_WRAP_LIMIT_DEGREES = 90.0;
    // Turret center offset from robot center
    public static double TURRET_OFFSET_X_IN = -3.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;
    // Drivetrain
    public static double INTAKE_FORWARD_POWER = 1.0;
    public static double INTAKE_BACKWARD_POWER = -1.0;
    public static double SLOWMODE_MULTIPLIER = 0.4;
    // Pinpoint odometry configuration
    // Robot 16.5" wide (center to left edge = 8.25"). X pod is 3.1" in from left =>
    // +
    public static double PINPOINT_X_OFFSET_MM = 131;
    // Robot 16.5" long (center to back edge = 8.25"). Y pod is 1.4" in from back =>
    // -6.85 => -174mm
    public static double PINPOINT_Y_OFFSET_MM = -174;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static boolean PINPOINT_RESET_IMU_ON_INIT = true;
    // Spindexer targets defined as servo positions [0..1]
    public static double SPINDEXER_INTAKE_ONE = 0.04;
    public static double SPINDEXER_INTAKE_TWO = 0.41;
    public static double SPINDEXER_INTAKE_THREE = 0.8;
    public static double SPINDEXER_OUTTAKE_ONE = 0.61;
    public static double SPINDEXER_OUTTAKE_TWO = 0.98;
    public static double SPINDEXER_OUTTAKE_THREE = 0.23;
    // Transfer / shooter
    public static double TRANSFER_DOWN = 0.49;
    public static double TRANSFER_UP = 0.3;

    // Shooter velocity controller (RPM-based)
    // Motor/geometry
    public static double SHOOTER_TICKS_PER_MOTOR_REV = 28.0; // Built-in encoder CPR
    public static double SHOOTER_GEAR_RATIO = 1.0; // Motor:flywheel
    public static double SHOOTER_MAX_RPM = 6000.0; // Bare motor free speed
    // Feedforward + feedback coefficients (power units)
    public static double SHOOTER_KS = 0.11; // static friction offset (power)
    public static double SHOOTER_KV = 0.00017; // power per RPM
    public static double SHOOTER_KA = 0.0; // power per (RPM/s)
    public static double SHOOTER_KP = 0.002; // P on velocity error (RPM)
    // Hood initial position
    public static double HOOD_INITIAL_POS = 0.5;
    // Voltage normalization
    public static double NOMINAL_BATTERY_VOLTAGE = 12.0;
    public static double BATTERY_VOLTAGE_SAMPLE_PERIOD_SEC = 0.5;

    public static double HOOD_FAR_POS = 0.5;
    public static double SHOOTER_FAR_RPM = 5700.0;

    public static double DRIVE_DEFAULT_MULT = 0.8;
    public static double DRIVE_SLOW_MULT = 0.5;
    // Field targeting and start pose (inches/degrees)
    public static double RIGHT_FIELD_START_X_IN = 87.0;
    public static double RIGHT_FIELD_START_Y_IN = 8.5;
    public static double LEFT_FIELD_START_X_IN = 57.0;
    public static double LEFT_FIELD_START_Y_IN = 8.5;
    public static double START_HEADING_DEG = 270.0;
    public static double RIGHT_FIELD_TARGET_X_IN = 152.0;
    public static double RIGHT_FIELD_TARGET_Y_IN = 144.0;
    public static double RIGHT_FIELD_ACTUAL_TARGET_X_IN = 144.0;
    public static double RIGHT_FIELD_ACTUAL_TARGET_Y_IN = 144.0;
    public static double LEFT_FIELD_TARGET_X_IN = -8.0;
    public static double LEFT_FIELD_TARGET_Y_IN = 144.0;
    public static double LEFT_FIELD_ACTUAL_TARGET_X_IN = 0;
    public static double LEFT_FIELD_ACTUAL_TARGET_Y_IN = 144.0;
    // Selected field target used by opmodes; default to RIGHT
    public static double START_X_IN = RIGHT_FIELD_START_X_IN;
    public static double START_Y_IN = RIGHT_FIELD_START_Y_IN;
    public static double TARGET_X_IN = RIGHT_FIELD_TARGET_X_IN;
    public static double TARGET_Y_IN = RIGHT_FIELD_TARGET_Y_IN;
    public static double ACTUAL_TARGET_X_IN = RIGHT_FIELD_ACTUAL_TARGET_X_IN;
    public static double ACTUAL_TARGET_Y_IN = RIGHT_FIELD_TARGET_Y_IN;
    // Interpolation Tables
    public static InterpLUT shooterInterpLUT = ShooterHoodData.getShooterInterpLUT();
    public static InterpLUT hoodInterpLUT = ShooterHoodData.getHoodInterpLUT();
}
