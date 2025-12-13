package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.util.InterpLUT;

@Config
public class Common {
    // Turret PIDF
    public static double TURRET_KP = 0.025;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.0007;
    public static double TURRET_KF = 0.11;
    // Turret geometry
    public static double TURRET_FULL_ROTATION_DEGREES = 360.0;
    public static double TURRET_TICKS_PER_MOTOR_REV = 28.0;
    public static double TURRET_GEARBOX_RATIO = 13.7; // goBILDA 435 rpm (13.7:1)
    public static double TURRET_DRIVING_PULLEY_TEETH = 40.0;
    public static double TURRET_DRIVEN_PULLEY_TEETH = 108.0;
    public static double TURRET_TOLERANCE_DEG = 0.8;
    public static double TURRET_WIRE_WRAP_LIMIT_DEGREES = 90.0;
    // Turret center offset from robot center
    public static double TURRET_OFFSET_X_IN = -3.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;
    // Drivetrain
    public static double INTAKE_FORWARD_POWER = 1.0;
    public static double INTAKE_BACKWARD_POWER = -1.0;
    public static double SLOWMODE_MULTIPLIER = 0.4;
    // Pinpoint odometry configuration
    // Robot 16.5" wide (center to left edge = 8.25"). X pod is 6" in from left =>
    // +
    public static double PINPOINT_X_OFFSET_MM = 58;
    // Robot 16.5" long (center to back edge = 8.25"). Y pod is 1.4" in from back =>
    // -6.85 => -174mm
    public static double PINPOINT_Y_OFFSET_MM = -174;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static boolean PINPOINT_RESET_IMU_ON_INIT = true;
    // Spindexer targets defined as servo positions [0..1]
    public static double SPINDEXER_INTAKE_ONE = 0.02;
    public static double SPINDEXER_INTAKE_TWO = 0.24;
    public static double SPINDEXER_INTAKE_THREE = 0.44;
    public static double SPINDEXER_OUTTAKE_ONE = 0.34;
    public static double SPINDEXER_OUTTAKE_TWO = 0.55;
    public static double SPINDEXER_OUTTAKE_THREE = 0.76;
    // Transfer / shooter
    public static double TRANSFER_DOWN = 0.485;
    public static double TRANSFER_UP = 0.3;

    // Shooter velocity controller (RPM-based)
    // Motor/geometry
    public static double SHOOTER_TICKS_PER_MOTOR_REV = 28.0; // Built-in encoder CPR
    public static double SHOOTER_GEAR_RATIO = 1.0; // Motor:flywheel
    public static double SHOOTER_MAX_RPM = 6000.0; // Bare motor free speed
    // Feedforward + feedback coefficients (power units)
    public static double SHOOTER_KS = 0.09; // static friction offset (power)
    public static double SHOOTER_KV = 0.000155; // power per RPM
    public static double SHOOTER_KA = 0.0; // power per (RPM/s)
    public static double SHOOTER_KP = 0.008; // P on velocity error (RPM)
    // Hood initial position
    public static double HOOD_INITIAL_POS = 0.70;
    public static double HOOD_MAX_POS = 0.14;
    // Voltage normalization
    public static double NOMINAL_BATTERY_VOLTAGE = 12.0;
    public static double BATTERY_VOLTAGE_SAMPLE_PERIOD_SEC = 0.5;

    public static double DRIVE_DEFAULT_MULT = 0.8;
    public static double DRIVE_SLOW_MULT = 0.5;
    // Field starting positions (inches) and headings (degrees)
    public static double RIGHT_FAR_START_X_IN = 87.0;
    public static double RIGHT_FAR_START_Y_IN = 8.25;
    public static double RIGHT_FAR_START_HEADING_DEG = 90.0;
    public static double RIGHT_CLOSE_START_X_IN = 121.0;
    public static double RIGHT_CLOSE_START_Y_IN = 120.0;
    public static double RIGHT_CLOSE_START_HEADING_DEG = 0.0;
    public static double LEFT_FAR_START_X_IN = 57.0;
    public static double LEFT_FAR_START_Y_IN = 8.25;
    public static double LEFT_FAR_START_HEADING_DEG = 90.0;
    public static double LEFT_CLOSE_START_X_IN = 23.0;
    public static double LEFT_CLOSE_START_Y_IN = 120.0;
    public static double LEFT_CLOSE_START_HEADING_DEG = 180.0;
    public static double RIGHT_FIELD_TARGET_X_IN = 144.0;
    public static double RIGHT_FIELD_TARGET_Y_IN = 144.0;
    public static double RIGHT_FIELD_ACTUAL_TARGET_X_IN = 144.0;
    public static double RIGHT_FIELD_ACTUAL_TARGET_Y_IN = 144.0;
    public static double LEFT_FIELD_TARGET_X_IN = 0.0;
    public static double LEFT_FIELD_TARGET_Y_IN = 144.0;
    public static double LEFT_FIELD_ACTUAL_TARGET_X_IN = 0;
    public static double LEFT_FIELD_ACTUAL_TARGET_Y_IN = 144.0;
    // Selected field target used by opmodes; default to RIGHT FAR
    public static double START_X_IN = RIGHT_FAR_START_X_IN;
    public static double START_Y_IN = RIGHT_FAR_START_Y_IN;
    public static double START_HEADING_DEG = RIGHT_FAR_START_HEADING_DEG;

    public static double TARGET_X_IN = RIGHT_FIELD_TARGET_X_IN;
    public static double TARGET_Y_IN = RIGHT_FIELD_TARGET_Y_IN;
    public static double ACTUAL_TARGET_X_IN = RIGHT_FIELD_ACTUAL_TARGET_X_IN;
    public static double ACTUAL_TARGET_Y_IN = RIGHT_FIELD_TARGET_Y_IN;
    // Auto-intake ball detection

    // Ball color detection thresholds
    public static double PURPLE_HUE_MIN = 200.0;
    public static double PURPLE_HUE_MAX = 300.0;
    public static double GREEN_HUE_MIN = 60.0;
    public static double GREEN_HUE_MAX = 165.0;
    public static double COLOR_MIN_SATURATION = 0.3;
    public static long COLOR_CONFIRMATION_TIMEOUT_MS = 100;

    // Ball color enum for slot tracking
    public enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN,
        NONE  // Empty slot
    }

    // Interpolation Tables
    public static InterpLUT shooterInterpLUT = ShooterHoodData.getShooterInterpLUT();
    public static InterpLUT hoodInterpLUT = ShooterHoodData.getHoodInterpLUT();
}
