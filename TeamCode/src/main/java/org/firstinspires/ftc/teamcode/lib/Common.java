package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Config
public class Common {
    // Turret PIDF
    public static double TURRET_KP = 0.0005;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.0;
    public static double TURRET_KF = 0.0;
    // Turret geometry
    public static double TURRET_FULL_ROTATION_DEGREES = 360.0;
    public static double TURRET_TICKS_PER_MOTOR_REV = 28.0;
    public static double TURRET_GEARBOX_RATIO = 3.7;
    public static double TURRET_DRIVING_PULLEY_TEETH = 38.0;
    public static double TURRET_DRIVEN_PULLEY_TEETH = 108.0;
    public static double TURRET_TOLERANCE_TICKS = 10.0;
    public static double TURRET_WIRE_WRAP_LIMIT_DEGREES = 530.0;
    // Spindexer PIDF
    public static double SPINDEXER_KP = 0.001;
    public static double SPINDEXER_KI = 0.0;
    public static double SPINDEXER_KD = 0.0;
    public static double SPINDEXER_KF = 0.0;
    // Spindexer measurements
    public static double AXON_ANALOG_RANGE = 3.3;
    public static double AXON_DEGREE_RANGE = 360.0;
    // Drivetrain
    public static double INTAKE_FORWARD_POWER = 1.0;
    public static double INTAKE_BACKWARD_POWER = -1.0;
    public static double SLOWMODE_MULTIPLIER = 0.4;
    // Pinpoint odometry configuration
    public static String PINPOINT_HARDWARE_NAME = "pinpoint";
    public static double PINPOINT_X_OFFSET_MM = -84.0;
    public static double PINPOINT_Y_OFFSET_MM = -168.0;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static boolean PINPOINT_RESET_IMU_ON_INIT = true;
    // Spindexer targets defined in degrees
    public static double SPINDEXER_INTAKE_ONE = 0.0;
    public static double SPINDEXER_INTAKE_TWO = 0.0;
    public static double SPINDEXER_INTAKE_THREE = 0.0;
    public static double SPINDEXER_OUTTAKE_ONE = 0.0;
    public static double SPINDEXER_OUTTAKE_TWO = 0.0;
    public static double SPINDEXER_OUTTAKE_THREE = 0.0;
    // Transfer / shooter
    public static double TRANSFER_DOWN = 0.0;
    public static double TRANSFER_UP = 0.0;
    public static double FLYWHEEL_ON = 1.0;
}
