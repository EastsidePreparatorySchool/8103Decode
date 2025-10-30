package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Common {
    public static double TURRET_KP = 0.0005;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.0;
    public static double TURRET_KF = 0.0;
    public static double SPINDEXER_KP = 0.001;
    public static double SPINDEXER_KI = 0.0;
    public static double SPINDEXER_KD = 0.0;
    public static double SPINDEXER_KF = 0.0;
    public static double INTAKE_FORWARD_POWER = 1.0;
    public static double INTAKE_BACKWARD_POWER = -1.0;
  
    public static double SLOWMODE_MULTIPLIER = 0.4;
    public static double AXON_ANALOG_RANGE = 3.3;
    public static double AXON_DEGREE_RANGE = 360;
    public static double TURRET_FULL_ROTATION_DEGREES = 360.0;
    public static double TURRET_TICKS_PER_MOTOR_REV = 28.0;
    public static double TURRET_GEARBOX_RATIO = 3.7;
    public static double TURRET_DRIVING_PULLEY_TEETH = 38.0;
    public static double TURRET_DRIVEN_PULLEY_TEETH = 108.0;
    public static double TURRET_TOLERANCE_TICKS = 10.0;
    // Spindexer targets defined in degrees
    public static double SPINDEXER_INTAKE_ONE = 0;
    public static double SPINDEXER_INTAKE_TWO = 0;
    public static double SPINDEXER_INTAKE_THREE = 0;
    public static double SPINDEXER_OUTTAKE_ONE = 0;
    public static double SPINDEXER_OUTTAKE_TWO = 0;
    public static double SPINDEXER_OUTTAKE_THREE = 0;

    public static double TRANSFER_DOWN = 0;
    public static double TRANSFER_UP = 0;
    public static double FLYWHEEL_ON = 1.0;

}
