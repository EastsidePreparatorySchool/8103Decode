package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.Common;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class ShooterSubsystem extends SubsystemBase {
    public enum ShooterState {
        ON,
        OFF
    }

    private final RobotHardware robot;
    public volatile ShooterState state;

    // Hood servo position [0.0, 1.0]
    public double hoodPos;

    // Velocity control (RPM)
    public double targetRpm = 0.0;
    public double currentRpm = 0.0;
    public double lastRpm = 0.0;
    public int lastTicks = 0;
    public double power = 0.0; // last commanded power

    // Feedforward + feedback (tunable via Common)
    private double kS = Common.SHOOTER_KS;
    private double kV = Common.SHOOTER_KV;
    private double kA = Common.SHOOTER_KA;
    private double kP = Common.SHOOTER_KP;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0.0;

    public ShooterSubsystem() {
        robot = RobotHardware.getInstance();
        setShooterState(ShooterState.OFF);
        hoodPos = Common.HOOD_INITIAL_POS;
        // initialize timer baseline
        timer.reset();
        lastTime = timer.seconds();
        lastTicks = robot.flywheel.getCurrentPosition();
    }

    public void setShooterState (ShooterState shooterState) {
        state = shooterState;
        if (state == ShooterState.OFF) {
            power = 0.0;
            robot.flywheel.setPower(0.0);
        }
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, rpm); // forward only
    }

    public void setHoodPosition(double pos) {
        hoodPos = pos;
    }

    public void tickServoPosition(int inc) {
        hoodPos += inc;
    }

    public void applyVelocityCoefficients(double kS, double kV, double kA, double kP) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kP = kP;
    }

    public double getTicksPerFlywheelRev() {
        return Common.SHOOTER_TICKS_PER_MOTOR_REV * Common.SHOOTER_GEAR_RATIO;
    }

    public double ticksPerSecondToRpm(double tps) {
        double ticksPerRev = getTicksPerFlywheelRev();
        if (ticksPerRev == 0) return 0.0;
        return (tps / ticksPerRev) * 60.0;
    }

    public double rpmToTicksPerSecond(double rpm) {
        double ticksPerRev = getTicksPerFlywheelRev();
        return (rpm / 60.0) * ticksPerRev;
    }

    public void updateHardware() {
        // Update servo each loop
        robot.hood.setPosition(hoodPos);

        // If off, ensure motor is stopped and keep telemetry
        if (state == ShooterState.OFF) {
            robot.flywheel.setPower(0.0);
            robot.telemetry.addData("shooter/state", state);
            robot.telemetry.addData("shooter/target rpm", 0.0);
            robot.telemetry.addData("shooter/current rpm", 0.0);
            robot.telemetry.addData("shooter/power", 0.0);
            return;
        }

        // Measure dt and velocity from position deltas for robust measurement
        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-3;

        int ticks = robot.flywheel.getCurrentPosition();
        int deltaTicks = ticks - lastTicks;
        double ticksPerSecond = deltaTicks / dt;
        currentRpm = ticksPerSecondToRpm(ticksPerSecond);

        // Acceleration estimate (rpm/s)
        double accelRpmPerSec = (currentRpm - lastRpm) / dt;

        // Feedforward (forward-only)
        double ff;
        if (targetRpm > 0.0) {
            ff = kS + kV * targetRpm + kA * accelRpmPerSec;
        } else {
            ff = 0.0;
        }

        // Feedback (P on velocity error)
        double error = targetRpm - currentRpm;
        double fb = kP * error;

        // Combine
        power = ff + fb;

        // Normalize to nominal voltage (12V) to compensate for battery sag
        double vbat = robot.getBatteryVoltage();
        if (vbat > 1e-3) {
            double scale = Common.NOMINAL_BATTERY_VOLTAGE / vbat;
            power *= scale;
        }

        // Clamp to [0, 1] forward-only
        if (power < 0.0) power = 0.0;
        if (power > 1.0) power = 1.0;

        robot.flywheel.setPower(power);

        // Telemetry
        robot.telemetry.addData("shooter/state", state);
        robot.telemetry.addData("shooter/target rpm", targetRpm);
        robot.telemetry.addData("shooter/current rpm", currentRpm);
        robot.telemetry.addData("shooter/error rpm", error);
        robot.telemetry.addData("shooter/power", power);

        // Save time history
        lastTime = now;
        lastTicks = ticks;
        lastRpm = currentRpm;
    }

    public void periodic() {
        updateHardware();
    }
}
