package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;

public class ColorSensorSubsystem extends SubsystemBase {
    public enum ColorSensorState {
        OFF,
        READING_COLOR,
        READING_DISTANCE,
        READING_BOTH
    }

    private final RobotHardware robot;
    public volatile ColorSensorState state;

    // HSV color values (Hue: 0-360, Saturation: 0-1, Value: 0-1)
    private float hue = 0f;
    private float saturation = 0f;
    private float value = 0f;

    // Raw RGB values (for reference)
    private int red = 0;
    private int green = 0;
    private int blue = 0;
    private int alpha = 0;

    // Distance in inches
    private double distance = 0.0;

    // Reusable array for HSV conversion
    private final float[] hsvValues = new float[3];

    public ColorSensorSubsystem() {
        robot = RobotHardware.getInstance();
        setState(ColorSensorState.OFF);
    }

    public void setState(ColorSensorState newState) {
        state = newState;
    }

    public ColorSensorState getState() {
        return state;
    }

    // ---------- Getters for HSV values ----------
    public float getHue() {
        return hue;
    }

    public float getSaturation() {
        return saturation;
    }

    public float getValue() {
        return value;
    }

    /**
     * @return HSV values as array: [hue, saturation, value]
     */
    public float[] getHSV() {
        return new float[]{hue, saturation, value};
    }

    // ---------- Getters for RGB values ----------
    public int getRed() {
        return red;
    }

    public int getGreen() {
        return green;
    }

    public int getBlue() {
        return blue;
    }

    public int getAlpha() {
        return alpha;
    }

    // ---------- Getter for distance ----------
    public double getDistance() {
        return distance;
    }

    /**
     * Reads color data from the sensor and converts to HSV.
     */
    private void readColor() {
        red = robot.colorSensor.red();
        green = robot.colorSensor.green();
        blue = robot.colorSensor.blue();
        alpha = robot.colorSensor.alpha();

        // Convert RGB to HSV using Android's Color utility
        Color.RGBToHSV(red, green, blue, hsvValues);
        hue = hsvValues[0];         // 0-360 degrees
        saturation = hsvValues[1];  // 0-1
        value = hsvValues[2];       // 0-1
    }

    /**
     * Reads distance data from the sensor.
     */
    private void readDistance() {
        distance = robot.colorSensor.getDistance(DistanceUnit.INCHES);
    }

    public void updateHardware() {
        // No hardware to update for sensor (it's read-only)
    }

    @Override
    public void periodic() {
        switch (state) {
            case OFF:
                // Do nothing
                break;
            case READING_COLOR:
                readColor();
                break;
            case READING_DISTANCE:
                readDistance();
                break;
            case READING_BOTH:
                readColor();
                readDistance();
                break;
        }
    }
}
