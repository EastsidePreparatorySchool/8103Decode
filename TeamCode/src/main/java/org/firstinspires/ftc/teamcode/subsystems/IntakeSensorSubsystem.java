package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;

/**
 * Subsystem for intake sensors:
 * - Brushland Labs proximity sensor (digital mode on digital0)
 * - REV V3 Color Sensor (read only on rising edge of proximity sensor)
 */
public class IntakeSensorSubsystem extends SubsystemBase {

    private RobotHardware robot;
    
    // State tracking for rising edge detection
    private boolean previousProximityState = false;
    
    // Public variables as requested
    /** True if an object is currently detected in front of the proximity sensor */
    public boolean objectDetected = false;
    
    /** True on the loop when a NEW object is detected (rising edge) */
    public boolean risingEdge = false;
    
    /** The most recently read color from the color sensor (updated on rising edge only) */
    public NormalizedRGBA detectedColor = null;
    
    /** HSV values of the most recently read color [hue, saturation, value] */
    public float[] detectedColorHSV = new float[3];

    public IntakeSensorSubsystem() {
        robot = RobotHardware.getInstance();
        
        // Turn on the color sensor light if possible
        if (robot.intakeColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.intakeColorSensor).enableLight(true);
        }
    }

    /**
     * Called every loop. Updates proximity sensor state and detects rising edges.
     * Color sensor is only read on rising edge of proximity sensor.
     */
    @Override
    public void periodic() {
        // Read current proximity sensor state
        // Note: Digital sensors typically return false when object is detected
        // (active low), so we invert the reading
        boolean currentProximityState = robot.intakeProximitySensor.getState();
        
        // Update public object detected state
        objectDetected = currentProximityState;
        
        // Detect rising edge (object newly detected)
        risingEdge = currentProximityState && !previousProximityState;
        
        // Read color sensor only on rising edge
        if (risingEdge) {
            detectedColor = robot.intakeColorSensor.getNormalizedColors();
            // Convert to HSV for easier color identification
            Color.colorToHSV(detectedColor.toColor(), detectedColorHSV);
        }
        
        // Save current state for next loop's edge detection
        previousProximityState = currentProximityState;
    }
    
    /**
     * Gets the hue component of the last detected color (0-360 degrees).
     * Useful for color classification.
     * @return Hue value, or -1 if no color has been read yet
     */
    public float getDetectedHue() {
        if (detectedColor == null) {
            return -1;
        }
        return detectedColorHSV[0];
    }
    
    /**
     * Gets the saturation component of the last detected color (0-1).
     * @return Saturation value, or -1 if no color has been read yet
     */
    public float getDetectedSaturation() {
        if (detectedColor == null) {
            return -1;
        }
        return detectedColorHSV[1];
    }
    
    /**
     * Gets the value (brightness) component of the last detected color (0-1).
     * @return Value, or -1 if no color has been read yet
     */
    public float getDetectedValue() {
        if (detectedColor == null) {
            return -1;
        }
        return detectedColorHSV[2];
    }
    
    /**
     * Forces a color sensor read regardless of proximity sensor state.
     * Useful for calibration or testing.
     */
    public void forceColorRead() {
        detectedColor = robot.intakeColorSensor.getNormalizedColors();
        Color.colorToHSV(detectedColor.toColor(), detectedColorHSV);
    }
}
