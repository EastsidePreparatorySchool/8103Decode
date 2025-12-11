package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Autonomous(name = "ColorTest", group = "Test")
public class ColorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        NormalizedColorSensor colorSensor = hardwareMap.get(ColorRangeSensor.class, "color");
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        
        waitForStart();

        while (opModeIsActive()) {
            // Read RGB values
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            int red = (int) (colors.red * 255);
            int green = (int) (colors.green * 255);
            int blue = (int) (colors.blue * 255);
            int alpha = (int) (colors.alpha * 255);
            
            // Convert RGB to HSV
            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);
            
            // Output RGB
            multipleTelemetry.addData("Red", "%.3f", colors.red);
            multipleTelemetry.addData("Green", "%.3f", colors.green);
            multipleTelemetry.addData("Blue", "%.3f", colors.blue);
            multipleTelemetry.addData("Alpha", "%.3f", colors.alpha);
            
            // Output HSV
            multipleTelemetry.addData("Hue", hsv[0]);
            multipleTelemetry.addData("Saturation", hsv[1]);
            multipleTelemetry.addData("Value", hsv[2]);
            
            multipleTelemetry.update();
        }
    }
}
