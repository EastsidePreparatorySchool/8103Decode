package org.firstinspires.ftc.teamcode.lib;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterHoodData {

    public static InterpLUT getShooterInterpLUT() {
        InterpLUT lut = new InterpLUT();
        // Add data points here: lut.add(distance, rpm);
        lut.add(20, 4000);
        lut.add(40, 4000);
        lut.add(60, 4100);
        lut.add(80, 4400);
        lut.add(100, 4700);
        lut.add(120, 5100);
        lut.add(140, 5500);
        lut.add(160, 5600);
        lut.createLUT();
        return lut;
    }

    public static InterpLUT getHoodInterpLUT() {
        InterpLUT lut = new InterpLUT();
        // Add data points here: lut.add(distance, hoodPos);
        // Example placeholders:
        lut.add(20, 0);
        lut.add(40, 0);
        lut.add(60, 0.02);
        lut.add(80, 0.04);
        lut.add(100, 0.05);
        lut.add(120, 0.06);
        lut.add(140, 0.075);
        lut.add(160, 0.08);
        lut.createLUT();
        return lut;
    }
}
