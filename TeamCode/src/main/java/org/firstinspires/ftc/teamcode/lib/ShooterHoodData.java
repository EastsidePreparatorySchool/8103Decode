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
        lut.add(20, 0.47);
        lut.add(40, 0.47);
        lut.add(60, 0.49);
        lut.add(80, 0.51);
        lut.add(100, 0.52);
        lut.add(120, 0.53);
        lut.add(140, 0.54);
        lut.add(160, 0.54);
        lut.createLUT();
        return lut;
    }
}
