package org.firstinspires.ftc.teamcode.lib;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterHoodData {

    public static InterpLUT getShooterInterpLUT() {
        InterpLUT lut = new InterpLUT();
        // Add data points here: lut.add(distance, rpm);
        lut.add(74.61, 4000);
        lut.add(81.5, 4300);
        lut.add(110, 4600);
        lut.add(120, 4800);
        lut.add(131.6, 5000);
        lut.add(141.0, 5400);
        lut.add(150, 5700);
        lut.createLUT();
        return lut;
    }

    public static InterpLUT getHoodInterpLUT() {
        InterpLUT lut = new InterpLUT();
        // Add data points here: lut.add(distance, hoodPos);
        // Example placeholders:
        lut.add(74.61, 0.48);
        lut.add(81.5, 0.48);
        lut.add(110, 0.51);
        lut.add(120, 0.52);
        lut.add(131.6, 0.54);
        lut.add(141.0, 0.55);
        lut.add(150, 0.56);
        lut.createLUT();
        return lut;
    }
}
