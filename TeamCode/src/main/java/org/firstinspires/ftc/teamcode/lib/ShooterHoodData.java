package org.firstinspires.ftc.teamcode.lib;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterHoodData {

    public static InterpLUT getShooterInterpLUT() {
        InterpLUT lut = new InterpLUT();
        // Add data points here: lut.add(distance, rpm);
        // Example placeholders:
        lut.add(0.0, 3000.0);
        lut.add(100.0, 4000.0);
        lut.createLUT();
        return lut;
    }

    public static InterpLUT getHoodInterpLUT() {
        InterpLUT lut = new InterpLUT();
        // Add data points here: lut.add(distance, hoodPos);
        // Example placeholders:
        lut.add(0.0, 0.5);
        lut.add(100.0, 0.6);
        lut.createLUT();
        return lut;
    }
}
