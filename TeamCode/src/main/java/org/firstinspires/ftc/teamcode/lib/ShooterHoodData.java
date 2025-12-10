package org.firstinspires.ftc.teamcode.lib;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterHoodData {

    public static InterpLUT getShooterInterpLUT() {
        InterpLUT lut = new InterpLUT();
        // Add data points here: lut.add(distance, rpm);
        lut.add(0, 3500);
        lut.add(40, 3500);
        lut.add(50, 3700);
        lut.add(60, 3900);
        lut.add(70, 4100);
        lut.add(80, 4300);
        lut.add(90, 4400);
        lut.add(100, 4500);
        lut.add(110, 4700);
        lut.add(120, 4800);
        lut.add(130, 4800);
        lut.add(140, 4950);
        lut.add(150, 5050);
        lut.add(160, 5200);
        lut.add(500, 5300);
        lut.createLUT();
        return lut;
    }

    public static InterpLUT getHoodInterpLUT() {
        InterpLUT lut = new InterpLUT();
        // Add data points here: lut.add(distance, hoodPos);
        lut.add(0, Common.HOOD_INITIAL_POS + (0.7 - 0.7));
        lut.add(40, Common.HOOD_INITIAL_POS + (0.66 - 0.7));
        lut.add(50, Common.HOOD_INITIAL_POS + (0.56 - 0.7));
        lut.add(60, Common.HOOD_INITIAL_POS + (0.40 - 0.7));
        lut.add(70, Common.HOOD_INITIAL_POS + (0.30 - 0.7));
        lut.add(80, Common.HOOD_INITIAL_POS + (0.17 - 0.7));
        lut.add(90, Common.HOOD_INITIAL_POS + (0.16 - 0.7));
        lut.add(100, Common.HOOD_INITIAL_POS + (0.15 - 0.7));
        lut.add(110, Common.HOOD_INITIAL_POS + (0.15 - 0.7));
        lut.add(120, Common.HOOD_INITIAL_POS + (0.15 - 0.7));
        lut.add(130, Common.HOOD_INITIAL_POS + (0.14 - 0.7));
        lut.add(140, Common.HOOD_INITIAL_POS + (0.14 - 0.7));
        lut.add(150, Common.HOOD_INITIAL_POS + (0.14 - 0.7));
        lut.add(160, Common.HOOD_INITIAL_POS + (0.14 - 0.7));
        lut.add(500, Common.HOOD_INITIAL_POS + (0.14 - 0.7));
        lut.createLUT();
        return lut;
    }
}
