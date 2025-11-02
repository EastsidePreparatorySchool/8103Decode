package org.firstinspires.ftc.teamcode.autonomous;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.commandbase.safecommands.MecanumPowerMotorsCommand;
import org.firstinspires.ftc.teamcode.lib.Common;

@Autonomous(name = "RightThreePieceAuto", group = "Command")
public class    RightThreePieceAuto extends ThreePieceAuto {
    @Override
    public void initialize() {
        Common.SELECTED_FIELD_TARGET_X_IN = Common.RIGHT_FIELD_TARGET_X_IN;
        Common.SELECTED_FIELD_TARGET_Y_IN = Common.RIGHT_FIELD_TARGET_Y_IN;
        super.initialize();
    }
}
