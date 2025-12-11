package org.firstinspires.ftc.teamcode.commandbase.safecommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class SpindexerSetPositionCommand extends CommandBase {
    private final SpindexerSubsystem spindexerSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final SpindexerSubsystem.SpindexerState targetState;
    private long startTime;
    private long waitTime;
    private boolean isFinishedImmediately = false;

    // Wait times matrix [from][to]
    // Indices: 0=INTAKE_ONE, 1=INTAKE_TWO, 2=INTAKE_THREE, 3=OUTTAKE_ONE, 4=OUTTAKE_TWO, 5=OUTTAKE_THREE
    private static final long[][] WAIT_TIMES = {
            {0, 175, 350, 275, 525, 775},   // From INTAKE_ONE (1)
            {175, 0, 175, 100, 275, 525},   // From INTAKE_TWO (2)
            {350, 175, 0, 100, 100, 350},   // From INTAKE_THREE (3) - Assumed 3->2 is 250 based on symmetry/pattern
            {500, 125, 125, 0, 175, 350},   // From OUTTAKE_ONE (4)
            {700, 500, 125, 175, 0, 175},   // From OUTTAKE_TWO (5)
            {125, 125, 500, 350, 175, 0}    // From OUTTAKE_THREE (6)
    };

    public SpindexerSetPositionCommand(SpindexerSubsystem spindexerSubsystem, TransferSubsystem transferSubsystem, SpindexerSubsystem.SpindexerState state) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.targetState = state;
        addRequirements(spindexerSubsystem, transferSubsystem);
    }

    public SpindexerSetPositionCommand(SpindexerSubsystem.SpindexerState state) {
        this(RobotHardware.getInstance().spindexerSubsystem,
             RobotHardware.getInstance().transferSubsystem,
             state);
    }

    @Override
    public void initialize() {
        if (transferSubsystem.state != TransferSubsystem.TransferState.DOWN) {
            isFinishedImmediately = true;
            return;
        }

        isFinishedImmediately = false;
        SpindexerSubsystem.SpindexerState currentState = spindexerSubsystem.state;
        
        int fromIndex = getStateIndex(currentState);
        int toIndex = getStateIndex(targetState);

        if (fromIndex != -1 && toIndex != -1) {
            waitTime = WAIT_TIMES[fromIndex][toIndex];
        } else {
            waitTime = 750; // Default fallback
        }

        spindexerSubsystem.setSpindexerState(targetState);
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        if (isFinishedImmediately) return true;
        return (System.currentTimeMillis() - startTime) >= waitTime;
    }

    private int getStateIndex(SpindexerSubsystem.SpindexerState state) {
        switch (state) {
            case INTAKE_ONE: return 0;
            case INTAKE_TWO: return 1;
            case INTAKE_THREE: return 2;
            case OUTTAKE_ONE: return 3;
            case OUTTAKE_TWO: return 4;
            case OUTTAKE_THREE: return 5;
            default: return -1;
        }
    }
}
