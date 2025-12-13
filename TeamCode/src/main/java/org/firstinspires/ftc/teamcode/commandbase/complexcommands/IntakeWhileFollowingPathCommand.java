package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

/**
 * Follows a path while intaking balls and auto-switching spindexer slots.
 * Ends when EITHER the path finishes OR all 3 slots are full.
 * 
 * Uses ParallelRaceGroup to compose FollowPathCommand with AutoIntakeCommand.
 */
public class IntakeWhileFollowingPathCommand extends ParallelRaceGroup {
    public IntakeWhileFollowingPathCommand(Follower follower, PathChain path) {
        addCommands(
            new FollowPathCommand(follower, path),  // Ends when path done
            new AutoIntakeCommand()                 // Ends when 3 slots full
        );
    }
}
