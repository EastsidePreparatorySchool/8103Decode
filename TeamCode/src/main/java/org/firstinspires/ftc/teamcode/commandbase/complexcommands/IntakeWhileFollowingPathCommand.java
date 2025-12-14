package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

/**
 * Follows a path while intaking balls and auto-switching spindexer slots.
 * Ends when EITHER the path finishes (+ 500ms extra time) OR all 3 slots are full.
 * 
 * Uses ParallelRaceGroup to compose FollowPathCommand with AutoIntakeCommand.
 */
public class IntakeWhileFollowingPathCommand extends ParallelRaceGroup {
    public IntakeWhileFollowingPathCommand(Follower follower, PathChain path) {
        addCommands(
            new SequentialCommandGroup(
                new FollowPathCommand(follower, path),  // Ends when path done
                new WaitCommand(500)                     // Wait 500ms after path
            ),
            new AutoIntakeCommand()                      // Ends when 3 slots full
        );
    }
}
