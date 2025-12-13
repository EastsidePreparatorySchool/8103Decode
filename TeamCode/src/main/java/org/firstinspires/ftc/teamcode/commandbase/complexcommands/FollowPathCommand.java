package org.firstinspires.ftc.teamcode.commandbase.complexcommands;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

/**
 * Custom FollowPathCommand that uses follower.isAtParametricEnd() as the end condition
 * instead of !follower.isBusy(). This provides more precise path completion detection.
 * 
 * Based on SolversLib FollowPathCommand by Arush & Saket - FTC 23511
 */
public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain pathChain;
    private boolean holdEnd;
    private double maxPower = 1.0;

    public FollowPathCommand(Follower follower, PathChain pathChain) {
        this(follower, pathChain, true);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd) {
        this(follower, pathChain, holdEnd, 1.0);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, double maxPower) {
        this(follower, pathChain, true, maxPower);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowPathCommand(Follower follower, Path pathChain) {
        this(follower, pathChain, true);
    }

    public FollowPathCommand(Follower follower, Path pathChain, boolean holdEnd) {
        this(follower, pathChain, holdEnd, 1.0);
    }

    public FollowPathCommand(Follower follower, Path pathChain, double maxPower) {
        this(follower, pathChain, true, maxPower);
    }

    public FollowPathCommand(Follower follower, Path pathChain, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.pathChain = new PathChain(pathChain);
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    /**
     * Sets Global Maximum Power for Follower, and overwrites maxPower in constructor
     *
     * @param globalMaxPower The new globalMaxPower
     * @return This command for compatibility in command groups
     */
    public FollowPathCommand setGlobalMaxPower(double globalMaxPower) {
        follower.setMaxPower(globalMaxPower);
        maxPower = globalMaxPower;
        return this;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain, maxPower, holdEnd);
    }

    @Override
    public boolean isFinished() {
        // Use isAtParametricEnd() for more precise path completion detection
        return follower.atParametricEnd();
    }
}
