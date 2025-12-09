package org.firstinspires.ftc.teamcode.lib;

/**
 * A simple utility class to enforce a target loop rate in an OpMode.
 * 
 * Usage:
 *   LoopRateLimiter limiter = new LoopRateLimiter(50); // 50 Hz target
 *   
 *   while (opModeIsActive()) {
 *       // ... your loop code ...
 *       limiter.waitForNextLoop();
 *   }
 */
public class LoopRateLimiter {
    private final long targetLoopTimeNs;
    private long loopStartTimeNs;

    /**
     * Creates a LoopRateLimiter with the specified target frequency.
     * @param targetHz The desired loop frequency in Hz
     */
    public LoopRateLimiter(double targetHz) {
        this.targetLoopTimeNs = (long) (1_000_000_000.0 / targetHz);
        this.loopStartTimeNs = System.nanoTime();
    }

    /**
     * Call this at the end of each loop iteration.
     * It will sleep if necessary to maintain the target loop rate.
     * Also marks the start of the next loop.
     */
    public void waitForNextLoop() {
        long elapsed = System.nanoTime() - loopStartTimeNs;
        long sleepTimeNs = targetLoopTimeNs - elapsed;

        if (sleepTimeNs > 0) {
            try {
                long sleepMs = sleepTimeNs / 1_000_000;
                int sleepNs = (int) (sleepTimeNs % 1_000_000);
                Thread.sleep(sleepMs, sleepNs);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Mark the start of the next loop
        loopStartTimeNs = System.nanoTime();
    }

    /**
     * Reset the loop timer. Call this before entering the main loop
     * if there's significant setup time between construction and first loop.
     */
    public void reset() {
        loopStartTimeNs = System.nanoTime();
    }

    /**
     * Returns the actual time taken for the last loop in milliseconds,
     * useful for debugging/telemetry.
     */
    public double getActualLoopTimeMs() {
        return (System.nanoTime() - loopStartTimeNs) / 1_000_000.0;
    }

    /**
     * Returns the target loop time in milliseconds.
     */
    public double getTargetLoopTimeMs() {
        return targetLoopTimeNs / 1_000_000.0;
    }
}
