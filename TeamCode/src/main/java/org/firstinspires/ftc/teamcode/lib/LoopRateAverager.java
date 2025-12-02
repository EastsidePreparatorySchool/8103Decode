package org.firstinspires.ftc.teamcode.lib;

/**
 * Computes a sliding-window average loop rate (Hz).
 * Use update() once per loop, then read getHz().
 */
public class LoopRateAverager {
    private final long[] timesNs;
    private final int windowSize;
    private int count = 0;
    private int index = 0;
    private double hz = 0.0;

    public LoopRateAverager(int windowSize) {
        this.windowSize = Math.max(2, windowSize);
        this.timesNs = new long[this.windowSize];
    }

    public LoopRateAverager() {
        this(50);
    }

    public void reset() {
        count = 0;
        index = 0;
        hz = 0.0;
    }

    public void update() {
        long now = System.nanoTime();
        timesNs[index] = now;
        index = (index + 1) % windowSize;
        if (count < windowSize) {
            count++;
        }
        if (count >= 2) {
            int oldest = (count == windowSize) ? index : 0;
            long tOld = timesNs[oldest];
            long tNew = timesNs[(index - 1 + windowSize) % windowSize];
            long dtNs = tNew - tOld;
            if (dtNs > 0) {
                int samples = (count == windowSize) ? (windowSize - 1) : (count - 1);
                double dtSec = dtNs / 1e9;
                hz = samples / dtSec;
            }
        }
    }

    public double getHz() {
        return hz;
    }
}

