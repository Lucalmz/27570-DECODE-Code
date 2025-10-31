package org.firstinspires.ftc.teamcode.pedroPathing.Models;

import java.util.concurrent.atomic.AtomicInteger;

public class BallsInQueue {
    public final AtomicInteger PurpleBalls = new AtomicInteger(0);
    public final AtomicInteger GreenBalls = new AtomicInteger(0);
    private static BallsInQueue INSTANCE = null;

    private BallsInQueue(int purple, int green) {
        PurpleBalls.set(purple);
        GreenBalls.set(green);
    }

    public static synchronized BallsInQueue getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new BallsInQueue(0, 0);
        }
        return INSTANCE;
    }

    public synchronized void AddPurpleBall() {
        PurpleBalls.addAndGet(1);
    }

    public synchronized void AddGreenBall() {
        GreenBalls.addAndGet(1);
    }

    public synchronized void RemovePurpleBall() {
        PurpleBalls.addAndGet(-1);
    }

    public synchronized void RemoveGreenBall() {
        GreenBalls.addAndGet(-1);
    }

    public synchronized void ClearBalls() {
        PurpleBalls.set(0);
        GreenBalls .set(0);
    }
}
