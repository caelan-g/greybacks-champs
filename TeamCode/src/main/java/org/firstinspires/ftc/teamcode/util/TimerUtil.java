package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.util.NanoClock;

public class TimerUtil {

    static NanoClock clock = NanoClock.system();
    private double lastLoop;

    public TimerUtil(){
        lastLoop = clock.seconds();
    }

    public double msSinceLastCall() {
        double thisLoop = clock.seconds();
        double loopTime = (thisLoop - lastLoop) * 1000; //return in milliseconds
        lastLoop = thisLoop;
        return loopTime;
    }
}
