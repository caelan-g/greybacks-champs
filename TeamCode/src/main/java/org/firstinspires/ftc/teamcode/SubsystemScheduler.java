package org.firstinspires.ftc.teamcode;

public class SubsystemScheduler {

    private double targetUpdateFrequency;
    private double nextUpdateTime = 0;

    public SubsystemScheduler(double targetUpdateMilliseconds){
        this.targetUpdateFrequency = targetUpdateMilliseconds/1000;
    }

    public boolean updateDesired(double thisUpdateTime) {
        if( nextUpdateTime > thisUpdateTime ) return false;

        nextUpdateTime = thisUpdateTime + targetUpdateFrequency;
        return true;
    }
    
    public double getNextUpdateTime(){
        return nextUpdateTime;
    }

}
