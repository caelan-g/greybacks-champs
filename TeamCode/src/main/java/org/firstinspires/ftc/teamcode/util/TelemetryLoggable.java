package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

public class TelemetryLoggable {

    private List<Double> dataBuffer;
    private int nextPosition = 0;
    private int bufferSize;

    public TelemetryLoggable(int bufferSize){
        dataBuffer = new ArrayList<>();
        for( int i = 0; i < bufferSize; i++){
            dataBuffer.add(i, 0d);
        }
        this.bufferSize = bufferSize;
    }

    public void addData(double value){
        dataBuffer.set(Math.abs(nextPosition % bufferSize), value);
        nextPosition = Math.abs(nextPosition + 1);
    }

    public double getAverage(){
        double total = 0;
        for(double value : dataBuffer){
            total += value;
        }
        return total / bufferSize;
    }

    public double getMax(){
        double bigNumber = dataBuffer.get(0);
        for(double value : dataBuffer){
            if(value > bigNumber) bigNumber = value;
        }
        return bigNumber;
    }

    public double getMin(){
        double smallNumber = dataBuffer.get(0);
        for(double value : dataBuffer){
            if(value < smallNumber) smallNumber = value;
        }
        return smallNumber;
    }

    public double getLast(){
        return dataBuffer.get((nextPosition - 1) % bufferSize);
    }
}
