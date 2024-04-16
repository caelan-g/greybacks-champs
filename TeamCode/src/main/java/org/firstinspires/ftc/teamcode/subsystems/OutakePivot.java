package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class OutakePivot {

    private ServoImplEx outakeServo;

    private AnalogInput outakeSensor;
    private double outakeServoOffset = 0;
    private double outakeLastTarget = 0;
    private double outakePosition = 0;
    private double outakeTargetPosition = 240;
    public OutakePivot(HardwareMap hardwareMap) {
        outakeServo = hardwareMap.get(ServoImplEx.class, "sCHub2");
        outakeServo.setPwmRange(new PwmControl.PwmRange(600, 2400));
        //outakeSensor = hardwareMap.get(AnalogInput.class, "eHubA0");
        writeOutakeServo(240);
        outakeServo.setPwmEnable();
    }

    public void update() {
        updatePosition();
        writeOutakeServo(outakeTargetPosition);
    }

    private void updatePosition() {
//        outakePosition = (outakeSensor.getVoltage() / 3.3) * 360 - outakeServoOffset;
    }

    public void setTargetPosition(double target) {
        outakeTargetPosition = target;
    }

    private void writeOutakeServo(double outakePosition) {
        outakeServo.setPosition((outakePosition + outakeServoOffset ) / 355d);
        outakeLastTarget = outakePosition;
    }

    public void setTargetPosition(Positions.OutakePivot setPosition) {
        setTargetPosition(setPosition.position);
    }

    public double getTargetPosition() {
        return outakeLastTarget;
    }

    public void disablePWM() {
        outakeServo.setPwmDisable();
    }


}
