package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {


    private CRServoImplEx intakeServo;
    private double requestedPower;
    private double lastPower;
    private static final double INTAKE_POWER_WRITE_THRESHOLD = 0.02;


    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServoImplEx.class, "sCHub0");
        intakeServo.setPwmRange(new PwmControl.PwmRange(600, 2400));
    }

    public void update() {
        if(Math.abs(requestedPower - lastPower) > INTAKE_POWER_WRITE_THRESHOLD){
            intakeServo.setPower(requestedPower);
            lastPower = requestedPower;
        }
    }

    public void setServoPower(double servoPower) {
        requestedPower = servoPower;
    }
    public void setServoPower(Positions.Intake setPosition) {
        setServoPower(setPosition.power);
    }

    public void disableServos(){
        intakeServo.setPower(0);
    }
}
