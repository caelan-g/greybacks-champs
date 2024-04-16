package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class HorizontalExtension {


    public static PIDCoefficients EXTENSION_PID_COEFFICIENTS = new PIDCoefficients(0.018, 0, 0.014);
    double kV = 0;
    double kA = 0;
    public static double kStatic = 0.0;
    DigitalChannel extensionLimitSwitch0;
    DigitalChannel extensionLimitSwitch1;
    final double encoderTicksPerRevolution = 4096;
    final double spindleDiameter = 45.84; //in mm
    private double extensionHomingOffset = 0;
    public boolean isHomed = false;
    double extensionTolerance = 10;
    double extensionPosition = 0;
    double extensionPositionLast = 0;
    double extensionVelocity = 0;
    double extensionVelocityLast = 0;
    double extensionAcceleration = 0;
    double timeSinceLastRun = 1;
    double motorPower = 0;
    boolean isManual = false;
    boolean requireHomed = false;
    double encoderRatio = 1;
    public static double val = 0.045;
    private double intakeHeightTarget = 0;

    private final double motorWriteThreshold = 0.007;
    private double lastMotorPower = 0;

    private double extensionPositionOffset = 0;

    public double getExtensionPositionOffset() {
        return extensionPositionOffset;
    }

    public void setExtensionPositionOffset(double extensionPositionOffset) {
        this.extensionPositionOffset = extensionPositionOffset;
    }


    public enum Position {
        TOP_POLE(760),
        MIDDLE_POLE(530),
        LOW_POLE(300),
        OFFSET(57),
        BOTTOM(0);

        double position;

        Position(double position) {
            this.position = position;
        }
    }

    static double kFeedforward(double measuredPosition, double measuredVelocity) {
        //this is added to the pid controller output
        //TODO add look up table of static friciton for given heights

        return val+val*measuredVelocity*0.;
    }

    NanoClock clock = NanoClock.system();
    double lastRunTime = 0;
    boolean lastRunTimeDefined = false;

    private final PIDFController extensionController = new PIDFController(EXTENSION_PID_COEFFICIENTS, kV, kA, kStatic, HorizontalExtension::kFeedforward);

    private final DcMotorEx extension0;
    private final DcMotorEx extension1;
    private final List<DcMotorEx> motors;
    private final Encoder extensionEncoder;

    public HorizontalExtension( HardwareMap hardwareMap){

        extensionLimitSwitch0 = hardwareMap.get(DigitalChannel.class, "extensionLimitSwitch0");
        extensionLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "extensionLimitSwitch1");
        extensionLimitSwitch0.setMode(DigitalChannel.Mode.INPUT);
        extensionLimitSwitch1.setMode(DigitalChannel.Mode.INPUT);

        extension0 = hardwareMap.get(DcMotorEx.class, "extension0");
        extension1 = hardwareMap.get(DcMotorEx.class, "extension1");

        extensionEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "extension2"));

        motors = Arrays.asList(extension0, extension1);

        for(DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void update(){
        updateTargetPosition();
        updateTimeSinceLastRun();
        updatePosition();
        motorPower = extensionController.update(extensionPosition, extensionVelocity);

        if(extensionController.getTargetPosition() == 0 && extensionPosition == 0) motorPower = 0;

        if(Math.abs(motorPower-lastMotorPower) > motorWriteThreshold) {
            extension0.setPower(motorPower);
            extension1.setPower(motorPower);
            lastMotorPower = motorPower;
        }
    }
    private void updateTargetPosition() {
        setControllerPosition(intakeHeightTarget + extensionPositionOffset);
    }
    public void setTargetPosition(double targetPosition){
        intakeHeightTarget = targetPosition;
    }
    private void setControllerPosition(double targetPosition) {
        if (requireHomed && !isHomed && targetPosition < 5) {
            extensionController.setTargetPosition(getCurrentPosition() - 20);
        }
        else {
            extensionController.setTargetPosition(targetPosition);
        }
    }

    public double getRequestedIntakeHeight(){
        return intakeHeightTarget;
    }

    public double getTargetPosition() {
        return extensionController.getTargetPosition();
    }

    public void setTargetPosition(Position position) {
        setTargetPosition(position.position);
    }

    public boolean isAtSetpoint() {
        return isHomed && Math.abs(extensionController.getLastError()) < extensionTolerance;
    }
    public double getCurrentPosition() {
        return extensionPosition;
    }

    public double getCurrentVelocity() {
        return extensionVelocity;
    }

    public double getCurrentAcceleration() {
        return extensionAcceleration;
    }

    public double getCurrentMotorPower() {
        return motorPower;
    }

    private void updatePosition() {
        extensionPositionLast = extensionPosition;
        if(extensionHomeSwitch()) {
            extensionHomingOffset = fetchEncoderTicksinMilimetres();
            isHomed = true;
            extensionPosition = 0;
            if(extensionController.getTargetPosition()<0) extensionController.setTargetPosition(0);
        }
        else {
            extensionPosition = fetchEncoderTicksinMilimetres() - extensionHomingOffset;
            if(getTargetPosition()<0.01 & getCurrentPosition()<15){ extensionHomingOffset -= 1; }
        }

        extensionVelocityLast = extensionVelocity;
        extensionVelocity = (extensionPosition - extensionPositionLast) / timeSinceLastRun;

        extensionAcceleration = (extensionVelocity - extensionVelocityLast) / timeSinceLastRun;
    }

    private double fetchEncoderTicksinMilimetres() {
        return ((extensionEncoder.getCurrentPosition() / encoderTicksPerRevolution) / encoderRatio) * spindleDiameter * Math.PI ;
    }

    private boolean extensionHomeSwitch() {
        return extensionLimitSwitch0.getState()|extensionLimitSwitch1.getState();
    }



    private void updateTimeSinceLastRun() {
        if(lastRunTimeDefined) {
            double currentTime = clock.seconds();
            timeSinceLastRun = currentTime - lastRunTime;
            lastRunTime = currentTime;
        }
        else {
            timeSinceLastRun = 1;
        }
    }

    public void updateSensorsOnly() {
        updatePosition();
    }

    public void setHomeRequiredPolicy(boolean policy){
        requireHomed = policy;
    }

}
