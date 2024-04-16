package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;

@Config
public class VerticalElevator {

    //Hardware
    private final DcMotorEx elevator0;
    private final DcMotorEx elevator1;
    private final List<DcMotorEx> motors;
    private final Encoder elevatorEncoder;
    DigitalChannel elevatorLimitSwitch0;
    DigitalChannel elevatorLimitSwitch1;

    NanoClock clock = NanoClock.system();
    private final PIDFController elevatorController = new PIDFController(ELEVATOR_PID_COEFFICIENTS, kV, kA, kStatic);
    private MotionProfile activeProfile;
    private double profileStartTime = 0.0;

    private double encoderOffset = 0;
    private double powerCoefficient = 1;
    public double elevatorPosition = 0;
    private double elevatorVelocity = 0;
    public double targetPosition = 0;
    private double lastMotorPower = 0;
    private boolean isHomeSwitchPressed = false;

    public static PIDCoefficients ELEVATOR_PID_COEFFICIENTS = new PIDCoefficients(0.04, 0.0 , 0.0);
    public static double kV = 0.00225;
    public static double kA = 0.00024;
    public static double kStatic = 0.0;
    public static double kGravity = 0.1;
    private final double GEAR_RATIO = 15;
    private final double SPINDLE_DIAMETER = 45.84;
    private final double TICKS_PER_MM = 28.0 * GEAR_RATIO / (SPINDLE_DIAMETER * Math.PI);
    public static double VELOCITY_LIMIT = 1200.0;
    public static double ACCELERATION_LIMIT = 1500.0;
    public static double JERK_LIMIT = 3500;
    private static final double ELEVATOR_POSITION_TOLERANCE = 5.0;
    private static final double AUTO_HOME_INCREMENT = 0.4;
    private static final double ELEVATOR_MOTOR_WRITE_THRESHOLD = 0.05;



    public VerticalElevator(HardwareMap hardwareMap) {

        elevatorLimitSwitch0 = hardwareMap.get(DigitalChannel.class, "dEHub0");
        elevatorLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "dEHub2");
        elevatorLimitSwitch0.setMode(DigitalChannel.Mode.INPUT);
        elevatorLimitSwitch1.setMode(DigitalChannel.Mode.INPUT);

        elevator0 = hardwareMap.get(DcMotorEx.class, "eHub0");
        elevator1 = hardwareMap.get(DcMotorEx.class, "eHub1");
        elevatorEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "eHub0"));
        elevatorEncoder.setDirection(Encoder.Direction.REVERSE);

        motors = Arrays.asList(elevator0, elevator1);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void update() {

        updatePosition();

        if (!isProfileRelevant()) {
            updateMotionProfile();
        }

        if (activeProfile != null && isActiveProfileWithinDuration()) {
            MotionState state = generateMotionState();
            elevatorController.setTargetPosition(state.getX());
            elevatorController.setTargetVelocity(state.getV());
            elevatorController.setTargetAcceleration(state.getA());
        }
        else {
            elevatorController.setTargetPosition(targetPosition);
            elevatorController.setTargetVelocity(0.0);
            elevatorController.setTargetAcceleration(0.0);
        }

        double motorPower = elevatorController.update(elevatorPosition, elevatorVelocity) + calculateGravityFeedForward();
        setMotorPower(motorPower);



    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }
    public boolean queryHomeSwitchHardware() {
        isHomeSwitchPressed = !elevatorLimitSwitch0.getState() | !elevatorLimitSwitch1.getState();
        return isHomeSwitchPressed;
    }

    public boolean isHomeSwitchPressed(){
        return isHomeSwitchPressed;
    }

    private void updatePosition() {
        if (!queryHomeSwitchHardware()) {
            elevatorPosition = elevatorEncoder.getCurrentPosition() / TICKS_PER_MM - encoderOffset;
            if (targetPosition < 1) {
               encoderOffset -= AUTO_HOME_INCREMENT;
               encoderOffset = elevatorPosition < -10 ? encoderOffset = elevatorEncoder.getCurrentPosition() / TICKS_PER_MM - 10 : encoderOffset;
            }
        }
        else {

            elevatorPosition = 0;
            encoderOffset = elevatorEncoder.getCurrentPosition() / TICKS_PER_MM;
        }
        elevatorVelocity = elevatorEncoder.getRawVelocity() / TICKS_PER_MM;
    }

    private boolean isProfileRelevant() {
        if (activeProfile == null) return false;
        if (Math.abs(targetPosition - activeProfile.end().getX()) > ELEVATOR_POSITION_TOLERANCE) return false;
        return true;
    }

    private boolean isActiveProfileWithinDuration() {
        return  clock.seconds() < activeProfile.duration() + profileStartTime;
    }

    private void updateMotionProfile(){
        if (Math.abs(elevatorPosition - targetPosition) > ELEVATOR_POSITION_TOLERANCE) {
            try {
                activeProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(elevatorPosition, elevatorVelocity, 0),
                        new MotionState(targetPosition, 0, 0),
                        VELOCITY_LIMIT,
                        ACCELERATION_LIMIT,
                        JERK_LIMIT,
                        false
                );
                profileStartTime = clock.seconds();
            }
            catch(Exception e) {
                activeProfile = null;
            }
        }
        else {
            activeProfile = null;
        }
    }

    private MotionState generateMotionState() {
        return activeProfile.get(clock.seconds() - profileStartTime);
    }

    private void setMotorPower(double motorPower) {
        if (queryHomeSwitchHardware() && motorPower < 0) {
            motorPower = 0;
        }
        
        if(Math.abs(motorPower - lastMotorPower) > ELEVATOR_MOTOR_WRITE_THRESHOLD) {
            elevator0.setPower(motorPower*powerCoefficient);
            elevator1.setPower(motorPower*powerCoefficient);
            lastMotorPower = motorPower;
        }
    }

    private double calculateGravityFeedForward() {
        if (targetPosition < 1 && elevatorPosition < 1) return 0;
        return kGravity;
    }

    public double getMotorCurrent(){
        return elevator0.getCurrent(CurrentUnit.AMPS);
    }
    
    public double getMotorPower(){
        return lastMotorPower;
    }

    public double getControllerTargetPosition(){
        return elevatorController.getTargetPosition();
    }

    public double getHeight() {
        return elevatorPosition;
    }

    public void setPowerConstraint(double powerCoefficient) {
        this.powerCoefficient = powerCoefficient;
    }


}
