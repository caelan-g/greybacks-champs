package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outake;
import org.firstinspires.ftc.teamcode.subsystems.OutakePivot;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.VerticalElevator;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TelemetryLoggable;
import org.firstinspires.ftc.teamcode.util.TimerUtil;

import java.util.ArrayList;
import java.util.List;

public class Robot {



    public SampleMecanumDrive drive;
    public MultipleTelemetry telemetry;
    public VerticalElevator verticalElevator;
    public Intake intake;
    public Outake outake;
    public OutakePivot outakePivot;
    public PlaneLauncher planeLauncher;
    public StateManager stateManager;

    NanoClock clock = NanoClock.system();
    HardwareMap hardwareMap;
    private List<LynxModule> connectedHubs;

    TimerUtil driveMonitor = new TimerUtil();
    TimerUtil robotMonitor = new TimerUtil();
    private TelemetryLoggable drivetrainLoopTime = new TelemetryLoggable(50);
    private TelemetryLoggable robotLoopTime = new TelemetryLoggable(50);

    private int autoRandomisation = 1;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        verticalElevator = new VerticalElevator(hardwareMap);
        intake = new Intake(hardwareMap);
        outake = new Outake(hardwareMap);
        outakePivot = new OutakePivot(hardwareMap);
        planeLauncher = new PlaneLauncher(hardwareMap);

        List<LynxModule> connectedHubs = hardwareMap.getAll(LynxModule.class);

        if(connectedHubs.get(0).isParent()){
            this.connectedHubs = connectedHubs;
        } else {
            this.connectedHubs = new ArrayList<>();
            this.connectedHubs.add(0, connectedHubs.get(1));
            this.connectedHubs.add(1, connectedHubs.get(0));
        }

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        stateManager = new StateManager(intake,outake,outakePivot,verticalElevator);
    }
    public void update(){
        refreshBulkCache(0);
        drive.update();
        drivetrainLoopTime.addData(driveMonitor.msSinceLastCall());

        refreshBulkCache(1);
        stateManager.update();
        verticalElevator.update();
        outakePivot.update();

        refreshBulkCache(0);
        drive.update();
        drivetrainLoopTime.addData(driveMonitor.msSinceLastCall());

        intake.update();
        outake.update();
        robotLoopTime.addData(robotMonitor.msSinceLastCall());

        refreshBulkCache(0);
        drive.update();
        drivetrainLoopTime.addData(driveMonitor.msSinceLastCall());

        updateTelemetry();
    }

    public void updateRobotWhileDriveBusy() {
        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            update();
        }
    }

    public void executeTrajectory(Trajectory trajectory){
        drive.followTrajectoryAsync(trajectory);
        updateRobotWhileDriveBusy();
    }

    public void executeTrajectorySequence(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
        updateRobotWhileDriveBusy();
    }

    public double createTimerReference(double timerLength){
        return timerLength + clock.seconds();
    }

    public boolean isAsyncTimerFinished(double timerReference) { return timerReference > clock.seconds();}

    public void blockingTimer(double timerReference){
        updateRobotUntilTimer(timerReference);
    }

    public void updateTelemetry() {

        telemetry.addData("elevator pos: ", verticalElevator.elevatorPosition);
        telemetry.addData("elevator current", verticalElevator.getMotorCurrent());
        telemetry.addData("robotX", drive.getPoseEstimate().getX());
        telemetry.addData("robotY", drive.getPoseEstimate().getY());
        telemetry.addData("outake pivot target", outakePivot.getTargetPosition());

        telemetry.addData("elevatorMotorPower", verticalElevator.getMotorPower());
        telemetry.addData("Elevator Current Position", verticalElevator.elevatorPosition);
        telemetry.addData("Elevator Controller Target", verticalElevator.getControllerTargetPosition());
        telemetry.addData("Elevator Target Position", verticalElevator.targetPosition);
        telemetry.addData("Home Switches Pressed: ", verticalElevator.queryHomeSwitchHardware());

        telemetry.addData("Swerve Loop Average", drivetrainLoopTime.getAverage());
        telemetry.addData("Swerve Loop Max", drivetrainLoopTime.getMax());
        telemetry.addData("Robot Loop Average", robotLoopTime.getAverage());

        telemetry.update();
    }

    public void updateRobotUntilTimer(double timerReference) {
        while(!Thread.currentThread().isInterrupted() && isAsyncTimerFinished(timerReference)) {
            update();
        }
    }

    public int getAutoRandomisation() {
        return autoRandomisation;
    }

    public void setAutoRandomisation(int autoRandomisation) {
        this.autoRandomisation = autoRandomisation;
    }

    private void refreshBulkCache(int moduleNumber){
        if(connectedHubs.size()>=moduleNumber) connectedHubs.get(moduleNumber).clearBulkCache();
    }

    private void refreshBulkCache(){
        for (LynxModule module : connectedHubs) {
            module.clearBulkCache();
        }
    }

    public void stopServos(){
        intake.disableServos();
        outake.disableServos();
    }

}
