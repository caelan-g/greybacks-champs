package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.StateManager;
import org.firstinspires.ftc.teamcode.vision.RedPositionDetection;

// vector2d not imported?
@Config
@Autonomous(group = "drive")
public class redStage extends LinearOpMode {
    private int propPos = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        double ROBOT_LENGTH = 17.75; // in inches, 450.85 in mm
        double ROBOT_WIDTH = 16.75; // in inches, 425.45 in mm
        Pose2d STARTING_LOCATION = new Pose2d(13,ROBOT_LENGTH / 2 - 72, Math.toRadians(270));
        Pose2d BACKBOARD_LEFT = new Pose2d(60 - ROBOT_LENGTH / 2,-29, Math.toRadians(180));
        Pose2d BACKBOARD_MID = new Pose2d(60 - ROBOT_LENGTH / 2, -36, Math.toRadians(180));
        Pose2d BACKBOARD_RIGHT = new Pose2d(60 - ROBOT_LENGTH / 2, -43, Math.toRadians(180));
        Pose2d MID_PROP_POSITION = new Pose2d(25, -26, Math.toRadians(185));
        Pose2d RIGHT_PROP_POSITION = new Pose2d(32.5, -36, Math.toRadians(185));
        Pose2d LEFT_PROP_POSITION = new Pose2d(15, -36, Math.toRadians(185));
        Pose2d OTHER_LEFT_PROP_POSITION = new Pose2d(9, -36, Math.toRadians(185));

        Robot robot = new Robot(hardwareMap, telemetry);
        RedPositionDetection posDetect = new RedPositionDetection(hardwareMap);

        robot.drive.setPoseEstimate(STARTING_LOCATION);

        Trajectory toSpikeMark[] = {
            robot.drive.trajectoryBuilder(STARTING_LOCATION, true)
                    .splineToSplineHeading(LEFT_PROP_POSITION, LEFT_PROP_POSITION.getHeading())
                    .splineToSplineHeading(OTHER_LEFT_PROP_POSITION, OTHER_LEFT_PROP_POSITION.getHeading())
                    .build(),
            robot.drive.trajectoryBuilder(STARTING_LOCATION)
                    .lineToSplineHeading(MID_PROP_POSITION)
                    .build(),
            robot.drive.trajectoryBuilder(STARTING_LOCATION)
                    .lineToSplineHeading(RIGHT_PROP_POSITION)
                    .build()
        };

        Trajectory leftForward = robot.drive.trajectoryBuilder(toSpikeMark[0].end())
                .forward(5)
                .build();

        Trajectory toBackboard[] = {
                robot.drive.trajectoryBuilder(toSpikeMark[0].end())
                        .lineToSplineHeading(BACKBOARD_LEFT)
                        .build(),
                robot.drive.trajectoryBuilder(toSpikeMark[1].end())
                        .lineToSplineHeading(BACKBOARD_MID)
                        .build(),
                robot.drive.trajectoryBuilder(toSpikeMark[2].end())
                        .lineToSplineHeading(BACKBOARD_RIGHT)
                        .build()
        };

        Trajectory midBack = robot.drive.trajectoryBuilder(toBackboard[1].end())
                .forward(1.5)
                .build();

        Trajectory back[] = {
                robot.drive.trajectoryBuilder(toBackboard[0].end())
                        .forward(5)
                        .build(),
                robot.drive.trajectoryBuilder(midBack.end())
                        .forward(5)
                        .build(),
                robot.drive.trajectoryBuilder(toBackboard[2].end())
                        .forward(5)
                        .build()
        };

        Trajectory park[] = {
            robot.drive.trajectoryBuilder(back[0].end())
                    .strafeLeft(40)
                    .build(),
            robot.drive.trajectoryBuilder(back[1].end())
                    .strafeLeft(40)
                    .build(),
            robot.drive.trajectoryBuilder(back[2].end())
                    .strafeLeft(40)
                    .build()
        };

        while(opModeInInit()){
            robot.intake.setServoPower(0);
            robot.outake.setServoPower(0);
            posDetect.updateDetector();
            telemetry.addData("Position Number: ", posDetect.getPropPosition());
            telemetry.addData("position: ", posDetect.getPropExactPosition());
            telemetry.addData("x", robot.drive.getPoseEstimate().getX());
            telemetry.addData("y", robot.drive.getPoseEstimate().getY());
            telemetry.update();
            propPos = posDetect.getPropPosition();

        }
        propPos--;

        if (isStopRequested()) return;


        robot.stateManager.setRobotState(StateManager.RobotState.INTAKING);
        robot.executeTrajectory(toSpikeMark[propPos]);
//        if (propPos == 0) robot.drive.followTrajectory(leftForward);

        robot.stateManager.setIntakePower(0.5);
        robot.blockingTimer(robot.createTimerReference(1));
        robot.stateManager.setIntakePower(0);

        robot.executeTrajectory(toBackboard[propPos]);
        if (propPos == 1) robot.executeTrajectory(midBack);

        robot.stateManager.setScoringHeight(400);
        robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
        robot.blockingTimer(robot.createTimerReference(2));
        robot.stateManager.setIsOutaking(true);
        robot.blockingTimer(robot.createTimerReference(2));
        robot.stateManager.setIsOutaking(false);
        robot.stateManager.setRobotState(StateManager.RobotState.INTAKING);
        robot.blockingTimer(robot.createTimerReference(2));

        robot.executeTrajectory(back[propPos]);
        robot.executeTrajectory(park[propPos]);
    }
}
