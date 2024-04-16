package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.StateManager;
import org.firstinspires.ftc.teamcode.vision.BluePositionDetection;

// vector2d not imported?
@Config
@Autonomous(group = "drive")
public class blueStage extends LinearOpMode {
    private int propPos = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        double ROBOT_LENGTH = 17.75; // in inches, 450.85 in mm
        double ROBOT_WIDTH = 16.75; // in inches, 425.45 in mm
        Pose2d STARTING_LOCATION = new Pose2d(13,72 - ROBOT_LENGTH / 2, Math.toRadians(90));
        Pose2d BACKBOARD_RIGHT = new Pose2d(60 - ROBOT_LENGTH / 2,28, Math.toRadians(185));
        Pose2d BACKBOARD_MID = new Pose2d(60 - ROBOT_LENGTH / 2, 34, Math.toRadians(185));
        Pose2d BACKBOARD_LEFT = new Pose2d(60 - ROBOT_LENGTH / 2, 44, Math.toRadians(180));
        Pose2d MID_PROP_POSITION = new Pose2d(25, 25, Math.toRadians(175));
        Pose2d LEFT_PROP_POSITION = new Pose2d(33, 41, Math.toRadians(175));
        Pose2d RIGHT_PROP_POSITION = new Pose2d(15, 36, Math.toRadians(175));
        Pose2d OTHER_RIGHT_PROP_POSITION = new Pose2d(9, 36, Math.toRadians(175));

        Robot robot = new Robot(hardwareMap, telemetry);
        BluePositionDetection posDetect = new BluePositionDetection(hardwareMap);

        robot.drive.setPoseEstimate(STARTING_LOCATION);

        Trajectory toSpikeMark[] = {
                robot.drive.trajectoryBuilder(STARTING_LOCATION)
                        .lineToSplineHeading(LEFT_PROP_POSITION)
                        .build(),
                robot.drive.trajectoryBuilder(STARTING_LOCATION)
                        .lineToSplineHeading(MID_PROP_POSITION)
                        .build(),
                robot.drive.trajectoryBuilder(STARTING_LOCATION, true)
                        .splineToSplineHeading(RIGHT_PROP_POSITION, RIGHT_PROP_POSITION.getHeading())
                        .splineToSplineHeading(OTHER_RIGHT_PROP_POSITION, OTHER_RIGHT_PROP_POSITION.getHeading())
                        .build()
        };

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
                        .strafeRight(40)
                        .build(),
                robot.drive.trajectoryBuilder(back[1].end())
                        .strafeRight(40)
                        .build(),
                robot.drive.trajectoryBuilder(back[2].end())
                        .strafeRight(40)
                        .build()
        };

        while(opModeInInit()){
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
//        if (propPos == 2) robot.drive.followTrajectory(forwardToProp);

        robot.stateManager.setIntakePower(0.5);
        robot.blockingTimer(robot.createTimerReference(2));
        robot.stateManager.setIntakePower(0);

        robot.executeTrajectory(toBackboard[propPos]);
        if (propPos == 1) robot.executeTrajectory(midBack);



        //if (propPos == 1) robot.stateManager.setScoringHeight(300);
        robot.stateManager.setScoringHeight(500);
        robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
        robot.blockingTimer(robot.createTimerReference(4));
        robot.stateManager.setIsOutaking(true);
        robot.blockingTimer(robot.createTimerReference(2));
        robot.stateManager.setIsOutaking(false);
        robot.stateManager.setRobotState(StateManager.RobotState.INTAKING);
        robot.blockingTimer(robot.createTimerReference(2));

        robot.executeTrajectory(back[propPos]);
        robot.executeTrajectory(park[propPos]);
    }
}
