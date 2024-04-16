package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.StateManager;
import org.firstinspires.ftc.teamcode.subsystems.Positions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.RedPositionDetection;

// vector2d not imported?
@Config
@Autonomous(group = "drive")
public class redAudience extends LinearOpMode {
    private int propPos = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        double ROBOT_LENGTH = 17.75; // in inches, 450.85 in mm
        double ROBOT_WIDTH = 16.75; // in inches, 425.45 in mm
        Pose2d STARTING_LOCATION = new Pose2d(ROBOT_WIDTH/2-48,-72 + ROBOT_LENGTH/2, Math.toRadians(270));
        Pose2d BACKBOARD_LEFT = new Pose2d(62 - ROBOT_LENGTH / 2,-28, Math.toRadians(180));
        Pose2d BACKBOARD_MID = new Pose2d(57 - ROBOT_LENGTH / 2, -50, Math.toRadians(174));
        Pose2d BACKBOARD_RIGHT = new Pose2d(62 - ROBOT_LENGTH / 2, -48, Math.toRadians(180));
        Pose2d MID_PROP_POSITION = new Pose2d(-52, -25, Math.toRadians(0));
        Pose2d RIGHT_PROP_POSITION = new Pose2d(-36, -39, Math.toRadians(0));
        Pose2d RIGHT_PRE_PROP_POSITION = new Pose2d(-45, -40, Math.toRadians(0));
        Pose2d LEFT_PROP_POSITION = new Pose2d(-38, -36, Math.toRadians(180));
        Pose2d LEFT_AUDIENCE = new Pose2d(-40,-ROBOT_WIDTH/2-4 , Math.toRadians(180));
        Pose2d CENTRE_LEFT_AUDIENCE = new Pose2d(-56,-ROBOT_WIDTH/2-4 , Math.toRadians(0));
        Pose2d LEFT_MID_AUDIENCE = new Pose2d(-40, -ROBOT_WIDTH/2-9, Math.toRadians(180));
        Pose2d LEFT_BACKBOARD = new Pose2d(45, -ROBOT_WIDTH/2-3, Math.toRadians(180));
        Pose2d LEFT_MID_BACKBOARD = new Pose2d(45, -17, Math.toRadians(180));

        Robot robot = new Robot(hardwareMap, telemetry);
        RedPositionDetection posDetect = new RedPositionDetection(hardwareMap);

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
                        //.splineToSplineHeading(RIGHT_PROP_POSITION, RIGHT_PROP_POSITION.getHeading())
                        .build()
        };

        TrajectorySequence toPreBar[] = {
                robot.drive.trajectorySequenceBuilder(toSpikeMark[0].end())
                        .lineToSplineHeading(LEFT_AUDIENCE)
                        .build(),
                robot.drive.trajectorySequenceBuilder(toSpikeMark[1].end())
                        .lineToSplineHeading(CENTRE_LEFT_AUDIENCE)
                        .turn(Math.toRadians(180))
                        .build(),
                robot.drive.trajectorySequenceBuilder(toSpikeMark[2].end())
                        .lineToSplineHeading(LEFT_AUDIENCE)
                        .build()
        };

        Trajectory toPostBar[] = {
                robot.drive.trajectoryBuilder(toPreBar[0].end())
                        .lineToSplineHeading(LEFT_BACKBOARD)
                        .build(),
                robot.drive.trajectoryBuilder(toPreBar[1].end())
                        .lineToSplineHeading(LEFT_MID_BACKBOARD)
                        .build(),
                robot.drive.trajectoryBuilder(toPreBar[2].end())
                        .lineToSplineHeading(LEFT_BACKBOARD)
                        .build()
        };

        Trajectory toBackBoard[] = {
                robot.drive.trajectoryBuilder(toPostBar[0].end())
                        .lineToSplineHeading(BACKBOARD_LEFT)
                        .build(),
                robot.drive.trajectoryBuilder(toPostBar[1].end())
                        .lineToSplineHeading(BACKBOARD_MID)
                        .build(),
                robot.drive.trajectoryBuilder(toPostBar[2].end())
                        .lineToSplineHeading(BACKBOARD_RIGHT)
                        .build()
        };

        Trajectory back[] = {
            robot.drive.trajectoryBuilder(toBackBoard[0].end())
                .forward(4)
                .build(),
            robot.drive.trajectoryBuilder(toBackBoard[1].end())
                .forward(4)
                .build(),
            robot.drive.trajectoryBuilder(toBackBoard[2].end())
                .forward(4)
                .build()
        };

        Trajectory toPark[] = {
            robot.drive.trajectoryBuilder(back[0].end())
                .strafeLeft(30)
                .build(),
            robot.drive.trajectoryBuilder(back[1].end())
                .strafeLeft(30)
                .build(),
            robot.drive.trajectoryBuilder(back[2].end())
                .strafeLeft(30)
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

        robot.stateManager.setIntakePower(0.5);
        robot.blockingTimer(robot.createTimerReference(2));
        robot.stateManager.setIntakePower(0);


        robot.executeTrajectorySequence(toPreBar[propPos]);
        robot.blockingTimer(robot.createTimerReference(3));
        robot.executeTrajectory(toPostBar[propPos]);
        robot.executeTrajectory(toBackBoard[propPos]);

        robot.stateManager.setScoringHeight(400);
        robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
        robot.blockingTimer(robot.createTimerReference(4));
        robot.stateManager.setIsOutaking(true);
        robot.blockingTimer(robot.createTimerReference(2));
        robot.stateManager.setIsOutaking(false);
        robot.stateManager.setRobotState(StateManager.RobotState.INTAKING);
        robot.blockingTimer(robot.createTimerReference(2));

//        robot.executeTrajectory(back[propPos]);
//        robot.executeTrajectory(toPark[propPos]);
    }
}
