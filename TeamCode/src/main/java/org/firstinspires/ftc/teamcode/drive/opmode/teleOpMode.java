package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.StateManager;
import org.firstinspires.ftc.teamcode.subsystems.Positions;

@Config
@TeleOp(group = "drive")
public class teleOpMode extends LinearOpMode {

    NanoClock clock = NanoClock.system();

    double lastLoopTime = 0;
    double thisLoopTime = 0;
    double elevatorTarget= 0;
    boolean canOutake = true;
    boolean canUp = true;
    boolean canDown = false;
    double middleElevatorTarget = 233.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);


        while(opModeInInit()){
            robot.stopServos();
        }

        robot.stateManager.setRobotState(StateManager.RobotState.INTAKING);
        robot.drive.setExternalHeading(90);
        while (opModeIsActive()) {

            //robot.drive.setWeightedDrivePower(robot.drive.calculateFieldOrientedSignal(-gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x));
            if (gamepad1.right_trigger > 0)  {
                if (Math.abs(gamepad1.left_stick_x) > 0.2) robot.drive.setWeightedDrivePower(robot.drive.calculateFieldOrientedSignal(-gamepad1.right_stick_x/2, gamepad1.right_stick_y/2, Math.pow(gamepad1.left_stick_x, 3)/2));
                else robot.drive.setWeightedDrivePower(robot.drive.calculateFieldOrientedSignal(-gamepad1.right_stick_x/2, gamepad1.right_stick_y/2, 0));
            }
            else {
                if (Math.abs(gamepad1.left_stick_x) > 0.2) robot.drive.setWeightedDrivePower(robot.drive.calculateFieldOrientedSignal(-gamepad1.right_stick_x, gamepad1.right_stick_y, Math.pow(gamepad1.left_stick_x, 3)));
                else robot.drive.setWeightedDrivePower(robot.drive.calculateFieldOrientedSignal(-gamepad1.right_stick_x, gamepad1.right_stick_y, 0));
            }










            if(gamepad2.a) {
                elevatorTarget = middleElevatorTarget - 63.5;
                robot.stateManager.setScoringHeight(elevatorTarget);
                robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
            }
            if(gamepad2.x) {
                elevatorTarget = middleElevatorTarget;
                robot.stateManager.setScoringHeight(elevatorTarget);
                robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
            }
            if(gamepad2.b) {
                elevatorTarget = 0;
                robot.stateManager.setRobotState(StateManager.RobotState.INTAKING);
            }
            if(gamepad2.y) {
                elevatorTarget = middleElevatorTarget + 63.5;
                robot.stateManager.setScoringHeight(elevatorTarget);
                robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
            }

            if (gamepad2.start) {
                robot.stateManager.setScoringHeight(400);
                robot.stateManager.setRobotState(StateManager.RobotState.CLIMBING);
            }

            if (gamepad2.back) {
                robot.stateManager.setScoringHeight(150);
                robot.stateManager.setRobotState(StateManager.RobotState.CLIMBING);
            }

            if (gamepad2.dpad_up && canUp) {
                canUp = false;
                middleElevatorTarget += 63.5;
                robot.stateManager.setScoringHeight(elevatorTarget);
                robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
            }
            else {
                canUp = true;
            }
            if (gamepad2.dpad_down && canDown) {
                canDown = false;
                middleElevatorTarget -= 63.5;
                robot.stateManager.setScoringHeight(elevatorTarget);
                robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
            }
            else {
                canDown = true;
            }

            if (gamepad2.dpad_left) {
                robot.stateManager.setOutakePosition(Positions.OutakePivot.OUTAKE);
            }
            if (gamepad2.dpad_right) {
                robot.stateManager.setOutakePosition(Positions.OutakePivot.FAR_OUTAKE);
            }



            if (gamepad2.left_trigger > 0) robot.stateManager.setIntakePower(-1);
            else if (gamepad2.right_bumper) robot.stateManager.setOutakePosition(Positions.OutakePivot.MEDIUM_OUTAKE);
            else robot.stateManager.setIntakePower(0);

            if (gamepad2.right_trigger > 0) {
                if (canOutake) {
                    robot.verticalElevator.setPowerConstraint(0.6);
                    canOutake = false;
                    robot.stateManager.setIsOutaking(true);
                    robot.stateManager.setIntakePower(1);
                    robot.blockingTimer(robot.createTimerReference(0.2));
                    elevatorTarget += 127;
                    robot.stateManager.setScoringHeight(elevatorTarget);
                    robot.stateManager.setRobotState(StateManager.RobotState.OUTAKING);
                }
            }
            else {
                canOutake = true;
                robot.verticalElevator.setPowerConstraint(1);
                robot.stateManager.setIsOutaking(false);
            }


            if (gamepad2.left_bumper) robot.planeLauncher.shootPlane(0);

            if (gamepad1.back) robot.drive.setExternalHeading(0);
            if (gamepad1.start) robot.drive.setExternalHeading(90);

            telemetry.addData("heading", robot.drive.getPoseEstimate());
            telemetry.addData("heading2", robot.drive.getExternalHeading());
            telemetry.addData("X", robot.drive.getPoseEstimate().getX());
            telemetry.addData("Y", robot.drive.getPoseEstimate().getY());
            telemetry.addData("field oriented signal X", robot.drive.calculateFieldOrientedSignal(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x).getX());
            telemetry.addData("field oriented signal Y", robot.drive.calculateFieldOrientedSignal(-gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x).getY());
            telemetry.addData("wheel positions", robot.drive.getWheelPositions());
            telemetry.addData("elevator target: ", elevatorTarget);

            robot.update();
            telemetry.update();

            if(isStopRequested()){
                robot.stopServos();
                sleep(10000);
            }
        }
    }


}
