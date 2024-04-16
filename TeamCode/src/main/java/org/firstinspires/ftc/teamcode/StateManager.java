package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outake;
import org.firstinspires.ftc.teamcode.subsystems.OutakePivot;
import org.firstinspires.ftc.teamcode.subsystems.Positions;
import org.firstinspires.ftc.teamcode.subsystems.VerticalElevator;

public class StateManager {

    NanoClock clock = NanoClock.system();

    private Intake intake;
    private Outake outake;
    private OutakePivot outakePivot;
    private VerticalElevator verticalElevator;

    private RobotState robotState = RobotState.LOADED;
    private RobotState requestedState = RobotState.LOADED;
    private boolean transitionActive = false;
    private double transitionStartTime;
    private boolean upFinished = false;

    private double scoringHeight = 450;
    private double outakePosition = Positions.OutakePivot.MEDIUM_OUTAKE.position;
    private boolean isOutaking = false;
    private double intakePower = 0;

    public StateManager(Intake intake, Outake outake, OutakePivot outakePivot, VerticalElevator verticalElevator){
        this.intake = intake;
        this.outake = outake;
        this.outakePivot = outakePivot;
        this.verticalElevator = verticalElevator;
    }

    public void update(){

        if(robotState!=requestedState){

            if(!transitionActive){
                transitionActive = true;
                transitionStartTime = clock.seconds();
            }

            double transitionTimer = clock.seconds() - transitionStartTime;

            switch(requestedState){
                case INTAKING:
                    if (robotState != RobotState.CLIMBING) {
                        if (robotState == RobotState.OUTAKING) {
                            if (verticalElevator.getHeight() <=250 && !upFinished) {
                                upFinished = false;
                                verticalElevator.setTargetPosition(255);
                                transitionStartTime = clock.seconds()+0.5;
                            }
                            else if (transitionTimer < 0.2) {
                                upFinished = true;
                                outakePivot.setTargetPosition(Positions.OutakePivot.INTAKE);
                            } else if (verticalElevator.getHeight() > 3) {
                                outakePivot.setTargetPosition(Positions.OutakePivot.INTAKE);
                                verticalElevator.setTargetPosition(0);
                            } else {
                                upFinished = false;
                                robotState = requestedState;
                            }
                        } else {
                            robotState = requestedState;
                        }
                    }
                    break;

                case OUTAKING:
                    if (robotState == RobotState.INTAKING) {
                        if (scoringHeight <= 250) {
                            if (verticalElevator.getHeight() < 250) {
                                verticalElevator.setTargetPosition(255);
                                transitionStartTime = clock.seconds();
                            }
                            else if (transitionStartTime < 0.5) {
                                outakePivot.setTargetPosition(outakePosition);
                            }
                            else if (transitionStartTime < 1) {
                                outakePivot.setTargetPosition(outakePosition);
                                verticalElevator.setTargetPosition(scoringHeight);
                            }
                            else {
                                robotState = requestedState;
                            }
                        }
                        else {
                            if (verticalElevator.getHeight() < 250) {
                                verticalElevator.setTargetPosition(scoringHeight);
                                transitionStartTime = clock.seconds();
                            } else if (transitionStartTime < 0.5) {
                                verticalElevator.setTargetPosition(scoringHeight);
                                outakePivot.setTargetPosition(outakePosition);
                            } else {
                                robotState = requestedState;
                            }
                        }
                    }
                    else {
                        robotState = requestedState;
                    }
                    break;

                case CLIMBING:
                    if (robotState != RobotState.INTAKING) {
                        robotState = requestedState;
                    }
                    break;

            }
        }
        else {

            transitionActive = false;
            switch(robotState){
                case INTAKING:
                    verticalElevator.setTargetPosition(0);
                    outakePivot.setTargetPosition(Positions.OutakePivot.INTAKE);
                    intake.setServoPower(intakePower);
                    if (intakePower <= 0) outake.setServoPower(intakePower);
                    break;

                case OUTAKING:
                    verticalElevator.setTargetPosition(scoringHeight);
                    outakePivot.setTargetPosition(outakePosition);
                    intake.setServoPower(Positions.Intake.STOPPED);
                    if (isOutaking) outake.setServoPower(Positions.Outake.OUTTAKING);
                    else if (intakePower == -1) outake.setServoPower(Positions.Outake.INTAKING);
                    else outake.setServoPower(Positions.Outake.STOPPED);
                    break;

                case FAR_OUTAKING:
                    verticalElevator.setTargetPosition(scoringHeight);
                    outakePivot.setTargetPosition(outakePosition);
                    if (isOutaking) outake.setServoPower(Positions.Outake.AUTO_OUTTAKING);
                    else outake.setServoPower(Positions.Outake.STOPPED);
                    break;

                case CLIMBING:
                    verticalElevator.setTargetPosition(scoringHeight);
                    outakePivot.setTargetPosition(Positions.OutakePivot.CLIMB);
                    break;
            }

        }

    }

    public void setRobotState(RobotState state){
        this.requestedState = state;
    }

    public enum RobotState {
        INTAKING(),
        LOADED(),
        OUTAKING(),
        FAR_OUTAKING(),
        CLIMBING(),
        BACKBOARD_SCORE(),
    }

    public void setScoringHeight(double scoringHeight) {
        this.scoringHeight = scoringHeight;
    }

    public void setOutakePosition(Positions.OutakePivot outakePosition) {
        this.outakePosition = outakePosition.position;
    }

    public void setIsOutaking(boolean isOutaking) {
        this.isOutaking = isOutaking;
    }
    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public boolean isTransitionActive(){
        return transitionActive;
    }

    public RobotState getRobotState() {
        return robotState;
    }
}
