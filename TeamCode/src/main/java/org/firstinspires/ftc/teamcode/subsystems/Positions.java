package org.firstinspires.ftc.teamcode.subsystems;

public class Positions {

    public enum OutakePivot {
        INTAKE(240),
        OUTAKE(147),//305
        FAR_OUTAKE(137),
        MEDIUM_OUTAKE(142),
        CLIMB(0);

        public double position;

        OutakePivot(double position) {this.position = position;}
    }

    public enum Outake {
        INTAKING(-1),
        STOPPED(0),
        OUTTAKING(0.65),
        AUTO_OUTTAKING(0.4);

        double power;

        Outake(double power) {this.power = power;}
    }

    public enum Intake {
        INTAKING(-1),
        STOPPED(0),
        OUTTAKING(1);

        double power;

        Intake(double power) {this.power = power;}
    }

}
