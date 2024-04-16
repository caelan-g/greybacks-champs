package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class PlaneLauncher {

    private ServoImplEx launcher;

    public PlaneLauncher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(ServoImplEx.class, "sCHub5");
        launcher.setPwmRange(new PwmControl.PwmRange(600, 2400));
    }

    public void shootPlane(double position) {
        launcher.setPosition(position);
    }
}
