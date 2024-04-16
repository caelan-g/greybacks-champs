
package org.firstinspires.ftc.teamcode.drive.opmode;

        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.vision.RedPositionDetection;
        import org.firstinspires.ftc.teamcode.Robot;


@Config
@Autonomous(group = "drive")
public class propTest extends LinearOpMode {

    private int propPos = 1;
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, telemetry);
        RedPositionDetection posDetect = new RedPositionDetection(hardwareMap);

        robot.drive.setPoseEstimate(new Pose2d(0,0,0));

        while(opModeInInit()){

                posDetect.updateDetector();
                telemetry.addData("Position Number: ", posDetect.getPropPosition());
                telemetry.addData("position: ", posDetect.getPropExactPosition());
                telemetry.update();
                propPos = posDetect.getPropPosition();

        }

        posDetect.stopDetector();

        if (isStopRequested()) return;

        telemetry.addData("finalChoice: ",propPos);
        if(propPos == 3){

        }
        else if(propPos == 2){

        }
        else {

        }
        while (true);
    }
}
