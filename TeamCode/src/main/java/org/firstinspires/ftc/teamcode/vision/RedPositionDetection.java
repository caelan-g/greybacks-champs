package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RedPositionDetection {
    OpenCvCamera camera;
    RedPropPipeline redPropPipeline;

    int result = 1;

    public RedPositionDetection(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        redPropPipeline = new RedPropPipeline(0,0,0,0);

        redPropPipeline.configureScalarLower(redPropPipeline.scalarLowerYCrCb.val[0], redPropPipeline.scalarLowerYCrCb.val[1], redPropPipeline.scalarLowerYCrCb.val[2]);
        redPropPipeline.configureScalarUpper(redPropPipeline.scalarUpperYCrCb.val[0], redPropPipeline.scalarUpperYCrCb.val[1], redPropPipeline.scalarUpperYCrCb.val[2]);

        camera.setPipeline(redPropPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void stopDetector() {
        camera.closeCameraDevice();
    }

    private void runDetection() {
        redPropPipeline.configureBorders(0,0,0,0);
        if(redPropPipeline.error){

        }
        // Only use this line of the code when you want to find the lower and upper values
//            testing(myPipeline);


            if(redPropPipeline.getRectMidpointX() == 600.5){
                result = 1;
            }
            else if(redPropPipeline.getRectMidpointX() > 400){
                result = 3;
            }
            else if(redPropPipeline.getRectMidpointX() <= 400){
                result = 2;
            }


    }

    public void updateDetector() {
        runDetection();
    }

    public int getPropPosition() {
        return result;
    }

    public double getPropExactPosition() {
        return redPropPipeline.getRectMidpointX();
    }
}
