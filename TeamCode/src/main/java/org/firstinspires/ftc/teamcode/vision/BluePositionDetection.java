package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class BluePositionDetection {
    OpenCvCamera camera;
    BluePropPipeline BluePropPipeline;

    int result = 1;

    public BluePositionDetection(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BluePropPipeline = new BluePropPipeline(0,0,0,0);

        BluePropPipeline.configureScalarLower(BluePropPipeline.scalarLowerYCrCb.val[0], BluePropPipeline.scalarLowerYCrCb.val[1], BluePropPipeline.scalarLowerYCrCb.val[2]);
        BluePropPipeline.configureScalarUpper(BluePropPipeline.scalarUpperYCrCb.val[0], BluePropPipeline.scalarUpperYCrCb.val[1], BluePropPipeline.scalarUpperYCrCb.val[2]);

        camera.setPipeline(BluePropPipeline);
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
        BluePropPipeline.configureBorders(0,0,0,0);
        if(BluePropPipeline.error){

        }
        // Only use this line of the code when you want to find the lower and upper values
//            testing(myPipeline);


            if(BluePropPipeline.getRectMidpointX() == 600.5){
                result = 1;
            }
            else if(BluePropPipeline.getRectMidpointX() > 400){
                result = 3;
            }
            else if(BluePropPipeline.getRectMidpointX() <= 400){
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
        return BluePropPipeline.getRectMidpointX();
    }
}
