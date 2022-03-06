package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ColorClass;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// Credit to several Stack Overflow users (your work isn't in vain but idk who you are) and the OpenCV Java tutorial.

@TeleOp(name = "Color Finder")
public class ColorFinder extends LinearOpMode {
    private OpenCvCamera webcam;


    @Override
    public void runOpMode() {

        // VISION
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        ColorClass.ColorDetection pipeline = new ColorClass.ColorDetection();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Streaming on http://192.168.43.1:8080/dash");

        waitForStart();

        while (opModeIsActive()){
            telemetry.addLine("Streaming on http://192.168.43.1:8080/dash");
            telemetry.addData("Object Center", pipeline.getCenter());
            telemetry.update();
            sleep(50);
        }

    }
}