package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ColorClass;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/*
Documentation:
This function seeks to calibrate the cube detection by showing what the mask looks like with different colors from the camera selected
By using gamepad1.left_stick_x/y, the detection point can be moved around to calibrate the base detection color
Pressing (or holding) gamepad1.x increases the shift amount, gamepad1.y decreases the shift amount.
Pressing gamepad1.a switches the mode of the
 */

@TeleOp(name = "Camera Calibration", group = "Vision")
public class ColorCalibration extends LinearOpMode {
    private static double keepInRange(double min, double max, double input, boolean rollover) {
        if (max < min) {
            return (max - min) / 2;
        }
        if (!rollover) {
            if (input > max) {
                return max;
            } else return Math.max(input, min);
        }
        else {
            if (min % 1 == 0 && max % 1 == 0) {
                if (input > max) {
                    input = min + ((input - max) % (max - min));
                } else if (input < min) {
                    input = (max - (Math.abs(input - min) % (max - min))) - 1;
                }
                return input;

            } else {
                return (max - min) / 2;
            }
        }
    }

    @Override
    public void runOpMode() {

        // inits
        final String[] modes = new String[]{"Raw Mask", "Eroded/Dilated Mask", "Color Calibration", "Drawn Calibration"};
        final ElapsedTime timer = new ElapsedTime();
        int modeSelection = 0;


        // VISION
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        ColorClass.maskCalibration pipeline = new ColorClass.maskCalibration();
        FtcDashboard.getInstance().startCameraStream(webcam, 15);
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

        timer.reset();
        telemetry.addLine("Streaming on http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double xCenter = pipeline.getXCenter();
            double yCenter = pipeline.getYCenter();
            xCenter += gamepad1.left_stick_x / 100;
            yCenter += gamepad1.left_stick_y / 100;
            pipeline.XCenter(keepInRange(-640, 640, xCenter, false));
            pipeline.YCenter(keepInRange(-360, 360, yCenter, false));


            if (gamepad1.a && timer.milliseconds() > 250) {
                modeSelection = (int) keepInRange(0, 3, modeSelection + 1, true);
                timer.reset();
            }
            pipeline.setImageOption(modeSelection);

            if (timer.milliseconds() > 250 && gamepad1.x) {
                pipeline.newShiftAmount(pipeline.getShiftAmount() + 1);
                timer.reset();
            }

            if (timer.milliseconds() > 250 && gamepad1.y) {
                pipeline.newShiftAmount(pipeline.getShiftAmount() - 1);
                timer.reset();
            }


            telemetry.addData("Center Color", pipeline.getPoint());
            telemetry.addData("centerCoordinate", new Point(keepInRange(-640, 640, xCenter, false), keepInRange(-360, 360, yCenter, false)));
            telemetry.addData("shiftAmount", pipeline.getShiftAmount());
            telemetry.addData("Mode", modes[modeSelection]);
            telemetry.update();
        }

    }
}