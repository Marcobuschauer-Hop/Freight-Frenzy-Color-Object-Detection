package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class ColorClass{
    public static class ColorDetection extends OpenCvPipeline {

        private static Point center;

        final Scalar detectionColor = new Scalar(255, 0, 0);

        // Finds the contours of the mask and draws a line around them
        private Mat findAndDraw(Mat maskedImage, Mat input) {
            // init
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            // find contours
            Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            // if any contour exist...
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                    Imgproc.drawContours(input, contours, idx, detectionColor);
                }
            }
            return input;
        }

        // Makes sure that the colors don't exceed 255 or 0.
        private int colorMinMax(int magnitude) {
            if (magnitude > 255) {
                return 255;
            } else return Math.max(magnitude, 0);
        }

        // Shifts the color by the given arbitrary amount
        private Scalar colorShift(Scalar input, int shiftAmount) {
            int one, two, three;

            one = (int) input.val[0] + shiftAmount;
            two = (int) input.val[1] + shiftAmount;
            three = (int) input.val[2] + shiftAmount;

            return new Scalar(colorMinMax(one), colorMinMax(two), colorMinMax(three), input.val[3]);
        }

        // Given the mask of an image, it finds each contour, selects the biggest one and returns the center of it
        private Point largestContourCenter(Mat mask) {
            List<MatOfPoint> Contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            // finds contours
            Imgproc.findContours(mask, Contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            // Finds the largest contour
            double maxVal = 0;
            int maxValIdx = 0;
            if (Contours.size() < 1) {
                return null;
            }
            for (int contourIdx = 0; contourIdx < Contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(Contours.get(contourIdx));
                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
            // Finds the center of the contour
            try {
                //If there is no contour, it returns an error which is caught here
                Moments p = Imgproc.moments(Contours.get(maxValIdx));
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());
                return new Point(x, y);
            } catch (Exception IndexOutOfBoundsException) {
                return null;
            }
        }

        // Function that gets called whenever the pipeline gets called
        @Override
        public Mat processFrame(Mat input) {
            Mat mask = new Mat();
            Mat morphOutput = new Mat();
            // robotics room
//            Scalar base = new Scalar(170, 140, 40, 255);

            //home
//            Scalar base = new Scalar(130, 102, 30, 255);

            // green cube
            Scalar base = new Scalar(131, 119, 90, 255);

            int shiftAmount = 72;


//            Scalar notebookRed = new Scalar(120, 36, 60, 255);
//            int shiftAmount = 64;

            // Assigning minimum values of the inrange func using shiftAmount
            Scalar minVals = colorShift(base, -1 * Math.round((float) shiftAmount / 2));
            Scalar maxVals = colorShift(base, Math.round((float) shiftAmount / 2));

            // Takes out the colors between minVals and maxVals then puts everything into mask.
            Core.inRange(input, minVals, maxVals, mask);


            // Makes the mask more pixilated for better contour detection
            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));
            Imgproc.erode(mask, morphOutput, erodeElement);
            Imgproc.erode(morphOutput, morphOutput, erodeElement);
            Imgproc.dilate(morphOutput, morphOutput, dilateElement);
            Imgproc.dilate(morphOutput, morphOutput, dilateElement);

            // Finds the center of the largest contour in the image
            center = largestContourCenter(morphOutput);

            if (center != null) {
                Imgproc.circle(input, center, 3, detectionColor, 3);
            }
            // returns frame for streaming to FTC dashboard
            return findAndDraw(morphOutput, input);
        }

        public Point getCenter() {
            // Can be null or have a Point value.
            return center;
        }



        public double getXCenter() {
            if (center == null) {
                return -1;
            }
            return center.x;
        }

        public double getYCenter() {
            if (center == null) {
                return -1;
            }
            return center.y;
        }
    }

    public static class maskCalibration extends OpenCvPipeline {
        Scalar centerColor;
        double xShifter = 0;
        double yShifter = 0;
        int shiftAmount = 72;
        int imageOption = 1;
        Mat mask = new Mat();
        Mat morphOutput = new Mat();
        Scalar detectionColor = new Scalar(255, 0, 0);
        Scalar minVals;
        Scalar maxVals;
        double[] temp;

        @Override
        public Mat processFrame(Mat input) {
            // gets the colors from Original and assigns it to double[] temp
            temp = input.get((input.rows() / 2) + (int) yShifter, (input.cols() / 2) + (int) xShifter);

            if (imageOption == 0 || imageOption == 1 || imageOption == 3) {
                // asserts not null result
                if (temp != null) {
                    centerColor = new Scalar(temp[0], temp[1], temp[2], 255);
                } else {
                    centerColor = null;
                }

                // stops code from breaking if null
                if (centerColor != null) {
                    // Assigning minimum values of the inrange func using shiftAmount
                    minVals = colorShift(centerColor, -1 * Math.round((float) shiftAmount / 2));
                    maxVals = colorShift(centerColor, Math.round((float) shiftAmount / 2));

                    // Takes out the colors between minVals and maxVals then puts everything into mask.
                    Core.inRange(input, minVals, maxVals, mask);

                    if (imageOption == 0) {
                        return mask;
                    }
                    // Makes the mask more pixilated for better contour detection
                    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
                    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));
                    Imgproc.erode(mask, morphOutput, erodeElement);
                    Imgproc.erode(morphOutput, morphOutput, erodeElement);
                    Imgproc.dilate(morphOutput, morphOutput, dilateElement);
                    Imgproc.dilate(morphOutput, morphOutput, dilateElement);

                    if (imageOption == 1) {
                        return morphOutput;
                    }

                    // Finds the center of the largest contour in the image
                    Point center = largestContourCenter(morphOutput);

                    if (center != null) {
                        Imgproc.circle(input, center, 3, detectionColor, 3);
                    }
                    // returns frame for streaming to FTC dashboard
                    return findAndDraw(morphOutput, input);
                } else {
                    // creates a blank (presumed black) mat and puts white text on it
                    mask = Mat.zeros(input.rows() / 2, input.cols() / 2, CvType.CV_8UC1);
                    morphOutput = Mat.zeros(input.rows() / 2, input.cols() / 2, CvType.CV_8UC1);
                    Imgproc.putText(mask, "No Color Detected", new Point(input.size().width / 2, input.size().height / 2), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 255, 255), 1);
                    Imgproc.putText(morphOutput, "No Color Detected", new Point(input.size().width / 2, input.size().height / 2), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 255, 255), 1);

                }
            }

            // essentially still an if/else statement as code will return in the if statement, if met
            if (temp != null) {
                detectionColor = invertColor(temp);
            }
            Imgproc.circle(input, new Point((input.size().width / 2) + xShifter, (input.size().height / 2) + yShifter), 3, detectionColor, 3);
            Imgproc.putText(input, Arrays.toString(temp), new Point((input.size().width / 2) + xShifter, (input.size().height / 2) + yShifter), Imgproc.FONT_HERSHEY_COMPLEX, 1, detectionColor, 1);
            return input;

        }

        // sets the x value of the detection point
        public void XCenter(double newXCenter) {
            xShifter = newXCenter;
        }

        // sets the y value of the detection point
        public void YCenter(double newYCenter) {
            yShifter = newYCenter;
        }

        // sets the shift amount in the inRange function
        public void newShiftAmount(int newNumber) {
            shiftAmount = newNumber;
        }

        // returns the required displayed image
        public void setImageOption(int option) {
            imageOption = option;
        }

        // returns the x value of the detection point
        public double getXCenter() {
            return xShifter;
        }

        // returns the y value of the detection point
        public double getYCenter() {
            return yShifter;
        }

        // returns the values of the shift amount in the inRange function
        public int getShiftAmount() {
            return shiftAmount;
        }

        // gets the color at the detection point in rgb
        public Scalar getPoint() {
            // Can be null or have a Point value.
            return centerColor;
        }

        // Makes sure that the colors don't exceed 255 or 0.
        private int colorMinMax(int magnitude) {
            if (magnitude > 255) {
                return 255;
            } else return Math.max(magnitude, 0);
        }

        // Shifts the color by the given arbitrary amount
        private Scalar colorShift(Scalar input, int shiftAmount) {
            int one, two, three;

            one = (int) input.val[0] + shiftAmount;
            two = (int) input.val[1] + shiftAmount;
            three = (int) input.val[2] + shiftAmount;

            return new Scalar(colorMinMax(one), colorMinMax(two), colorMinMax(three), input.val[3]);
        }

        private Scalar invertColor(double[] input) {
            return new Scalar(255 - input[0], 255 - input[1], 255 - input[2], 255);
        }

        // Given the mask of an image, it finds each contour, selects the biggest one and returns the center of it
        private Point largestContourCenter(Mat mask) {
            List<MatOfPoint> Contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            // finds contours
            Imgproc.findContours(mask, Contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            // Finds the largest contour
            double maxVal = 0;
            int maxValIdx = 0;
            if (Contours.size() < 1) {
                return null;
            }
            for (int contourIdx = 0; contourIdx < Contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(Contours.get(contourIdx));
                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
            // Finds the center of the contour
            try {
                //If there is no contour, it returns an error which is caught here
                Moments p = Imgproc.moments(Contours.get(maxValIdx));
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());
                return new Point(x, y);
            } catch (Exception IndexOutOfBoundsException) {
                return null;
            }
        }

        // Finds the contours of the mask and draws a line around them
        private Mat findAndDraw(Mat maskedImage, Mat input) {
            // init
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            // find contours
            Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            // if any contour exist...
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                    Imgproc.drawContours(input, contours, idx, detectionColor);
                }
            }
            return input;
        }
    }
}
