/*
 * Copyright (c) 2023 Sebastian Erives
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class NewRedPropProcessor implements VisionProcessor {

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    public static int lowL = 0;
    public static int lowA = 50;
    public static int lowB = 50;
    public static int highL = 100;
    public static int highA = 128;
    public static int highB = 128;
    public Scalar lower = new Scalar(lowL,lowA,lowB);
    public Scalar upper = new Scalar(highL,highA,highB);

    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    public ColorSpace colorSpace = ColorSpace.Lab;
    private Mat labMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();

    private Telemetry telemetry = null;

    public enum Location{
        LEFT,MIDDLE,RIGHT
    }

    Location location;
    enum ColorSpace {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);

        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public NewRedPropProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Scalar lower = new Scalar(lowL,lowA,lowB);
        Scalar upper = new Scalar(highL,highA,highB);


        Imgproc.cvtColor(frame, labMat, colorSpace.cvtCode);

        Core.inRange(labMat, lower, upper, binaryMat);

        maskedInputMat.release();

        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat);


        //use binary mat from here
        List<MatOfPoint> countersList = new ArrayList<>();
        Imgproc.findContours(binaryMat, countersList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(binaryMat, countersList,0, new Scalar(255,0,0));

        Rect hat = new Rect(new Point(0,0), new Point(1,1));

        for (MatOfPoint countor : countersList)
        {

            Rect rect = Imgproc.boundingRect(countor);

            int centerY = rect.y + rect.height;
            if (rect.area() > hat.area() && centerY >= 480/2 ) {
                hat = rect;
            }

        }

        Imgproc.rectangle(maskedInputMat, hat, new Scalar(255,255,255));

        int centerX = hat.x + hat.width;
        int area = (int) hat.area();
        telemetry.addData("Area: ", area);
        telemetry.addData("CenterX: ", centerX);
        if(centerX <= 500 && area >= 2000 ){ //bottom half
            location = Location.MIDDLE;
            telemetry.addData("Position:", " MIDDLE");
        }else if(centerX >= 500 && centerX <= 1200  && area >= 2000){
            location = Location.RIGHT;
            telemetry.addData("Position:", " RIGHT");
        }else{
            location = Location.LEFT;
            telemetry.addData("Position:", " LEFT");
        }
        telemetry.update();

        maskedInputMat.copyTo(frame);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public Location getLocation(){
        return location;
    }
}