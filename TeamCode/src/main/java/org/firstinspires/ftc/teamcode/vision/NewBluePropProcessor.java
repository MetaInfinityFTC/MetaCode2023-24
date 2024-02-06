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
public class NewBluePropProcessor implements VisionProcessor {

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
    public static int lowY = 30;
    public static int lowCr = 50;
    public static int lowCb = 50;
    public static int highY = 70;
    public static int highCr = 120;
    public static int highCb = 250;
    public Scalar lower = new Scalar(lowY,lowCr,lowCb);
    public Scalar upper = new Scalar(highY,highCr,highCb);

    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    public ColorSpace colorSpace = ColorSpace.YCrCb;
    private Mat ycrcbMat       = new Mat();
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

    public NewBluePropProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Scalar lower = new Scalar(lowY,lowCr,lowCb);
        Scalar upper = new Scalar(highY,highCr,highCb);


        Imgproc.cvtColor(frame, ycrcbMat, colorSpace.cvtCode);

        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        maskedInputMat.release();

        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat);

        //use binary mat from here
        List<MatOfPoint> countersList = new ArrayList<>();
        Imgproc.findContours(binaryMat, countersList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(binaryMat, countersList,0, new Scalar(0,0,255));

        Rect hat = new Rect(new Point(0,0), new Point(1,1));

        for (MatOfPoint countor : countersList)
        {

            Rect rect = Imgproc.boundingRect(countor);
            int centerY = rect.y + rect.height;
            if (rect.area() > hat.area() && centerY >= 480/2) {
                hat = rect;
            }

        }

        Imgproc.rectangle(maskedInputMat, hat, new Scalar(255,255,255));

        int centerX = hat.x + hat.width;
        int area = (int) hat.area();
        telemetry.addData("Area: ", area);
        telemetry.addData("CenterX: ", centerX);
        if(centerX <= 300 && area >= 20 ){ //bottom half
            location = NewBluePropProcessor.Location.MIDDLE;
            telemetry.addData("Position:", " MIDDLE");
        }else if(centerX >= 300 && centerX <= 800 && area >= 20){
            location = NewBluePropProcessor.Location.RIGHT;
            telemetry.addData("Position:", " RIGHT");
        }else{
            location = NewBluePropProcessor.Location.LEFT;
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