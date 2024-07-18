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

package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
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
public class RedPropProcessor extends PropProcessor {

    public static int lowY = 40;
    public static int lowCr = 140;
    public static int lowCb = 100;
    public static int highY = 70;
    public static int highCr = 200;
    public static int highCb = 200;
    public Scalar lower = new Scalar(lowY,lowCr,lowCb);
    public Scalar upper = new Scalar(highY,highCr,highCb);

    public RedPropProcessor(Telemetry telemetry) {
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
        if(centerX <= 375 && area >= 200 ){ //bottom half
            location = Location.MIDDLE;
            telemetry.addData("Position:", " MIDDLE");
        }else if(centerX >= 375 && centerX <= 1200  && area >= 200){
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

}