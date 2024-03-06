package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.vision.processors.BluePropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Vision {

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    PropProcessor propProcessor;

    public static final double X_OFFSET = -7.25;
    public static final double Y_OFFSET = 3.125;
    public static final double THETA_OFFSET = Math.PI;


    public Vision(HardwareMap hardwareMap, Telemetry telemetry, boolean blue){
        propProcessor = blue ? new BluePropProcessor(telemetry) : new RedPropProcessor(telemetry);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS) //CHECK UNITS!
                .setLensIntrinsics(822.317f, 822.317f,319.495f, 242.502f)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.enableLiveView(true);
        builder.setCameraResolution(new Size(640, 480));

        builder.addProcessors(aprilTag,propProcessor);
        visionPortal = builder.build();

    }

    public PropProcessor.Location getPropLocation(){
        return propProcessor.getLocation();
    }


    public void disablePropProcessor(){
        visionPortal.setProcessorEnabled(propProcessor,false);
    }

    public List<Pose2d> getTagLocalizationData(){
        List<Pose2d> tagLocalizationData = new ArrayList<>();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                double xo = X_OFFSET;
                double yo = Y_OFFSET;
                double to = THETA_OFFSET;

                double xc = detection.ftcPose.x;
                double yc = detection.ftcPose.y;
                double tc = detection.ftcPose.yaw;

                double xa = detection.metadata.fieldPosition.get(0);
                double ya = detection.metadata.fieldPosition.get(1);
                double ta = detection.metadata.id <= 6 ? 0 : Math.PI; //tags 1 to 6 are backdrop, 7 to 10 are pixel stack

                double tg = ta - tc - to;

                to -= Math.PI/2; //clarity
                double v1 = Math.cos(to) * xc - Math.sin(to) * yc + xo;
                double v2 = Math.sin(to) * xc + Math.cos(to) * yc + yo;

                double xg = xa - (Math.cos(tg)*(v1) - Math.sin(tg)*(v1));
                double yg = ya - (Math.sin(tg)*(v2) + Math.cos(tg)*(v2));


                tagLocalizationData.add(new Pose2d(xg,yg,tg));

            }
        }

        return tagLocalizationData;

    }
}
