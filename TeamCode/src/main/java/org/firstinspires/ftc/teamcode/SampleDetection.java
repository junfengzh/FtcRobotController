package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvWebcam.StreamFormat;
import org.opencv.core.*;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "SampleDetection", group = "Autonomous")
public class SampleDetection extends LinearOpMode {
    OpenCvWebcam webcam;
    SampleDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        
        pipeline = new SampleDetectionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720,
                    OpenCvCameraRotation.UPRIGHT, StreamFormat.MJPEG);
            }
        
            @Override
            public void onError(int errorCode) {
                // Handle error appropriately
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
