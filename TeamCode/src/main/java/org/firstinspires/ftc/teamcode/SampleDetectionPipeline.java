package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.*;

public class SampleDetectionPipeline extends OpenCvPipeline {
    // Image sizes
    private final Size imageSize = new Size(640, 360);
    private final Size warpedImageSize = new Size(300, 450);
    private final int modelImageWidth = 320;
    private final int modelImageHeight = 480;

    // Perspective transformation source and destination points
    private final MatOfPoint2f imagePoints = new MatOfPoint2f(
            new Point(168, 43),    // Top-left
            new Point(391, 44),    // Top-right
            new Point(473, 158),    // Bottom-right
            new Point(3, 163)    // Bottom-left
    );
    private final MatOfPoint2f objectPoints = new MatOfPoint2f(
            new Point(0, 0),       // New top-left
            new Point(300, 0),     // New top-right
            new Point(300, 300),   // New bottom-right
            new Point(0, 300)      // New bottom-left
    );

    // Camera intrinsic parameters
    private final Mat cameraMatrix;
    private final Mat distCoeffs;
    
    // Real world measurements (in mm) converted to pixel scale
    private final int warpedWidthMM = 630;
    private final int sampleWidth;   // mm_to_pixel(38)
    private final int sampleHeight;  // mm_to_pixel(88)
    private final int sampleDepth;   // mm_to_pixel(38)

    // Camera 3D position
    private final int cameraX = 202;
    private final int cameraY = 487;
    private final int cameraZ = 115;

    // (Optional) Telemetry if you want to output values to the driver station
    private Telemetry telemetry;

    private YoloV8TFLiteDetector detector;

    public SampleDetectionPipeline(Context context, Telemetry telemetry) {
        this.telemetry = telemetry;
        detector = new YoloV8TFLiteDetector(context);
        
        // Initialize camera calibration matrices
        cameraMatrix = Mat.eye(3, 3, CvType.CV_32F);
        cameraMatrix.put(0, 0, 345.96589147568955, 0.0, 311.7387017952808,
                              0.0, 347.85001143507657, 173.23578163143304,
                              0.0, 0.0, 1.0);
        distCoeffs = new Mat(1, 5, CvType.CV_32F);
        distCoeffs.put(0, 0, -0.30485567438668043, 0.07975532745501325,
                            -0.0003135652017283016, -0.00207616958375859,
                             0.011229097994240867);

        // Define helper conversion functions via stored values
        sampleWidth = mmToPixel(38);
        sampleHeight = mmToPixel(88);
        sampleDepth = mmToPixel(38);
    }

    // Conversion functions between mm and pixel (based on warped image width)
    private int mmToPixel(double mm) {
        return (int) (mm * warpedImageSize.width / warpedWidthMM);
    }
    private int pixelToMM(double pixel) {
        return (int) (pixel * warpedWidthMM / warpedImageSize.width);
    }

    // Helper: Euclidean distance between two points.
    private double distance(Point a, Point b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    // Helper: Compute the angle (in degrees) between the vectors BA and BC.
    private double angleBetween(Point a, Point b, Point c) {
        double[] ba = {a.x - b.x, a.y - b.y};
        double[] bc = {c.x - b.x, c.y - b.y};
        double dot = ba[0] * bc[0] + ba[1] * bc[1];
        double normBA = Math.hypot(ba[0], ba[1]);
        double normBC = Math.hypot(bc[0], bc[1]);
        double cosineAngle = dot / (normBA * normBC);
        cosineAngle = Math.max(-1, Math.min(1, cosineAngle)); // clip to [-1,1]
        return Math.toDegrees(Math.acos(cosineAngle));
    }

    // Helper: Find point on the line from A to B at a given distance from A.
    private Point pointOnLine(Point A, Point B, double dist) {
        double dx = B.x - A.x;
        double dy = B.y - A.y;
        double len = Math.hypot(dx, dy);
        return new Point(A.x + (dist * dx / len), A.y + (dist * dy / len));
    }

    // Helper: Approximates the bottom vertex given a top vertex based on camera geometry.
    private Point sampleTopVertexToBottom(Point pt) {
        // Calculate: pt + (camera_position_xy - pt) * (sample_depth / cameraZ)
        double newX = pt.x + (cameraX - pt.x) * ((double) sampleDepth / cameraZ);
        double newY = pt.y + (cameraY - pt.y) * ((double) sampleDepth / cameraZ);
        return new Point(newX, newY);
    }

    @Override
    public Mat processFrame(Mat input) {
        // Resize input if needed
        Mat resized = new Mat();
        Imgproc.resize(input, resized, imageSize);

        // Undistort the image
        Mat undistorted = new Mat();
        Calib3d.undistort(resized, undistorted, cameraMatrix, distCoeffs);

        // Perspective transform
        Mat M = Imgproc.getPerspectiveTransform(imagePoints, objectPoints);
        Mat warped = new Mat();
        Imgproc.warpPerspective(undistorted, warped, M, warpedImageSize);

        // Draw the original transformation polygon on the undistorted image
        List<Point> ptsList = imagePoints.toList();
        MatOfPoint ptsMat = new MatOfPoint();
        ptsMat.fromList(ptsList);
        Imgproc.polylines(undistorted, Collections.singletonList(ptsMat), true, new Scalar(0, 255, 0), 2);

        // Color segmentation: convert to HSV and threshold for red hues.
        Mat hsv = new Mat();
        Imgproc.cvtColor(warped, hsv, Imgproc.COLOR_BGR2HSV);
        Scalar lowerRed1 = new Scalar(0, 30, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 30, 100);
        Scalar upperRed2 = new Scalar(180, 255, 255);
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Core.inRange(hsv, lowerRed1, upperRed1, mask1);
        Core.inRange(hsv, lowerRed2, upperRed2, mask2);
        Mat mask = new Mat();
        Core.add(mask1, mask2, mask);

        // Erode then dilate to reduce noise.
        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        // Create segmented image based on mask.
        Mat segmented = new Mat();
        Core.bitwise_and(warped, warped, segmented, mask);

        // Convert the Mat to input tensor.
        float[][][][] inputTensor =
            new float[1][modelImageWidth][modelImageHeight][3];
        for (int i = 0; i < warpedImageSize.width; i++) {
            for (int j = 0; j < warpedImageSize.height; j++) {
                double[] pixel = warped.get(j, i); // returns [B, G, R]
                // Normalize pixel values to [0, 1]
                inputTensor[0][i][j][0] = (float) (pixel[2] / 255.0);
                inputTensor[0][i][j][1] = (float) (pixel[1] / 255.0);
                inputTensor[0][i][j][2] = (float) (pixel[0] / 255.0);
            }
        }

        // Run the detector inference.
        float[][][] output = detector.runInference(inputTensor);
        int numGridCells = output[0][0].length;

        // Post-process the output tensor to extract oriented bounding boxes.
        Mat detected = warped.clone();
        List<List<Point>> topRectangles = new ArrayList<>();

        for (int i = 0; i < numGridCells; i++) {
            float cy = output[0][0][i];  // Center Y
            float cx = output[0][1][i];  // Center X
            float h = output[0][2][i];  // Height
            float w = output[0][3][i];  // Width
            float c1 = output[0][4][i];  // C1
            float c2 = output[0][5][i];  // C2
            float c3 = output[0][6][i]; // C3
            float angle = output[0][7][i];  // Rotation angle
            cx = cx * modelImageWidth;
            cy = cy * modelImageHeight;
            w = w * modelImageWidth;
            h = h * modelImageHeight;
            angle = (float) -Math.toDegrees(angle);

            float confidence = Math.max(c1, Math.max(c2, c3));
            if (confidence < 0.4) continue; // Skip low-confidence detections

            // Create a rotated rectangle using the detection outputs.
            Point center = new Point(cx, cy);
            Size rectSize = new Size(w, h);
            RotatedRect rRect = new RotatedRect(center, rectSize, angle);

            // Obtain the four vertices of the rotated rectangle.
            Point[] vertices = new Point[4];
            rRect.points(vertices);
            topRectangles.add(Arrays.asList(vertices[0], vertices[1], vertices[2], vertices[3]));
        }

        // Calculate sample bottom rectangles.
        for (List<Point> topRect : topRectangles) {
            MatOfPoint topPts = new MatOfPoint();
            topPts.fromList(topRect);
            Imgproc.polylines(detected, Collections.singletonList(topPts), true, new Scalar(0, 255, 0), 2);
            List<Point> bottomRect = new ArrayList<>();
            for (Point p : topRect)
                bottomRect.add(sampleTopVertexToBottom(p));
            MatOfPoint bottomPts = new MatOfPoint();
            bottomPts.fromList(bottomRect);
            Imgproc.polylines(detected, Collections.singletonList(bottomPts), true, new Scalar(255, 255, 0), 2);
        }

        // Send telemetry data for debugging.
        telemetry.addData("Top Rectangles", topRectangles.size());
        telemetry.update();

        // Return the final annotated image.
        return detected;
    }

    public class YoloV8TFLiteDetector {
        private Interpreter tflite;
    
        // Constructor: load the model from the assets folder.
        public YoloV8TFLiteDetector(Context context) {
            try {
                tflite = new Interpreter(loadModelFile(context, "yolov8_obb.tflite"));
            } catch (IOException e) {
                throw new RuntimeException("Failed to load TFLite model", e);
            }
        }
    
        // Loads the TFLite model file from assets.
        private MappedByteBuffer loadModelFile(Context context, String modelPath) throws IOException {
            AssetFileDescriptor fileDescriptor = context.getAssets().openFd(modelPath);
            FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
            FileChannel fileChannel = inputStream.getChannel();
            long startOffset = fileDescriptor.getStartOffset();
            long declaredLength = fileDescriptor.getDeclaredLength();
            return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
        }
    
        // Runs inference on an input tensor.
        public float[][][] runInference(float[][][][] input) {
            float[][][] output = new float[1][8][3150];
            tflite.run(input, output);
            return output;
        }
    }
}
