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
            // Tiles on desk
            new Point(202, 43),    // Top-left
            new Point(451, 47),    // Top-right
            new Point(631, 216),    // Bottom-right
            new Point(71, 221)    // Bottom-left

            // Actual field
            // new Point(168, 43),    // Top-left
            // new Point(391, 44),    // Top-right
            // new Point(473, 158),    // Bottom-right
            // new Point(3, 163)    // Bottom-left
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
    // Tiles on desk
    private final int warpedWidthMM = 630;
    // Actual field
    // private final int warpedWidthMM = 700;

    private final int sampleWidth;   // mm_to_pixel(38)
    private final int sampleHeight;  // mm_to_pixel(88)
    private final int sampleDepth;   // mm_to_pixel(38)

    // Camera 3D position
    // Tiles on desk
    private final int cameraX = 122;
    private final int cameraY = 432;
    private final int cameraZ = 129;

    // Actual field
    // private final int cameraX = 202;
    // private final int cameraY = 487;
    // private final int cameraZ = 115;

    private boolean detectBlue = true;
    private Telemetry telemetry;
    private YoloV8TFLiteDetector detector;

    private List<RotatedRect> detections = new ArrayList<>();

    public SampleDetectionPipeline(Context context, Telemetry telemetry, boolean detectBlue) {
        this.telemetry = telemetry;
        this.detectBlue = detectBlue;
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

    public List<RotatedRect> getDetections() {
        return this.detections;
    }

    // Conversion functions between mm and pixel (based on warped image width)
    private int mmToPixel(double mm) {
        return (int) (mm * warpedImageSize.width / warpedWidthMM);
    }
    private int pixelToMM(double pixel) {
        return (int) (pixel * warpedWidthMM / warpedImageSize.width);
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
        Imgproc.cvtColor(warped, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        if (detectBlue) {
            Scalar lowerBlue = new Scalar(80, 100, 10);
            Scalar upperBlue = new Scalar(140, 255, 255);
            Core.inRange(hsv, lowerBlue, upperBlue, mask);
        } else {
            Scalar lowerRed1 = new Scalar(0, 30, 100);
            Scalar upperRed1 = new Scalar(15, 255, 255);
            Scalar lowerRed2 = new Scalar(160, 30, 100);
            Scalar upperRed2 = new Scalar(180, 255, 255);
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsv, lowerRed1, upperRed1, mask1);
            Core.inRange(hsv, lowerRed2, upperRed2, mask2);
            Core.add(mask1, mask2, mask);
        }

        // Erode then dilate to reduce noise.
        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 3);

        // Create segmented image based on mask.
        Mat segmented = new Mat();
        Core.bitwise_and(warped, warped, segmented, mask);

        // Convert the Mat to input tensor.
        float[][][][] inputTensor =
            new float[1][modelImageWidth][modelImageHeight][3];
        for (int i = 0; i < warpedImageSize.width; i++) {
            for (int j = 0; j < warpedImageSize.height; j++) {
                double[] pixel = warped.get(j, i); // returns [R, G, B]
                // Normalize pixel values to [0, 1]
                inputTensor[0][i][j][0] = (float) (pixel[0] / 255.0);
                inputTensor[0][i][j][1] = (float) (pixel[1] / 255.0);
                inputTensor[0][i][j][2] = (float) (pixel[2] / 255.0);
            }
        }

        // Run the detector inference.
        float[][][] output = detector.runInference(inputTensor);
        int numGridCells = output[0][0].length;

        // Post-process the output tensor to extract oriented bounding boxes.
        List<List<Point>> topRectangles = new ArrayList<>();
        List<RotatedRect> detections = new ArrayList<>();

        // Used to dedupe overlapped detections
        List<Point> centers = new ArrayList<>();
        List<Float> confs = new ArrayList<>();

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

            // telemetry.addData("Raw", "%f %f %f %f %f %f %f %f",
            //     cx, cy, w, h, angle, c1, c2, c3);

            if (w < 1 || h < 1) continue; // Skip zero size to avoid crash

            float confidence = Math.max(c1, Math.max(c2, c3));
            if (confidence < 0.2) continue; // Skip low-confidence detections

            // Create a rotated rectangle using the detection outputs.
            Point center = new Point(cx, cy);
            Size rectSize = new Size(w, h);
            RotatedRect rRect = new RotatedRect(center, rectSize, angle);

            // Obtain the four vertices of the rotated rectangle.
            Point[] vertices = new Point[4];
            rRect.points(vertices);

            // Dedupe detections
            boolean isNew = true;
            for (int j = 0; j < centers.size(); j++) {
                double dist = distance(cx, cy, centers.get(j).x, centers.get(j).y);
                if (dist > sampleWidth / 2) {
                    continue;
                }
                isNew = false;
                if (confidence > confs.get(j)) {
                    centers.set(j, center);
                    confs.set(j, confidence);
                }
                break;
            }
            if (!isNew) {
                continue;
            }
            centers.add(center);
            confs.add(confidence);

            double ratio1 = maskedRatio(new MatOfPoint(vertices), mask);
            double ratio2 = whRatio(w, h);
            // telemetry.addData("High Conf", "%f %f %f %f %f %f %f %f",
            //     cx, cy, w, h, angle, confidence, ratio1, ratio2);

            // Filter out detections whose filled mask ratio is below threshold
            if (ratio1 < 0.8)
                continue;
            
            // Filter out detections by width/height ratio
            if (!(ratio2 > 2.0 && ratio2 < 2.6))
                continue;

            topRectangles.add(Arrays.asList(vertices[0], vertices[1], vertices[2], vertices[3]));

            // Calculate bottom center
            Point bottomCenter = sampleTopVertexToBottom(center);
            Point bottomCenterReal = new Point(
                pixelToMM(bottomCenter.x - cameraX),
                pixelToMM(bottomCenter.y - cameraY));
            RotatedRect bottomRect = new RotatedRect(bottomCenterReal, rectSize, angle);
            detections.add(bottomRect);
            telemetry.addData("Detected", "%d %d %d %d",
                (int) bottomCenterReal.x, (int) bottomCenterReal.y,
                (int) angle, (int) (confidence * 100));
        }

        detections.sort(Comparator.comparingDouble(
            detection -> -confs.get(detections.indexOf(detection))));
        this.detections = detections;

        // Calculate sample bottom rectangles.
        for (List<Point> topRect : topRectangles) {
            MatOfPoint topPts = new MatOfPoint();
            topPts.fromList(topRect);
            Imgproc.polylines(warped, Collections.singletonList(topPts), true, new Scalar(0, 255, 0), 2);
            List<Point> bottomRect = new ArrayList<>();
            for (Point p : topRect)
                bottomRect.add(sampleTopVertexToBottom(p));
            MatOfPoint bottomPts = new MatOfPoint();
            bottomPts.fromList(bottomRect);
            Imgproc.polylines(warped, Collections.singletonList(bottomPts), true, new Scalar(255, 255, 0), 2);
        }

        // Send telemetry data for debugging.
        telemetry.addData("Detections", topRectangles.size());
        telemetry.update();

        // Return the final annotated image.
        return warped;
    }

    private static double maskedRatio(MatOfPoint box, Mat mask) {
        // Create a blank mask with the same size and type as 'mask'
        Mat boxMask = Mat.zeros(mask.size(), mask.type());
        
        // Fill the polygon defined by 'box' with white (255)
        List<MatOfPoint> pts = new ArrayList<>();
        pts.add(box);
        Imgproc.fillPoly(boxMask, pts, new Scalar(255));
        
        // Compute the intersection between the provided mask and the box mask
        Mat intersection = new Mat();
        Core.bitwise_and(mask, boxMask, intersection);
        
        int boxArea = Core.countNonZero(boxMask);
        int intersectionArea = Core.countNonZero(intersection);
        
        return (double) intersectionArea / boxArea;
    }

    private static double whRatio(double w, double h) {
        double ratio = w / h;
        if (ratio < 1.0) {
            ratio = 1.0 / ratio;
        }
        return ratio;
    }

    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    public class YoloV8TFLiteDetector {
        private Interpreter tflite;
    
        // Constructor: load the model from the assets folder.
        public YoloV8TFLiteDetector(Context context) {
            try {
                tflite = new Interpreter(loadModelFile(context, "yolov8_obb_v3.tflite"));
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
