package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.Imgproc;
import java.util.*;

public class SampleDetectionPipeline extends OpenCvPipeline {
    // Image sizes
    private final Size imageSize = new Size(640, 360);
    private final Size warpedImageSize = new Size(300, 465);

    // Perspective transformation source and destination points
    private final MatOfPoint2f imagePoints = new MatOfPoint2f(
            new Point(178, 35),    // Top-left
            new Point(400, 15),    // Top-right
            new Point(573, 99),    // Bottom-right
            new Point(139, 160)    // Bottom-left
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
    private final int warpedWidthMM = 700;
    private final int sampleWidth;   // mm_to_pixel(38)
    private final int sampleHeight;  // mm_to_pixel(88)
    private final int sampleDepth;   // mm_to_pixel(38)

    // Camera 3D position in pixel units
    private final int cameraX;
    private final int cameraY;
    private final int cameraZ;       // height component

    // (Optional) Telemetry if you want to output values to the driver station
    private Telemetry telemetry;

    public SampleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
        
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
        cameraX = mmToPixel(155);
        cameraY = mmToPixel(1165);
        cameraZ = mmToPixel(270);
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

        // Find contours on the mask.
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Mat contoursImage = warped.clone();
        Imgproc.drawContours(contoursImage, contours, -1, new Scalar(0, 255, 0), 2);

        // Detect sample top rectangles from contours.
        Mat polyImage = warped.clone();
        List<List<Point>> topRectangles = new ArrayList<>();

        for (MatOfPoint cnt : contours) {
            double area = Imgproc.contourArea(cnt);
            if (area < sampleWidth * sampleHeight * 0.9)
                continue;

            double arcLen = Imgproc.arcLength(new MatOfPoint2f(cnt.toArray()), true);
            double epsilon = 0.03 * arcLen;
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(cnt.toArray()), approx, epsilon, true);
            List<Point> approxList = approx.toList();

            // Get sorted y-coordinates
            List<Double> yCoords = new ArrayList<>();
            for (Point p : approxList)
                yCoords.add(p.y);
            Collections.sort(yCoords);
            // Grab the two highest (largest y) values
            Set<Double> higherYCoordinates = new HashSet<>();
            if (yCoords.size() >= 2) {
                higherYCoordinates.add(yCoords.get(yCoords.size() - 1));
                higherYCoordinates.add(yCoords.get(yCoords.size() - 2));
            }

            List<Point> topRectangle = null;

            // Iterate over polygon vertices
            for (int i = 0; i < approxList.size(); i++) {
                Point pt0 = approxList.get((i - 1 + approxList.size()) % approxList.size());
                Point pt1 = approxList.get(i);
                Point pt2 = approxList.get((i + 1) % approxList.size());

                // For visualization: draw each segment with a random color.
                Scalar randomColor = new Scalar(Math.random() * 255, Math.random() * 255, Math.random() * 255);
                Imgproc.line(polyImage, pt1, pt2, randomColor, 2);

                double angle = angleBetween(pt0, pt1, pt2);
                if (angle < 75 || angle > 105)
                    continue;

                double side1 = distance(pt0, pt1);
                double side2 = distance(pt1, pt2);
                double minSide = Math.min(side1, side2);
                double maxSide = Math.max(side1, side2);
                boolean useMinSide = false, useMaxSide = false;
                if (sampleWidth * 0.8 < minSide && minSide < sampleWidth * 1.4 && maxSide > sampleHeight * 0.9)
                    useMinSide = true;
                if (maxSide > sampleHeight * 0.9 && maxSide < sampleHeight * 1.2 && minSide > sampleWidth * 0.9)
                    useMaxSide = true;
                if (!useMinSide && !useMaxSide)
                    continue;

                // Mark vertex for visualization.
                Imgproc.circle(polyImage, pt1, 5, new Scalar(0, 255, (useMaxSide ? 255 : 0)), 2);

                // Adjust the points to “cut” the longer side.
                if (useMinSide) {
                    if (side1 == minSide)
                        pt2 = pointOnLine(pt1, pt2, sampleHeight);
                    else
                        pt0 = pointOnLine(pt1, pt0, sampleHeight);
                } else {
                    if (side1 == maxSide)
                        pt2 = pointOnLine(pt1, pt2, sampleWidth);
                    else
                        pt0 = pointOnLine(pt1, pt0, sampleWidth);
                }
                Point pt3 = new Point(pt0.x + pt2.x - pt1.x, pt0.y + pt2.y - pt1.y);
                List<Point> rect = Arrays.asList(pt0, pt1, pt2, pt3);

                // Check if most of the rectangle area is covered in the mask.
                Mat rectMask = Mat.zeros(mask.size(), CvType.CV_8UC1);
                MatOfPoint rectMat = new MatOfPoint();
                rectMat.fromList(rect);
                Imgproc.fillPoly(rectMask, Collections.singletonList(rectMat), new Scalar(255));
                int totalPixels = Core.countNonZero(rectMask);
                Mat maskedRect = new Mat();
                Core.bitwise_and(mask, mask, maskedRect, rectMask);
                int maskedPixels = Core.countNonZero(maskedRect);
                if (totalPixels == 0 || ((double) maskedPixels / totalPixels) < 0.9)
                    continue;

                if (higherYCoordinates.contains(pt1.y))
                    continue;
                if (topRectangle != null && pt1.y > topRectangle.get(1).y)
                    continue;

                topRectangle = rect;
            }
            if (topRectangle != null)
                topRectangles.add(topRectangle);
        }

        // Calculate sample bottom rectangles and annotate the image.
        Mat detected = warped.clone();
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

            // Calculate the centroid.
            Point centroid = new Point(0, 0);
            for (Point p : bottomRect) {
                centroid.x += p.x;
                centroid.y += p.y;
            }
            centroid.x /= 4;
            centroid.y /= 4;
            // Annotate with converted coordinates (mm)
            int centroidXMM = pixelToMM(centroid.x);
            int centroidYMM = pixelToMM(centroid.y);
            Imgproc.putText(detected, "[" + centroidXMM + ", " + centroidYMM + "]", centroid,
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 0), 2);
        }

        // (Optional) send telemetry data for debugging.
        if (telemetry != null) {
            telemetry.addData("Detected Top Rectangles", topRectangles.size());
            telemetry.update();
        }

        // Return the final annotated image.
        return detected;
    }
}
