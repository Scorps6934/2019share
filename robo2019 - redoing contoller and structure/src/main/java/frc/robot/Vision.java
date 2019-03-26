package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class Vision {
    //TODO: add blur and lower exposure
    public UsbCamera camera;
    public CvSink cvSink;
    public CvSource outputStream;
    public CvSource colorStream;
    public Mat source;
    public Mat output;
    public List<MatOfPoint> contours;

    public Vision(String name){
        camera = CameraServer.getInstance().startAutomaticCapture();
        cvSink = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo(name, 360, 360);
        colorStream = CameraServer.getInstance().putVideo(name+": Colorful!", 360, 360);
        source = new Mat();
        output = new Mat();
        contours = new ArrayList<MatOfPoint>();
    }

    public void setupCameraSettings(){
        camera.setResolution(360, 360);
        camera.setExposureManual(RobotMap.cameraExposure);
    }

    public void setupThresholding(){
        cvSink.grabFrame(source, 30);

        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(output, output, 0, 255, Imgproc.THRESH_OTSU);
    }
    public void setupContours(){
        // Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);
        
        Mat hierarchy = new Mat();
        Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);
    }

    public void filterAndDrawContours(){
        for (int i = 0; i < contours.size(); i++) {
            double contourArea = Imgproc.contourArea(contours.get(i));
            Rect boundRect = Imgproc.boundingRect(contours.get(i));
            double ratio = contourArea / (boundRect.width * boundRect.height); // solidity ratio
    
            // double ratio = (double)boundRect.width/boundRect.height; //if not using
            // aspect ratio can move contourArea definition to ration
            // System.out.println();
            if ((ratio < RobotMap.contourMinRatio || ratio > RobotMap.contourMaxRatio)
                && (contourArea < RobotMap.contourMinArea || contourArea > RobotMap.contourMaxArea)) {
                contours.remove(i);
                i--;
            continue;
            }
    
            // SmartDashboard.putNumber("dab", ratio);
            // System.out.println(i+" "+ ratio);
            // Imgproc.drawMarker(output, boundRect.br(), new Scalar(0,0,255));
            // Imgproc.drawMarker(source, boundRect.br(), new Scalar(0,0,255));
            Imgproc.rectangle(output, boundRect.br(), boundRect.tl(), new Scalar(0, 0, 255), 10);
            Imgproc.rectangle(source, boundRect.br(), boundRect.tl(), new Scalar(0, 0, 255), 10);
            Imgproc.drawContours(output, contours, i, new Scalar(255, 0, 0), 10);
            Imgproc.drawContours(source, contours, i, new Scalar(255, 0, 0), 10);
        }
        Imgproc.drawMarker(source, findCenter(), new Scalar(0,255,0)); //to draw may need different method
        Imgproc.drawMarker(output, findCenter(), new Scalar(0,255,0));
        colorStream.putFrame(source);
        outputStream.putFrame(output);
    }

    public Point findCenter(){
        Point center = null;
        Mat corners = new Mat();
        if(contours.get(0) != null){
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(0).toArray()));
            Imgproc.boxPoints(rotatedRect, corners);
            double[][] cornerz = {corners.get(0,0), corners.get(0,1), corners.get(0,2), corners.get(0,3)};
            insertionSortCorners(cornerz);
            
            center=new Point(((cornerz[0][0]+cornerz[1][0])/2),((cornerz[0][1]+cornerz[1][1])/2)); //midpoint
        }
        else {
            System.out.println("No contours found");
        }
        return center;
    }

    //tHIS wiLL WorK GuYS
    /**
     * insertion sorts in ascending order by the y value of the corners
     */
    public void insertionSortCorners(double[][] corners) 
    { 
       //int i, key, j; 
       for (int i = 1; i < 4; i++) 
       { 
          // corners = arr[i]; 
          int j = i-1; 
      
           while ( j >= 0 && corners[0][j] > corners[0][i]) 
           { 
                corners[j+1] = corners[j];
                j--; 
           } 
           
           corners[j+1] = corners[i];
       } 
    } 
   /* public double[] getAlignmentData(){
        
    }
    */
    public double calculateAngleToTurn(Point center){
        double centerOfImageX = (RobotMap.imageWidth/2.0)-0.5;
        double focalLength = (RobotMap.imageWidth/2*(Math.tan(RobotMap.FOV/2)));
        double degreesToChangeX = Math.atan((center.x - centerOfImageX) / focalLength);
        return degreesToChangeX;
    }
    //see Joey's drawings a=depth, b=horizontal
    public static double calculateHypotenuseC(Point center){
        double centerOfImageY = (RobotMap.imageHeight/2.0)-0.5;
        double focalLength = (RobotMap.imageHeight/2*(Math.tan(RobotMap.FOV/2)));
        double degreesToChangeY = Math.atan((center.y - centerOfImageY) / focalLength);

        double cameraAngleToTape = RobotMap.cameraAngle+degreesToChangeY;
        double hypotenuseC= RobotMap.cameraHeight*Math.tan(cameraAngleToTape);
        return hypotenuseC;
    }
    public static double calculateDepthDistance(double degreesToChangeX, double hypotenuseC){
        double depthDistance= hypotenuseC*Math.sin(degreesToChangeX);
        return depthDistance;
    }
    public static double calculateHorizontalDistance(double degreesToChangeX, double hypotenuseC){
        double horiontalDistance=hypotenuseC*Math.cos(degreesToChangeX);
        return horiontalDistance;
    }

}