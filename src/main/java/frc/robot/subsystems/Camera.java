package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    private static volatile Camera instance;

    private boolean processFrame;

    /**
     * The getter for the Camera class.
     * @return the singleton Camera object.
     */
    public static Camera getInstance() {
        if (instance == null) {
            synchronized (Camera.class) {
                if (instance == null) {
                    instance = new Camera();
                }
            }
        }
        return instance;
    }

    private Camera() {
      startStream();
    }

    public void processFrame() {
      processFrame = true;
    }

    public void startStream() {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(300, 300);
    }

    public void startProcessingStream() {
      new Thread(() -> {
        Thread.currentThread().setPriority(1);
  
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(300, 300);
  
        CvSink cvSink = CameraServer.getVideo();
        CvSource rgbThreshStream = CameraServer.putVideo("RGB Threshold", 300, 300);
        CvSource hsvThreshStream = CameraServer.putVideo("HSV Threshold", 300, 300);
        CvSource hslThreshStream = CameraServer.putVideo("HSL Threshold", 300, 300);
  
        Mat source = new Mat();
        Mat blur = new Mat();
        Mat rgb = new Mat();
        Mat hsv = new Mat();
        Mat hsl = new Mat();
        Mat rgb_thresh = new Mat();
        Mat hsv_thresh = new Mat();
        Mat hsl_thresh = new Mat();
        Mat rgb_hsv_and = new Mat();
        Mat thresh_and = new Mat();
  
  
          SmartDashboard.putNumber("RGB_Lower_r", 195);
          SmartDashboard.putNumber("RGB_Lower_g", 171);
          SmartDashboard.putNumber("RGB_Lower_b", 0);
  
          SmartDashboard.putNumber("RGB_Upper_r", 255);
          SmartDashboard.putNumber("RGB_Upper_g", 255);
          SmartDashboard.putNumber("RGB_Upper_b", 255);
  
          SmartDashboard.putNumber("HSV_Lower_r", 0);
          SmartDashboard.putNumber("HSV_Lower_g", 0);
          SmartDashboard.putNumber("HSV_Lower_b", 0);
  
          SmartDashboard.putNumber("HSV_Upper_r", 255);
          SmartDashboard.putNumber("HSV_Upper_g", 58);
          SmartDashboard.putNumber("HSV_Upper_b", 255);
  
          SmartDashboard.putNumber("HSL_Lower_r", 0);
          SmartDashboard.putNumber("HSL_Lower_g", 212);
          SmartDashboard.putNumber("HSL_Lower_b", 0);
  
          SmartDashboard.putNumber("HSL_Upper_r", 188);
          SmartDashboard.putNumber("HSL_Upper_g", 255);
          SmartDashboard.putNumber("HSL_Upper_b", 255);
  
        while(!Thread.interrupted()) {
          if (cvSink.grabFrame(source) == 0) {
            continue;
          }
          Imgproc.GaussianBlur(source, blur, new Size(3.0, 3.0), 3, 3.0);
          Imgproc.cvtColor(blur, rgb, Imgproc.COLOR_BGR2RGB);
          Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_BGR2HSV);
          Imgproc.cvtColor(blur, hsl, Imgproc.COLOR_BGR2HLS);
          Core.inRange(rgb, new Scalar(SmartDashboard.getNumber("RGB_Lower_r", 0), SmartDashboard.getNumber("RGB_Lower_g", 0), SmartDashboard.getNumber("RGB_Lower_b", 0)), new Scalar(SmartDashboard.getNumber("RGB_Upper_r", 255), SmartDashboard.getNumber("RGB_Upper_g", 255), SmartDashboard.getNumber("RGB_Upper_b", 255)), rgb_thresh);
          Core.inRange(hsv, new Scalar(SmartDashboard.getNumber("HSV_Lower_r", 0), SmartDashboard.getNumber("HSV_Lower_g", 0), SmartDashboard.getNumber("HSV_Lower_b", 0)), new Scalar(SmartDashboard.getNumber("HSV_Upper_r", 255), SmartDashboard.getNumber("HSV_Upper_g", 255), SmartDashboard.getNumber("HSV_Upper_b", 255)), hsv_thresh);
          Core.inRange(hsl, new Scalar(SmartDashboard.getNumber("HSL_Lower_r", 0), SmartDashboard.getNumber("HSL_Lower_g", 0), SmartDashboard.getNumber("HSL_Lower_b", 0)), new Scalar(SmartDashboard.getNumber("HSL_Upper_r", 255), SmartDashboard.getNumber("HSL_Upper_g", 255), SmartDashboard.getNumber("HSL_Upper_b", 255)), hsl_thresh);
          Core.bitwise_and(rgb_thresh, hsv_thresh, rgb_hsv_and);
          Core.bitwise_and(rgb_hsv_and, hsl_thresh, thresh_and);
  
          rgbThreshStream.putFrame(rgb_thresh);
          hsvThreshStream.putFrame(hsv_thresh);
          hslThreshStream.putFrame(hsl_thresh);
        }
      }).start();
    }
}