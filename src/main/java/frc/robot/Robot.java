// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Drivetrain.getInstance();
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new DriveWithJoystick());
    Thread.currentThread().setPriority(2);

    new Thread(() -> {
      Thread.currentThread().setPriority(1);

      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(352, 240);

      CvSink cvSink = CameraServer.getVideo();
      CvSource blurStream = CameraServer.putVideo("Blur", 352, 240);
      CvSource rgbThreshStream = CameraServer.putVideo("RGB Threshold", 352, 240);
      CvSource hsvThreshStream = CameraServer.putVideo("HSV Threshold", 352, 240);
      CvSource hslThreshStream = CameraServer.putVideo("HSL Threshold", 352, 240);
      CvSource threshOrStream = CameraServer.putVideo("Threshold Bitwise AND", 352, 240);
      CvSource outputStream = CameraServer.putVideo("Output", 352, 240);

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

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh_and, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Mat drawing = Mat.zeros(thresh_and.size(), CvType.CV_8UC3);
        MatOfPoint biggestContour = contours.get(0);

        for (int i = 0; i<contours.size(); i++) {
          if (contours.get(i).width() * contours.get(i).height() > biggestContour.width() * biggestContour.height()) {
            biggestContour = contours.get(i);
          }
          Scalar color = new Scalar(0, 200, 0);
          Imgproc.drawContours(drawing, contours, i, color, 2, 2, hierarchy, 0, new Point());
        }
        Point[] contourPoints = biggestContour.toArray();
        MatOfPoint2f poly = new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contourPoints), poly, 3, true);
        Rect rect = Imgproc.boundingRect(new MatOfPoint(poly));
        SmartDashboard.putNumber("BiggestContourCenterX", rect.x);
        SmartDashboard.putNumber("BiggestContourCenterY", rect.y);
        SmartDashboard.putNumber("BiggestContourArea", biggestContour.width() * biggestContour.height());
        blurStream.putFrame(blur);
        rgbThreshStream.putFrame(rgb_thresh);
        hsvThreshStream.putFrame(hsv_thresh);
        hslThreshStream.putFrame(hsl_thresh);
        threshOrStream.putFrame(thresh_and);
        outputStream.putFrame(drawing);
      }
    }).start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Drivetrain.getInstance().logToSmartDashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
//    m_autonomousCommand = new AUTONCOMMAND();
//    if (m_autonomousCommand != null) {
//      m_autonomousCommand.schedule();
//    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
//    if (m_autonomousCommand != null) {
//      m_autonomousCommand.cancel();
//    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
//    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}