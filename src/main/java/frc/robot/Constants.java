// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
//import org.lasarobotics.led.LEDStrip;
import org.lasarobotics.utils.PIDConstants;

import com.revrobotics.SparkMax;
import com.revrobotics.SparkBase.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Units;
import frc.robot.subsystems.drive.PurplePathPose;
import frc.robot.subsystems.vision.AprilTagCamera.Resolution;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Field {
    public static final double FIELD_WIDTH = 8.21;
    public static final double FIELD_LENGTH = 16.54;

    public static final Translation2d CENTER = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
    public static final Pair<Integer, Translation2d> BLUE_SPEAKER = new Pair<Integer, Translation2d>(7,
        new Translation2d(0.00, 5.55));
    public static final Pair<Integer, Translation2d> RED_SPEAKER = new Pair<Integer, Translation2d>(4,
        new Translation2d(15.64, 5.55));

    public static final PurplePathPose AMP = new PurplePathPose(
        new Pose2d(Units.Meters.of(1.85), Units.Meters.of(7.77), Rotation2d.fromDegrees(-90.0)),
        new Pose2d(Units.Meters.of(14.66), Units.Meters.of(7.77), Rotation2d.fromDegrees(-90.0)),
        Units.Meters.of(0.5),
        true);
    public static final PurplePathPose SOURCE = new PurplePathPose(
        new Pose2d(Units.Meters.of(15.48), Units.Meters.of(0.84), Rotation2d.fromDegrees(+120.00)),
        new Pose2d(Units.Meters.of(1.07), Units.Meters.of(0.82), Rotation2d.fromDegrees(+60.0)),
        Units.Meters.of(0.5),
        true);
  }

  /**
   * Constants for controllers
   * {@link HID#PRIMARY_CONTROLLER_PORT} is for the driver,
   * <br> <br>
   * {@link HID#SECONDARY_CONTROLLER_PORT} is for the operator
   */
  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.1;
  }

  public static class Drive {
    public static final PIDConstants DRIVE_ROTATE_PID = new PIDConstants(6.45, 0.0, 0.45, 0.0, 0.0);
    public static final double DRIVE_SLIP_RATIO = 0.05;
    public static final double DRIVE_TURN_SCALAR = 60.0;
    public static final double DRIVE_LOOKAHEAD = 6;

    
    public static final ControlCentricity DRIVE_CONTROL_CENTRICITY = ControlCentricity.FIELD_CENTRIC;

    private static final double[] DRIVE_THROTTLE_INPUT_CURVE_X = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };

    private static final double[] DRIVE_THROTTLE_INPUT_CURVE_Y = { 0.0, 0.052, 0.207, 0.465, 0.827, 1.293, 1.862, 2.534, 3.310, 4.189, 5.172 };
    
    private static final double[] DRIVE_TURN_INPUT_CURVE_X = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    
    private static final double[] DRIVE_TURN_INPUT_CURVE_Y = { 0.0, 0.010, 0.050, 0.100, 0.150, 0.200, 0.250, 0.300, 0.400, 0.600, 1.0 };

    
    
    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR
        .interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR
        .interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

    public static final MAXSwerveModule.GearRatio GEAR_RATIO = MAXSwerveModule.GearRatio.L3;
  }

  /** Pivot Constants */
  public static final class PivotConstants {
    // Id's for the pivot motors
    public static final int pivotRightCANID = 9;
    public static final int pivotLeftCANID = 10;

    // Controller idle mode and current
    public static final IdleMode pivotIdleMode = IdleMode.kBrake;
    public static final int pivotCurrentLimit = 60;
    public static final boolean pivotRightMotorInverted = true;

    // TODO: Tune these values for the pivot please
    /** The Proportional Gain for the pivot */
    public static final double kPivotP = 3.9;
    /** The Integral Gain for the pivot */
    public static final double kPivotI = 0.0001;
    /** The Derivative Gain for the pivot */
    public static final double kPivotD = 0.5;
    public static final boolean kPivotPositionPIDWrappingEnabled = false;
    public static final double kPivotMinOutput = -1.0;
    public static final double kPivotMaxOutput = 1.0;

    public static final double kPivotSpeed = 0.5;
    public static final double kPivotUpSpeed = 1.5;
    public static final double kPivotDownSpeed = -0.5;
    public static final double kPivotNoSpeed = 0.09; //Not zero to stop the pivot from falling due to gravity

    // Range of values the smart controller will use as a hard stop
    public static final float kPivotForwardSoftLimit = 0.31f;
    public static final float kPivotReverseSoftLimit = 0.0f;

    /** The direction the pivot is currently moving */
    public enum PivotDirection {
      UP,
      DOWN,
      STOP
    }
  }

  /** Constants for the Swerve Modules */
  public static class DriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 5);
    public static final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 6);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 7);
    public static final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 8);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 3);
    public static final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 4);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 1);
    public static final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 2);
    //public static final LEDStrip.ID LED_STRIP_ID = new LEDStrip.ID("DriveHardware/LEDStrip", 0, 200);
  }

  /** Constants for Vision (Unused 2024)*/
  public static class VisionHardware {
    public static final String CAMERA_A_NAME = "Arducam_OV9782_USB_Camera_A";
    public static final Transform3d CAMERA_A_LOCATION = new Transform3d(
        new Translation3d(0.381, 0.133, 0.102),
        new Rotation3d(0.0, Math.toRadians(-20.0), 0.0));
    public static final Resolution CAMERA_A_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_A_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_B_NAME = "Arducam_OV9782_USB_Camera_B";
    public static final Transform3d CAMERA_B_LOCATION = new Transform3d(
        new Translation3d(0.148, 0.2667, 0.47),
        new Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(+180.0)));
    public static final Resolution CAMERA_B_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_B_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_OBJECT_NAME = "Arducam_OV9782_USB_Camera_C";
    public static final Transform3d CAMERA_OBJECT_LOCATION = new Transform3d(
        new Translation3d(0.3, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(+15.0), 0));
    public static final Resolution CAMERA_OBJECT_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_OBJECT_FOV = Rotation2d.fromDegrees(79.7);
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }

  /** Intake Constants */
  public static final class IntakeConstants {
    public static final int kIntakeCANID = 15;
    public static final int kIntakeCurrentLimit = 60;
    public static final boolean kIntakeMotorInverted = true;
    public static final double kIntakeSpeed = 1;
    public static final double kOuttakeSpeed = -1;
    public static final IdleMode kIntakeIdleMode = IdleMode.kCoast;

    public enum IntakeState {
      IDLE,
      INTAKE,
      OUTTAKE,
    }
  }

  /** Feeder Constants */
  public static final class FeederConstants {
    public static final int kFeederCANID = 11;
    public static final int kFeederCurrentLimit = 60;
    public static final boolean kFeederMotorInverted = true;
    public static final double kFeederSpeed = 1;
    public static final int kFeederSensorPort = 0;
  }

  /** Shooter Constants */
  public static final class ShooterConstants {
    public static final int kKickerMotorCANID = 12;
    public static final int kShooterRightCANID = 13;
    public static final int kShooterLeftCANID = 14;

    public static final int kKickerMotorCurrentLimit = 60;
    public static final int kShooterMotorCurrentLimit = 60;
    public static final CANSparkBase.IdleMode kShooterMotorIdleMode = IdleMode.kCoast;
    public static final CANSparkBase.IdleMode kKickerMotorIdleMode = IdleMode.kCoast;

    public static final boolean kKickerMotorInvert = false;
    public static final boolean kShooterLeftMotorInverted = true;
    public static final boolean kShooterRightMotorInverted = false;

    public static final double kShooterReverseSpeed = -0.1;
    public static final double kShooterIdleSpeed = 0.1;


    public static final double PVal = 0.001;

    public static final double REL_ENC_CONVERSION = 2.0;

    public enum ShooterState {
      IDLE,
      INTAKE,
      OUTTAKE,
      REVERSE,
      SPEAKER,
      AMP
    }
  }
}