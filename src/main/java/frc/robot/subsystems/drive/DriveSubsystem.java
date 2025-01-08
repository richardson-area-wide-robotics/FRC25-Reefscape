// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.Getter;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.swerve.child.MAXSwerveModule;
import org.lasarobotics.drive.swerve.parent.REVSwerveModule;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.drive.RotatePIDController;
import org.lasarobotics.drive.ThrottleMap;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.led.LEDStrip.Pattern;
import org.lasarobotics.led.LEDSubsystem;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;      // For handling rotations
import edu.wpi.first.math.geometry.Translation2d;    // For handling 2D translations
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // For dashboard integration
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.DimensionlessUnit;
import org.lasarobotics.drive.swerve.DriveWheel;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    NavX2 navx;
    REVSwerveModule lFrontModule;
    REVSwerveModule rFrontModule;
    REVSwerveModule lRearModule;
    REVSwerveModule rRearModule;

    public Hardware(NavX2 navx,
          REVSwerveModule lFrontModule,
          REVSwerveModule rFrontModule,
          REVSwerveModule lRearModule,
          REVSwerveModule rRearModule) {
      this.navx = navx;
      this.lFrontModule = lFrontModule;
      this.rFrontModule = rFrontModule;
      this.lRearModule = lRearModule;
      this.rRearModule = rRearModule;
    }
  }

  // Drive specs
  public static final double DRIVE_WHEELBASE = 0.5588;
  public static final double DRIVE_TRACK_WIDTH = 0.5588;
  public static final double AUTO_LOCK_TIME = 3.0;
  public static final Time MAX_SLIPPING_TIME = Units.Seconds.of(1.2);
  public static final double DRIVE_CURRENT_LIMIT = 60.0;
  public static final AngularVelocity DRIVE_ROTATE_VELOCITY = Units.RadiansPerSecond.of(12 * Math.PI);
  public static final AngularVelocity AIM_VELOCITY_THRESHOLD = Units.DegreesPerSecond.of(5.0);
  public static final AngularAcceleration DRIVE_ROTATE_ACCELERATION = Units.RadiansPerSecond.of(4 * Math.PI).per(Units.Second);
  public static final Translation2d AIM_OFFSET = new Translation2d(0.0, -0.5);
  public final LinearVelocity DRIVE_MAX_LINEAR_SPEED;
  public final LinearAcceleration DRIVE_AUTO_ACCELERATION;
  private final AdvancedSwerveKinematics advancedKinematics = new AdvancedSwerveKinematics();
  private final ProfiledPIDController autoAimPIDControllerFront;
  private final ProfiledPIDController autoAimPIDControllerBack;
  private final MedianFilter xVelocityFilter;
  private final MedianFilter yVelocityFilter;

  // Other settings
  private static final double TIP_THRESHOLD = 35.0;
  private static final double BALANCED_THRESHOLD = 10.0;
  private static final double AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR = 0.1;
  private static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.03, 0.03, Math.toRadians(1.0));
  private static final Matrix<N3, N1> VISION_STDDEV = VecBuilder.fill(1.0, 1.0, Math.toRadians(3.0));
    // Log
  private static final String POSE_LOG_ENTRY = "/Pose";
  private static final String ACTUAL_SWERVE_STATE_LOG_ENTRY = "/ActualSwerveState";
  private static final String DESIRED_SWERVE_STATE_LOG_ENTRY = "/DesiredSwerveState";


  private final ThrottleMap throttleMap;
  private final RotatePIDController rotatePIDController;
  @Getter
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  public final NavX2 navx;
  private final REVSwerveModule lFrontModule;
  private final REVSwerveModule rFrontModule;
  private final REVSwerveModule lRearModule;
  private final REVSwerveModule rRearModule;


  private ControlCentricity controlCentricity;
  private ChassisSpeeds desiredChassisSpeeds;
  private boolean isTractionControlEnabled = true;
  private Pose2d m_previousPose;
  private Rotation2d currentHeading;
  private final Field2d field;

  public final Command ANTI_TIP_COMMAND = new FunctionalCommand(
    () -> LEDSubsystem.getInstance().startOverride(Pattern.RED_STROBE),
          this::antiTip,
    (interrupted) -> {
      resetRotatePID();
      stop();
      lock();
      LEDSubsystem.getInstance().endOverride();
    },
    this::isBalanced,
    this
  );

  /**
 * DriveSubsystem constructor for managing a swerve drivetrain.
 *
 * @param drivetrainHardware Hardware devices required by drivetrain.
 * @param pidf PID constants for the drive system.
 * @param controlCentricity Control centricity configuration.
 * @param throttleInputCurve Spline function for throttle input.
 * @param turnInputCurve Spline function for turn input.
 * @param turnScalar Scalar for turn input (degrees).
 * @param deadband Deadband for controller input [+0.001, +0.2].
 * @param lookAhead Rotate PID lookahead, in number of loops.
 */
public DriveSubsystem(Hardware drivetrainHardware, PIDConstants pidf, ControlCentricity controlCentricity,
                      PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve,
                      Angle turnScalar, Dimensionless deadband, Time lookAhead) {

    // Initialize subsystem name
    setSubsystem(this.getClass().getSimpleName());

    // Drivetrain hardware setup
    this.navx = drivetrainHardware.navx;
    this.lFrontModule = drivetrainHardware.lFrontModule;
    this.rFrontModule = drivetrainHardware.rFrontModule;
    this.lRearModule = drivetrainHardware.lRearModule;
    this.rRearModule = drivetrainHardware.rRearModule;

    // Drivetrain constants
    DRIVE_MAX_LINEAR_SPEED = drivetrainHardware.lFrontModule.getMaxLinearVelocity();
    DRIVE_AUTO_ACCELERATION = DRIVE_MAX_LINEAR_SPEED
        .per(Units.Second)
        .minus(Units.MetersPerSecondPerSecond.of(1.0));

    // Input curves and controllers
    this.controlCentricity = controlCentricity;
    this.throttleMap = new ThrottleMap(throttleInputCurve, DRIVE_MAX_LINEAR_SPEED, deadband);
    this.rotatePIDController = new RotatePIDController(turnInputCurve, pidf, turnScalar, deadband, lookAhead);

    // Path follower configuration
    //this.pathFollowerConfig = new HolonomicPathFollowerConfig(
    //    new com.pathplanner.lib.util.PIDConstants(3.1, 0.0, 0.0),
    //    new com.pathplanner.lib.util.PIDConstants(5.0, 0.0, 0.1),
    //    DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond),
    //    lFrontModule.getModuleCoordinate().getNorm(),
    //    new ReplanningConfig(),
    //);

    // NavX calibration
    while (navx.isCalibrating()) stop();
    navx.reset();

    // Swerve drive kinematics and pose estimator
    kinematics = new SwerveDriveKinematics(
        lFrontModule.getModuleCoordinate(),
        rFrontModule.getModuleCoordinate(),
        lRearModule.getModuleCoordinate(),
        rRearModule.getModuleCoordinate()
    );

    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        ODOMETRY_STDDEV,
        VISION_STDDEV
    );

    // Chassis speeds
    desiredChassisSpeeds = new ChassisSpeeds();

    // Field2d visualization
    field = new Field2d();
    SmartDashboard.putData(field);

    // Path logging for PathPlanner
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        if (poses.isEmpty()) return;
        var trajectory = TrajectoryGenerator.generateTrajectory(
            poses,
            new TrajectoryConfig(DRIVE_MAX_LINEAR_SPEED, DRIVE_AUTO_ACCELERATION)
        );
        field.getObject("currentPath").setTrajectory(trajectory);
    });
}

/**
 * Initialize hardware devices for drive subsystem
 * @return Hardware object containing all necessary devices for this subsystem
 */
public static Hardware initializeHardware() {
  NavX2 navx = new NavX2(Constants.DriveHardware.NAVX_ID);

  REVSwerveModule lFrontModule = createSwerve(
          Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID,
          Constants.DriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID,
          SwerveModule.Location.LeftFront);


  // Inverted back left and front right motors
  REVSwerveModule rFrontModule = createSwerve(
          Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID,
          Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID,
          SwerveModule.Location.RightFront);


  REVSwerveModule lRearModule = createSwerve(
          Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID,
          Constants.DriveHardware.LEFT_REAR_ROTATE_MOTOR_ID,
          SwerveModule.Location.LeftRear);

  REVSwerveModule rRearModule = createSwerve(
          Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID,
          Constants.DriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID,
          SwerveModule.Location.RightRear);

    return new Hardware(navx, lFrontModule, rFrontModule, lRearModule, rRearModule);
}

  private static REVSwerveModule createSwerve(Spark.ID driveMotor, Spark.ID rotateMotor, SwerveModule.Location location){
    return REVSwerveModule.create(
          new REVSwerveModule.Hardware(
                  new Spark(driveMotor, Spark.MotorKind.NEO),
                  new Spark(rotateMotor, Spark.MotorKind.NEO_550)
          ),
          location,
          Constants.Drive.GEAR_RATIO,
          DriveWheel.create(null,null,null), // Replace with actual drive wheel configuration
          PIDConstants.of(1, 1, 1,1,1), // Replace with actual PID constants
          FFConstants.of(1,1,1,1),  // Replace with actual feed-forward constants
          PIDConstants.of(1, 1, 1,1,1), // Replace with actual PID constants
          FFConstants.of(1,1,1,1),  // Replace with actual feed-forward constants
          of(Constants.Drive.DRIVE_SLIP_RATIO),
          Mass.ofBaseUnits(100, Units.Pounds), // Replace with actual mass value
          Distance.ofRelativeUnits(DRIVE_WHEELBASE, Units.Meter),
          Distance.ofRelativeUnits(DRIVE_TRACK_WIDTH, Units.Meter),
          Time.ofRelativeUnits(AUTO_LOCK_TIME, Units.Second),
          Current.ofRelativeUnits(DRIVE_CURRENT_LIMIT, Units.Amp));
  }

  /**
   * Set swerve modules
   * @param moduleStates Array of calculated module states
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates) {
    lFrontModule.set(moduleStates);
    rFrontModule.set(moduleStates);
    lRearModule.set(moduleStates);
    rRearModule.set(moduleStates);
    Logger.recordOutput(getName() + DESIRED_SWERVE_STATE_LOG_ENTRY, moduleStates);
  }

  /**
   * Set swerve modules, automatically applying traction control
   * @param moduleStates Array of calculated module states
   * @param inertialVelocity Current inertial velocity
   * @param rotateRate Desired robot rotate rate
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates, LinearVelocity inertialVelocity, AngularVelocity rotateRate) {
    lFrontModule.set(moduleStates);
    rFrontModule.set(moduleStates);
    lRearModule.set(moduleStates);
    rRearModule.set(moduleStates);
    Logger.recordOutput(getName() + DESIRED_SWERVE_STATE_LOG_ENTRY, moduleStates);
  }

  /**
   * Drive robot and apply traction control
   * @param xRequest Desired X (forward) velocity
   * @param yRequest Desired Y (sideways) velocity
   * @param rotateRequest Desired rotate rate
   * @param inertialVelocity Current robot inertial velocity
   * @param controlCentricity Current robot rotate rate
   */
  private void drive(ControlCentricity controlCentricity,
                     LinearVelocity xRequest,
                     LinearVelocity yRequest,
                     AngularVelocity rotateRequest,
                     LinearVelocity inertialVelocity) {
    // Get requested chassis speeds, correcting for second order kinematics
    desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, rotateRequest)
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = advancedKinematics.toSwerveModuleStates(
            desiredChassisSpeeds,
      getPose().getRotation(),
      controlCentricity
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITH traction control
    setSwerveModules(moduleStates, inertialVelocity, Units.RadiansPerSecond.of(desiredChassisSpeeds.omegaRadiansPerSecond));
  }

  /**
   * Drive robot without traction control
   *
   * @param xRequest      Desired X (forward) velocity
   * @param yRequest      Desired Y (sideways) velocity
   * @param rotateRequest Desired rotate rate
   */
  private void drive(LinearVelocity xRequest,
                     LinearVelocity yRequest,
                     AngularVelocity rotateRequest) {
    // Get requested chassis speeds, correcting for second order kinematics
    desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, rotateRequest)
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = advancedKinematics.toSwerveModuleStates(
            desiredChassisSpeeds,
      getPose().getRotation(),
            ControlCentricity.ROBOT_CENTRIC
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);
  }

  /**
   * Get current module states
   * @return Array of swerve module states
   */
  private SwerveModuleState[] getModuleStates() {
     return new SwerveModuleState[] {
      lFrontModule.getState(),
      rFrontModule.getState(),
      lRearModule.getState(),
      rRearModule.getState()
    };
  }

  /**
   * Get current module positions
   * @return Array of swerve module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      lFrontModule.getPosition(),
      rFrontModule.getPosition(),
      lRearModule.getPosition(),
      rRearModule.getPosition()
    };
  }

  /**
   * Update robot pose
   */
  private void updatePose() {
    // Save previous pose
    m_previousPose = getPose();

    // Update pose based on odometry
    poseEstimator.update(getRotation2d(), getModulePositions());

    // Update current heading
    currentHeading = new Rotation2d(getPose().getX() - m_previousPose.getX(), getPose().getY() - m_previousPose.getY());
  }

  /**
   * Log DriveSubsystem outputs
   */
  private void logOutputs() {
    Logger.recordOutput(getName() + POSE_LOG_ENTRY, getPose());
    Logger.recordOutput(getName() + ACTUAL_SWERVE_STATE_LOG_ENTRY, getModuleStates());
  }

  /**
   * SmartDashboard indicators
   */
  private void smartDashboard() {
    field.setRobotPose(getPose());
    SmartDashboard.putBoolean("TC", isTractionControlEnabled);
    SmartDashboard.putBoolean("FC", controlCentricity.equals(ControlCentricity.FIELD_CENTRIC));
  }

  /**
   * Start calling this repeatedly when robot is in danger of tipping over
   */
  private void antiTip() {
    // Calculate direction of tip
    double direction = Math.atan2(getRoll().in(Units.Degrees), getPitch().in(Units.Degrees));

    // Drive to counter tipping motion
    drive(
            DRIVE_MAX_LINEAR_SPEED.div(4).times(Math.cos(direction)),
      DRIVE_MAX_LINEAR_SPEED.div(4).times(Math.sin(direction)),
      Units.DegreesPerSecond.of(0.0)
    );
  }

  /**
   * Aim robot at a desired point on the field
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Target point, pass in null to signify invalid point
   * @param controlCentricity True to point back of robot to target
   * @param velocityCorrection True to compensate for robot's own velocity
   */
  private void aimAtPoint(ControlCentricity controlCentricity, double xRequest, double yRequest, double rotateRequest, Translation2d point, boolean reversed, boolean velocityCorrection) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    LinearVelocity velocityOutput = throttleMap.throttleLookup(moveRequest);

    // Drive normally and return if invalid point
    if (point == null) {
      AngularVelocity rotateOutput = rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest).unaryMinus();
      drive(
        controlCentricity,
        velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
        velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
        rotateOutput,
        getInertialVelocity()
      );
      return;
    }

    // Adjust point
    point = point.plus(AIM_OFFSET);
    // Get current pose
    Pose2d currentPose = getPose();
    // Angle to target point
    Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
    // Movement vector of robot
    Vector2D robotVector = new Vector2D(velocityOutput.times(currentHeading.getCos()).magnitude(), velocityOutput.times(currentHeading.getSin()).magnitude());
    // Aim point
    Translation2d aimPoint = point.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
    // Vector from robot to target
    Vector2D targetVector = new Vector2D(currentPose.getTranslation().getDistance(point) * targetAngle.getCos(), currentPose.getTranslation().getDistance(point) * targetAngle.getSin());
    // Parallel component of robot's motion to target vector
    Vector2D parallelRobotVector = targetVector.scalarMultiply(robotVector.dotProduct(targetVector) / targetVector.getNormSq());
    // Perpendicular component of robot's motion to target vector
    Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector).scalarMultiply(velocityCorrection ? AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR : 0.0);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = point.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());
    // Calculate necessary rotate rate
    double rotateOutput = reversed
      ? autoAimPIDControllerBack.calculate(currentPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees(), adjustedAngle.getDegrees())
      : autoAimPIDControllerFront.calculate(currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

    // Log aim point
    Logger.recordOutput(getName() + "/AimPoint", new Pose2d(aimPoint, new Rotation2d()));
    double aimError = currentPose.getRotation().getDegrees() - adjustedAngle.getDegrees();
    Logger.recordOutput(getName() + "/AimError", Math.copySign(((180 - Math.abs(aimError)) % 180), (aimError)));

    // Drive robot accordingly
    drive(
      controlCentricity,
      velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
      velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
      Units.DegreesPerSecond.of(rotateOutput),
      getInertialVelocity()
    );
  }



  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed [-1.0, +1.0]
   */
  private void teleopPID(double xRequest, double yRequest, double rotateRequest) {
    // Calculate move request and direction
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);

    // Get throttle and rotate output
    LinearVelocity velocityOutput = throttleMap.throttleLookup(moveRequest);
    AngularVelocity rotateOutput = rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest).unaryMinus();

    // Update auto-aim controllers
    autoAimPIDControllerFront.calculate(
      getPose().getRotation().getDegrees(),
      getPose().getRotation().getDegrees()
    );
    autoAimPIDControllerBack.calculate(
      getPose().getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees(),
      getPose().getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees()
    );

    // Drive robot
    drive(
            controlCentricity,
      velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
      velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
      rotateOutput,
      getInertialVelocity()
    );
  }

  /**
   * Lock swerve modules
   */
  private void lock() {
    lFrontModule.lock();
    rFrontModule.lock();
    lRearModule.lock();
    rRearModule.lock();
  }

  /**
   * Stop robot
   */
  private void stop() {
    lFrontModule.stop();
    rFrontModule.stop();
    lRearModule.stop();
    rRearModule.stop();
  }

  /**
   * Toggle traction control
   */
  private void toggleTractionControl() {
    isTractionControlEnabled = !isTractionControlEnabled;
    lFrontModule.toggleTractionControl();
    rFrontModule.toggleTractionControl();
    lRearModule.toggleTractionControl();
    rRearModule.toggleTractionControl();
  }

  /**
   * Enable traction control
   */
  private void enableTractionControl() {
    isTractionControlEnabled = true;
    lFrontModule.enableTractionControl();
    rFrontModule.enableTractionControl();
    lRearModule.enableTractionControl();
    rRearModule.enableTractionControl();
  }

  /**
   * Disable traction control
   */
  private void disableTractionControl() {
    isTractionControlEnabled = false;
    lFrontModule.disableTractionControl();
    rFrontModule.disableTractionControl();
    lRearModule.disableTractionControl();
    rRearModule.disableTractionControl();
  }

  /**
   * Reset pose estimator
   * @param pose Pose to set robot to
   */
  private void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
      getRotation2d(),
      getModulePositions(),
      pose
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Filter inertial velocity
    navx.getInputs().velocityX = (MutLinearVelocity) Units.MetersPerSecond.of(
      xVelocityFilter.calculate(navx.getInputs().velocityX.in(Units.MetersPerSecond))
    );
    navx.getInputs().velocityY = (MutLinearVelocity) Units.MetersPerSecond.of(
      yVelocityFilter.calculate(navx.getInputs().velocityY.in(Units.MetersPerSecond))
    );

    if (RobotBase.isSimulation()) return;
    updatePose();
    smartDashboard();
    logOutputs();
  }

  @Override
  public void simulationPeriodic() {

    updatePose();
    smartDashboard();
    logOutputs();
  }

  /**
   * Call this repeatedly to drive during autonomous
   * @param speeds Calculated swerve module states
   */
  public void autoDrive(ChassisSpeeds speeds) {
    // Get requested chassis speeds, correcting for second order kinematics
    desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(speeds);

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = advancedKinematics.toSwerveModuleStates(
            desiredChassisSpeeds,
      getPose().getRotation(),
      ControlCentricity.ROBOT_CENTRIC
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);

    // Update turn PID
    rotatePIDController.calculate(getAngle(), getRotateRate(), 0.0);

    // Update auto-aim controllers
    autoAimPIDControllerFront.calculate(
      getPose().getRotation().getDegrees(),
      getPose().getRotation().getDegrees()
    );
    autoAimPIDControllerBack.calculate(
      getPose().getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees(),
      getPose().getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees()
    );
  }

  /**
   * Toggles between field centric and robot centric drive control
   */
  private void toggleControlCentricity() {
    if (controlCentricity == ControlCentricity.FIELD_CENTRIC) {
      this.controlCentricity = ControlCentricity.ROBOT_CENTRIC;
    } else {
      this.controlCentricity = ControlCentricity.FIELD_CENTRIC;
    }
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y-axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param pointSupplier Desired point supplier
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(() -> aimAtPoint(
            controlCentricity,
      xRequestSupplier.getAsDouble(),
      yRequestSupplier.getAsDouble(),
      rotateRequestSupplier.getAsDouble(),
      pointSupplier.get(),
      reversed,
      velocityCorrection
    ),
            this::resetRotatePID
    );
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(xRequestSupplier, yRequestSupplier, rotateRequestSupplier, () -> point, reversed, velocityCorrection);
  }

  /**
   * Aim robot at desired point on the field
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim robot at point while strafing
   */
  public Command aimAtPointCommand(Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(() -> 0.0, () -> 0.0, () -> 0.0, () -> point, reversed, velocityCorrection);
  }


  /**
   * Drive the robot
   * @param xRequestSupplier X axis speed supplier
   * @param yRequestSupplier Y axis speed supplier
   * @param rotateRequestSupplier Rotate speed supplier
   * @return Command that will drive robot
   */
  public Command driveCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier) {
    return run(() -> teleopPID(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble(), rotateRequestSupplier.getAsDouble()));
  }

  /**
   * Lock swerve modules
   * @return Command to lock swerve modules
   */
  public Command lockCommand() {
    return runOnce(this::lock);
  }

  /**
   * Stop robot
   * @return Command to stop robot
   */
  public Command stopCommand() {
    return runOnce(() -> {
      stop();
      resetRotatePID();
    });
  }

  /**
   * Toggle traction control
   * @return Command to toggle traction control
   */
  public Command toggleTractionControlCommand() {
    return runOnce(this::toggleTractionControl);
  }

  /**
   * Toggles between field and robot oriented drive control
   * @return Command to toggle control centricity between robot and field centric drive control
   */
  public Command toggleCentricityCommand() {
    return runOnce(this::toggleControlCentricity);
  }

  /**
   * Enable traction control
   * @return Command to enable traction control
   */
  public Command enableTractionControlCommand() {
    return runOnce(this::enableTractionControl);
  }

  /**
   * Disable traction control
   * @return Command to disable traction control
   */
  public Command disableTractionControlCommand() {
    return runOnce(this::disableTractionControl);
  }

  /**
   * Reset pose estimator
   * @param poseSupplier Pose supplier
   * @return Command to reset pose
   */
  public Command resetPoseCommand(Supplier<Pose2d> poseSupplier) {
    return runOnce(() -> resetPose(poseSupplier.get()));
  }

  /**
   * @return Command to aim a point on the field in robot centric mode
   */
  public Command aimAtPointRobotCentric(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                        Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(() ->
      aimAtPoint(
        ControlCentricity.ROBOT_CENTRIC,
        xRequestSupplier.getAsDouble(),
        yRequestSupplier.getAsDouble(),
        rotateRequestSupplier.getAsDouble(),
        pointSupplier.get(),
        reversed,
        velocityCorrection
      ),
            this::resetRotatePID
    );

  }

  /**
   * Reset DriveSubsystem turn PID
   */
  public void resetRotatePID() {
    rotatePIDController.setSetpoint(getAngle());
    rotatePIDController.reset();
  }

    /**
   * Get constraints for path following
   * @return Path following constraints
   */
  public PathConstraints getPathConstraints() {
    return new PathConstraints(
      3.0,
      1.0,
      DRIVE_ROTATE_VELOCITY.in(Units.RadiansPerSecond),
      DRIVE_ROTATE_ACCELERATION.magnitude()
    );
  }

  /**
   * Get robot relative speeds
   * @return Robot relative speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get estimated robot pose
   * @return Currently estimated robot pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get whether or not robot is tipping over
   * @return True if robot is tipping
   */
  public boolean isTipping() {
    return Math.abs(getPitch().in(Units.Degrees)) > TIP_THRESHOLD ||
           Math.abs(getRoll().in(Units.Degrees)) > TIP_THRESHOLD;
  }

  /**
   * Get whether or not robot is nearly balanced
   * @return True if robot is (nearly) balanced
   */
  public boolean isBalanced() {
    return Math.abs(getPitch().in(Units.Degrees)) < BALANCED_THRESHOLD &&
           Math.abs(getRoll().in(Units.Degrees)) < BALANCED_THRESHOLD;
  }

  /**
   * Get if robot is aimed at desired target
   * @return True if aimed
   */
  public boolean isAimed() {
    return (autoAimPIDControllerFront.atGoal() || autoAimPIDControllerBack.atGoal()) && getRotateRate().lt(AIM_VELOCITY_THRESHOLD);
  }

  /**
   * Get inertial velocity of robot
   * @return Inertial velocity of robot in m/s
   */
  public LinearVelocity getInertialVelocity() {
    return Units.MetersPerSecond.of(
      Math.hypot(navx.getInputs().velocityX.in(Units.MetersPerSecond), navx.getInputs().velocityY.in(Units.MetersPerSecond))
    );
  }

  /**
   * Get pitch of robot
   * @return Current pitch angle of robot in degrees
   */
  public Angle getPitch() {
    // Robot pitch axis is navX pitch axis
    return navx.getInputs().pitchAngle;
  }

  /**
   * Get roll of robot
   * @return Current roll angle of robot in degrees
   */
  public Angle getRoll() {
    // Robot roll axis is navX roll axis
    return navx.getInputs().rollAngle;
  }

  /**
   * Return the heading of the robot in degrees
   * @return Current heading of the robot in degrees
   */
  public Angle getAngle() {
    return navx.getInputs().yawAngle;
  }

  /**
   * Get rotate rate of robot
   * @return Current rotate rate of robot
   */
  public AngularVelocity getRotateRate() {
    return navx.getInputs().yawRate;
  }

  /**
   * Return the heading of the robot as a Rotation2d.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * @return Current heading of the robot as a Rotation2d.
   */
  public Rotation2d getRotation2d() {
    return navx.getInputs().rotation2d;
  }

  @Override
  public void close() {
    navx.close();
    lFrontModule.close();
    rFrontModule.close();
    lRearModule.close();
    rRearModule.close();
  }
}