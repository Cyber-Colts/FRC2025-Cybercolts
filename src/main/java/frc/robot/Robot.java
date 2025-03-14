// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.AUTO_FWD;
import static frc.robot.Constants.AUTO_FWD_SLOW;
import static frc.robot.Constants.AUTO_ROT;
import static frc.robot.Constants.AUTO_STOP;
import static frc.robot.Constants.GOAL_HEIGHT_INCHES;
import static frc.robot.Constants.LIMELIGHT_LENS_HEIGHT_INCHES;
import static frc.robot.Constants.LIMELIGHT_MOUNT_ANGLE_DEGREES;
import static frc.robot.Constants.MT_AMOUNT;
import static frc.robot.Constants.MULTIPLIER;
import static frc.robot.Constants.MULTIPLIER_NORMAL;
import static frc.robot.Constants.MULTIPLIER_PITCH;
import static frc.robot.Constants.MULTIPLIER_TABLE;

public class Robot extends TimedRobot {
  private final CANBus kCANBus = new CANBus();

  private final TalonFX leftLeader = new TalonFX(1, kCANBus);
  private final TalonFX leftFollower = new TalonFX(2, kCANBus);
  private final TalonFX rightLeader = new TalonFX(3, kCANBus);
  private final TalonFX rightFollower = new TalonFX(4, kCANBus);
  
  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);
  
  private final Timer m_timer = new Timer();
  private SparkAbsoluteEncoder turntable_encoder;
  private SparkAbsoluteEncoder throughBoreTurntableEncoder;
  private SparkClosedLoopController PIDControllerTurntable;

  private static final int SMART_MOTION_SLOT = 0;
  // Offset in rotations to add to encoder value - offset from arm horizontal to sensor zero
  private static final double ENCODER_OFFSET = -0.58342d;
  private static final double GRAVITY_FF = 0.01;
  private static final float LIMIT_BOTTOM = 0.5804f;
  private static final float LIMIT_TOP = 0.8995f;

  private Double targetPosition = null;


  SparkMax turntable;
  SparkMax armPitch;  SparkMax armPitch2;
  SparkMax armExt; SparkMax intakeRoller1; SparkMax intakeRoller2; SparkMax intakePivot; SparkMax armRot;

  double encoderRaw; double encoderRawL; double encoderRawR;
  double wheelPosL; double wheelPosR;
  
  // Basic targeting data
  double tx;  double ty; double ta;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tableE = table.getEntry("tableE");

  double targetOffsetAngle_Vertical;
  double angleToGoalDegrees;
  double angleToGoalRadians; // converts to radians
  
  double heading;
  LimelightHelpers.PoseEstimate mt1;
  boolean mtCheck;
  double mtID = 0;
  double getTagID;
  double mtID1_2 = 1;
  boolean hasTarget;
  double tync;  double txnc;
  double degrees1 = 0;
  double degrees2 = 0;
  boolean doRejectUpdate;
  DifferentialDrivePoseEstimator m_poseEstimator;

  double distanceFromLimelightToGoalInches; //calculate distance
  double Estimate_Distance;

  PS4Controller joystick;
  Joystick joystick1;
  
  DifferentialDriveOdometry m_odometry;
  // Creating my kinematics object: track width of 0.58 meters
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.58);
  // Example chassis speeds: 2 meters per second linear velocity,
  // 1 radian per second angular velocity.
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
  // Convert to wheel speeds
  DifferentialDriveWheelSpeeds wheelSpeeds;

  // Left velocity
  //double leftVelocity = wheelSpeeds.leftMetersPerSecond;
  // Right velocity
  //double rightVelocity = wheelSpeeds.rightMetersPerSecond;
  
  AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  
  /*
  CAN IDS:
  LeftL = 1, LeftF = 2, RightL = 3, RightF = 4
  Rev PDP = 10
  */
  PowerDistribution PDP = new PowerDistribution(10, ModuleType.kRev);
  Field2d m_field = new Field2d();
  public enum CoralPivotPositions {
    L1(0.2),
    L2(0),
    L3(0),
    L4(0);

    private final double value;

    CoralPivotPositions(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }

  }
  public Robot() {
    //all the ports the limelight needs for configuartions
    CameraServer.startAutomaticCapture();
        
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5805, "limelight.local", 5805);

    PortForwarder.add(5801, "limelight-one.local", 5802);
    PortForwarder.add(5800, "limelight-one.local", 5808);
    PortForwarder.add(5805, "limelight-one.local", 5806);

    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
    turntable = new SparkMax(5, MotorType.kBrushless);
    armPitch = new SparkMax(6, MotorType.kBrushless);
    armPitch2 = new SparkMax(7, MotorType.kBrushless);
    PIDControllerTurntable = turntable.getClosedLoopController();
    turntable_encoder = turntable.getAbsoluteEncoder();
    armExt = new SparkMax(8, MotorType.kBrushless);
    intakeRoller1 = new SparkMax(11, MotorType.kBrushless);
    intakeRoller2 = new SparkMax(12, MotorType.kBrushless);
    intakePivot = new SparkMax(13, MotorType.kBrushless);
    armRot  = new SparkMax(10, MotorType.kBrushless);

    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    SparkMaxConfig turntableConfig = new SparkMaxConfig();
    SparkMaxConfig armExtConfig = new SparkMaxConfig();
    SparkMaxConfig intakerotConfig = new SparkMaxConfig();



    //table = NetworkTable.getTable("datatable");

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    launcherConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);
    turntableConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);
    armExtConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);
    intakerotConfig
        .smartCurrentLimit(100)
        .idleMode(IdleMode.kBrake);
    turntableConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(4)
        .d(0)
        .outputRange(-.5, .5);

    turntable.configure(turntableConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armPitch.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armPitch2.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armExt.configure(armExtConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeRoller1.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRoller2.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakePivot.configure(intakerotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armRot.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    


    leftLeader.getConfigurator().apply(leftConfiguration);
    leftFollower.getConfigurator().apply(leftConfiguration);
    rightLeader.getConfigurator().apply(rightConfiguration);
    rightFollower.getConfigurator().apply(rightConfiguration);

    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);

    // Initialize joystick
    joystick = new PS4Controller(0);
    joystick1 = new Joystick(1);
    
    m_poseEstimator =
      new DifferentialDrivePoseEstimator(
          kinematics,
          m_gyro.getRotation2d(),
          getEncoderLeft(),
          getEncoderRight(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    /*
    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      getEncoderLeft(), getEncoderRight(),
      new Pose2d(5.0, 13.5, new Rotation2d()));
    */
  }

  @Override
  public void robotPeriodic() {
    // Update limelight detection data
    updateLimelightData();
    // Display the data onto the dashboard
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("Teleop Is On", isTeleopEnabled());
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putNumber("Right Encoder", getEncoderRight());
    SmartDashboard.putNumber("Left Encoder", getEncoderLeft());
    SmartDashboard.putNumber("Turntable Encoder", getTurntablePosition());
    SmartDashboard.putNumber("Speed", m_gyro.getRobotCentricVelocityX());
    SmartDashboard.putNumber("Pitch", m_gyro.getPitch());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("PDP",PDP);
    //SmartDashboard.putNumber("LimelightX", tx);
    //SmartDashboard.putNumber("LimelightY", ty);
    /*
    TODO: remove this on the future, we only use this to see if the code is reading the limelight
    but the limelight already broadcasts the data to the networktable by itself
    */
    SmartDashboard.putNumber("LimelightArea", LimelightHelpers.getTA("limelight"));
    SmartDashboard.putNumber("Limelight Distance", mt1.rawFiducials[0].distToCamera);
    SmartDashboard.putNumber("LimelightID", LimelightHelpers.getFiducialID("limelight"));
    
  }
  /*public void limelightTarget() {
    xHeading_error = -x;
    yHeading_error = -y;
    x_adjust = 0.0f;
    y_adjust = 0.0f;
    if (x > 1.0)
    {
      x_adjust = Kp*xHeading_error - min_command;
    }
    else if (x < 1.0)
    {
      x_adjust = Kp*xHeading_error + min_command;
    }
    left_command += x_adjust;
    right_command -= x_adjust;

    if (y > 1.0)
    {
      y_adjust = Kp*yHeading_error - min_command;
    }
    else if (y < 1.0)
    {
      y_adjust = Kp*yHeading_error + min_command;
    }
    forward_command += y_adjust;
    backward_command -= y_adjust;

    m_robotDrive.driveCartesian(y_adjust, x_adjust, 0.0);

  }*/

  @Override
  public void autonomousInit() {
    //var limelightName = "limelight";
    m_timer.reset();
    m_timer.start();
    leftLeader.set(0);
    rightLeader.set(0);
    m_gyro.setAngleAdjustment(0);
    // Set a custom crop window for improved performance (-1 to 1 for each value)
    /*LimelightHelpers.setCropWindow(limelightName, -0.5, 0.5, -0.5, 0.5);
    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(limelightName, 0.5, 0.0, 0.5, 0.0, 30.0, 0.0);
    // Set AprilTag offset tracking point (meters)
    LimelightHelpers.setFiducial3DOffset(limelightName, 0.0, 0.0, 0.5);
    // Configure AprilTag detection
    LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, new int[]{0, 1, 2}); // Only track these tag IDs
    LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 2.0f); // Process at half resolution for improved framerate and reduced range
    */
  }
  
  @Override
  public void autonomousPeriodic() {
    //limelight logic
      if (mtCheck && getTagID == 1) {
          degrees1 = 44.0;
          degrees2 = 46.0;
      } else if (mtCheck && getTagID == 2) {
          degrees1 = -44.0;
          degrees2 = -46.0;
          mtID1_2 = 2;
      }
      // Check the functions below for the logic
      if (m_timer.get() > 0.0 && m_timer.get() < 10.0) { // DRIVE FORWARD, 10 SECONDS
          drive(AUTO_FWD, AUTO_FWD);
          if (mtCheck && getTagID == mtID) {
              adjustSpeedBasedOnDistance(mt1, AUTO_FWD_SLOW, AUTO_STOP);
          } else if (mtCheck && getTagID == mtID1_2) {
              rotateToTarget(heading, degrees1, degrees2, AUTO_ROT);
              adjustSpeedBasedOnDistance(mt1, AUTO_FWD_SLOW, AUTO_STOP);
          }
      }
      drive(AUTO_STOP, AUTO_STOP);
  }

  private void drive(double leftSpeed, double rightSpeed) {
      // drivetrain control logic 
      leftLeader.set(leftSpeed);
      rightLeader.set(rightSpeed);
  }

  private void adjustSpeedBasedOnDistance(LimelightHelpers.PoseEstimate mt1, double autoFwdSlow, double autoStop) {
    /*
     * Adjust speed based on distance to target
     * If the distance is greater than 1 meter and the target area is greater than 3%, drive forward
     * If the distance is less than 0.5 meters and the target area is less than 5%, drive backward
     */

     //TODO: fix Ficudal distance calculation (https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance)
    /*F
    if (mt1.rawFiducials[0].distToCamera > 1 && LimelightHelpers.getTA("limelight") > 3) {
          drive(autoFwdSlow, autoFwdSlow);
    } else if (mt1.rawFiducials[0].distToCamera < 0.5 && LimelightHelpers.getTA("limelight") < 5) {
          drive(-autoFwdSlow, -autoFwdSlow);
    }
    */
      if (mt1.rawFiducials[0].distToCamera > 1 || LimelightHelpers.getTA("limelight") < 10) {
          drive(autoFwdSlow, autoFwdSlow);
      } else if (mt1.rawFiducials[0].distToCamera < 0.5 || LimelightHelpers.getTA("limelight") > 13) {
          drive(-autoFwdSlow, -autoFwdSlow);
      }
      if (mt1.rawFiducials[0].distToCamera == 0.4){
          drive(autoStop, autoStop);
          autonomousExit();
      }
  }

  private void rotateToTarget(double heading, double degrees1, double degrees2, double autoRot) {
      /*
       * Rotate to target based on the heading and the degrees of the target
       * If the heading is less than the degrees of the target, rotate to the right
       * If the heading is greater than the degrees of the target, rotate to the left
       */
      if (heading < degrees1) {
          drive(autoRot, -autoRot);
      } else if (heading > degrees2) {
          drive(-autoRot, autoRot);
      }
  }

  private void updateLimelightData() {
    if(Math.abs(m_gyro.getRate()) > 360)
    {
      doRejectUpdate = true;
    }
    if(mt1.tagCount == 0)
    {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    m_poseEstimator.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds);
    heading = m_gyro.getAngle();
    mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    getTagID = LimelightHelpers.getFiducialID("limelight");
    tx = LimelightHelpers.getTX("limelight");  // Horizontal offset from crosshair to target in degrees
    ty = LimelightHelpers.getTY("limelight");  // Vertical offset from crosshair to target in degrees
    ta = LimelightHelpers.getTA("limelight");  // Target area (0% to 100% of image)
    hasTarget = LimelightHelpers.getTV("limelight"); // Do you have a valid target?
    txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
    tync = LimelightHelpers.getTYNC("limelight");  // Vertical  offset from principal pixel/point to target in degrees
    targetOffsetAngle_Vertical = tableE.getDouble(0.0);
    angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
    angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0); // converts to radians
    mtCheck = hasTarget && mt1.tagCount == MT_AMOUNT && ta >= 0.1;
    distanceFromLimelightToGoalInches = (GOAL_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians); // calculate distance
    Estimate_Distance = distanceFromLimelightToGoalInches;
    }
}

  @Override
  public void teleopInit() {
  }

  //TELEOP CODE
  @Override
  public void teleopPeriodic() {
    /**
     * Get forward and rotation values from the joystick. Invert the joystick's
     * Y value because its forward direction is negative.
     */
    /* Get forward and rotational throttle from joystick */
    /* invert the joystick Y because forward Y is negative */
    double fwd = -joystick.getRawAxis(1);
    double rot = joystick.getRawAxis(2);

    // Apply the multiplier to the joystick inputs
    if (joystick.getR1Button()) {
      rot *= MULTIPLIER;
      fwd *= MULTIPLIER;
    }
    else{
      rot *= MULTIPLIER_NORMAL;
      fwd *= MULTIPLIER_NORMAL;
    }
    double pitchturn = joystick1.getRawAxis(1);
    double rotturn = -joystick1.getRawAxis(0);
    pitchturn *= MULTIPLIER_PITCH;
    rotturn *= MULTIPLIER_TABLE;
    if(joystick.getPOV() >= 270 && joystick.getPOV() <= 90){
      leftLeader.set(20);
      rightLeader.set(-20);
    }
    else if (joystick.getPOV() <= 270 && joystick.getPOV() >= 90)
    {
      leftLeader.set(-20);
      rightLeader.set(20);
    }
    /* Set output to control frames */
    leftOut.Output = fwd + rot;
    rightOut.Output = fwd - rot;
    /* And set them to the motors */
    if (joystick1.getRawButton(1)){
      intakeRoller1.set(0.5);
      intakeRoller2.set(0.5);
    }
    else if (joystick1.getRawButton( 2)){
      intakeRoller1.set(-0.5);
      intakeRoller2.set(0.5);
    }
    else{
      intakeRoller1.set(0);
      intakeRoller2.set(0);
    }
    if (joystick1.getRawButton(3)){
      intakePivot.set(0.3);
    }
    else if (joystick1.getRawButton(4)){
      intakePivot.set(-0.3);
    }
    else{
      intakePivot.set(0);
    }
    if (!joystick1.getRawButton(1)) {
      leftLeader.setControl(leftOut);
      //set pid values
      rightLeader.setControl(rightOut);
      armPitch.set(pitchturn);
      turntable.set(rotturn);
      armPitch2.set(-pitchturn);
      //turntable.set(rotturn);
    }
  }
    
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
     /* Zero out controls so we aren't just relying on the enable frame */
     leftOut.Output = 0;
     rightOut.Output = 0;
     leftLeader.setControl(leftOut);
     rightLeader.setControl(rightOut);
     armPitch.set(0);
     armPitch2.set(0);
     turntable.set(0);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public double getEncoderLeft() {
    /*apply the multiplication of the gearbox ratio of 7.31:1 to the encoder value
    taking into account pi*6 inch wheel diameter (0.1524 meters)*/
    encoderRawL = leftLeader.getPosition().getValueAsDouble();
    wheelPosL = Math.PI * 0.1524 / encoderRawL * 7.31;
    return wheelPosL;
  }

  public double getEncoderRight() {
    encoderRawR = rightLeader.getPosition().getValueAsDouble();
    wheelPosR = Math.PI * 0.1524 / encoderRawR* 7.31;
    return wheelPosR;
  }

  public double getTurntablePosition() {
    return turntable_encoder.getPosition();
  }

  public double getTurntableVelocity() {
      return turntable_encoder.getVelocity();
  }
  public void pidSetPosition(CoralPivotPositions position) {
      PIDControllerTurntable.setReference(position.getValue(), ControlType.kPosition);
  }
}