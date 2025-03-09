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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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



public class Robot extends TimedRobot {
  private final CANBus kCANBus = new CANBus();

  private final TalonFX leftLeader = new TalonFX(1, kCANBus);
  private final TalonFX leftFollower = new TalonFX(2, kCANBus);
  private final TalonFX rightLeader = new TalonFX(3, kCANBus);
  private final TalonFX rightFollower = new TalonFX(4, kCANBus);
  
  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);
  
  private final Timer m_timer = new Timer();

  SparkMax turntable;
  SparkMax armPitch;
  SparkMax armPitch2;
  SparkMax armExt;
  SparkMax hand;
  
  double encoderRaw; double encoderRawL; double encoderRawR;
  double wheelPosL; double wheelPosR;
  double avgTagArea; double avgTagDist;
  double x; double y; 
  double area; 
  double left_command; 
  double right_command; double forward_command; double backward_command; 
  double xHeading_error; double yHeading_error; 
  double x_adjust; double y_adjust;

  float Kp;
  float min_command;
  
  // Basic targeting data
 
  double getTagID = LimelightHelpers.getFiducialID("limelight");

  double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset from crosshair to target in degrees
  double ty = LimelightHelpers.getTY("limelight");  // Vertical offset from crosshair to target in degrees
  double ta = LimelightHelpers.getTA("limelight");  // Target area (0% to 100% of image)
  boolean hasTarget = LimelightHelpers.getTV("limelight"); // Do you have a valid target?

  double txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
  double tync = LimelightHelpers.getTYNC("limelight");  // Vertical  offset from principal pixel/point to target in degrees

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tableE = table.getEntry("tableE");
  double targetOffsetAngle_Vertical = tableE.getDouble(0.0);
  double limelightMountAngleDegrees = 25.0; // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightLensHeightInches = 8.0; // distance from the center of the Limelight lens to the floor
  double goalHeightInches = 60.0; // distance from the target to the floor
  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159/180.0); // converts to radians
  
  double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians); //calculate distance
  double Estimate_Distance = distanceFromLimelightToGoalInches;

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

  public Robot() {
    PortForwarder.add(5801, "limelight.local", 5801);


    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
    turntable = new SparkMax(5, MotorType.kBrushless);
    armPitch = new SparkMax(6, MotorType.kBrushless);
    armPitch2 = new SparkMax(7, MotorType.kBrushless);


    SparkMaxConfig launcherConfig = new SparkMaxConfig();

    //table = NetworkTable.getTable("datatable");

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    launcherConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);

    turntable.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armPitch.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armPitch2.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftLeader.getConfigurator().apply(leftConfiguration);
    leftFollower.getConfigurator().apply(leftConfiguration);
    rightLeader.getConfigurator().apply(rightConfiguration);
    rightFollower.getConfigurator().apply(rightConfiguration);

    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);

    // Initialize joystick
    joystick = new PS4Controller(0);
    joystick1 = new Joystick(1);

    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      getEncoderLeft(), getEncoderRight(),
      new Pose2d(5.0, 13.5, new Rotation2d()));
  }

  @Override
  public void robotPeriodic() {
    // Display the applied output of the left and right side onto the dashboard
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("Teleop Is On", isTeleopEnabled());
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putNumber("Right Encoder", getEncoderRight());
    SmartDashboard.putNumber("Left Encoder", getEncoderLeft());
    SmartDashboard.putNumber("Turntable Encoder", getEncoderTurntable());
    SmartDashboard.putNumber("Speed", m_gyro.getRobotCentricVelocityX());
    SmartDashboard.putNumber("Pitch", m_gyro.getPitch());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("PDP",PDP);
    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightArea", ta);
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
    double autoFwd = 0.2;
    double autoFwdSlow = 0.05;
    double autoRot = 0.07; // Invert depending on side of the field
    double autoStop = 0;
    double heading = m_gyro.getAngle();
    double mtAmount = 1;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    var mtCheck = hasTarget == true && mt1.tagCount == mtAmount && ta >= 0.1;
    double mtID = 0;
    double mtID1_2 = 1;
    double degrees1 = 0;
    double degrees2 = 0;

    if (mtCheck && getTagID == 1) {
      degrees1 = 44.0;
      degrees2 = 46.0;
    } else if (mtCheck && getTagID == 2) {
      degrees1 = -44.0;
      degrees2 = -46.0;
      mtID1_2 = 2;
    }

    if (m_timer.get() > 0.0 && m_timer.get() < 10.0) { //DRIVE FORWARD, 2 SECONDS
      leftLeader.set(autoFwd);
      rightLeader.set(autoFwd);
      if(mtCheck && getTagID == mtID) {
        if(mt1.rawFiducials[0].distToCamera > 13 && ta > 15) {
          leftLeader.set(autoFwdSlow);
          rightLeader.set(autoFwdSlow);
        } else if(mt1.rawFiducials[0].distToCamera < 11 && ta < 20) {
          leftLeader.set(-autoFwdSlow);
          rightLeader.set(-autoFwdSlow);
        }
        leftLeader.set(autoStop);
        rightLeader.set(autoStop);
      } else if (mtCheck && getTagID == mtID1_2) {
        if (heading < degrees1) {
          leftLeader.set(autoRot);
          rightLeader.set(-autoRot);
        } else if (heading > degrees2) {
          leftLeader.set(-autoRot);
          rightLeader.set(autoRot);
        } 
        if(mt1.rawFiducials[0].distToCamera > 13 && ta > 15) {
          leftLeader.set(autoFwdSlow);
          rightLeader.set(autoFwdSlow);
        } else if(mt1.rawFiducials[0].distToCamera < 11 && ta < 20) {
          leftLeader.set(-autoFwdSlow);
          rightLeader.set(-autoFwdSlow);
        }
        leftLeader.set(autoStop);
        rightLeader.set(autoStop);
      } 
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
    double multiplier = 0.6;
    double multiplierPitch = 0.5;
    double multiplierNormal = 0.5;
    double multiplierTable = 0.6; 
    double multiplierSLOW = 0.1;

    // Apply the multiplier to the joystick inputs
    if (joystick.getR1Button()) {
      rot *= multiplier;
      fwd *= multiplier;
    }
    else{
      rot *= multiplierNormal;
      fwd *= multiplierNormal;
    }
    double pitchturn = joystick1.getRawAxis(1);
    double rotturn = -joystick1.getRawAxis(0);
    pitchturn *= multiplierNormal;
    rotturn *= multiplierTable;
    if(joystick.getPOV() >= 270 && joystick.getPOV() <= 90){
      leftLeader.set(10);
      rightLeader.set(-10);
    }
    else if (joystick.getPOV() <= 270 && joystick.getPOV() >= 90)
    {
      leftLeader.set(-10);
      rightLeader.set(10);
    }
    /* Set output to control frames */
    leftOut.Output = fwd + rot;
    rightOut.Output = fwd - rot;
    /* And set them to the motors */
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
  public double getEncoderTurntable() {
    encoderRaw = turntable.getAbsoluteEncoder().getPosition();
  return encoderRaw * 4;
  }
}