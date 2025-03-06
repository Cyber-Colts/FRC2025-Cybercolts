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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
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
  SparkMax turntable;
  SparkMax armPitch;
  SparkMax armPitch2;
  SparkMax armExt;
  SparkMax Rollers;
  SparkMax IntakeExt;
          
  double encoderRaw;
  double encoderRawL;
  double encoderRawR;
  double wheelPosL;
  double wheelPosR;
        
        
  PS4Controller joystick;
  Joystick joystick1;
  DifferentialDriveOdometry m_odometry;
  // Creating my kinematics object: track width of 27 inches
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.58);
  // Example chassis speeds: 2 meters per second linear velocity,
  // 1 radian per second angular velocity.
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);
  // Convert to wheel speeds
  DifferentialDriveWheelSpeeds wheelSpeeds;

  // Left velocity
  double leftVelocity = wheelSpeeds.leftMetersPerSecond;
  // Right velocity
  double rightVelocity = wheelSpeeds.rightMetersPerSecond;
  
  AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  
  /*
  CAN IDS:
  LeftL = 1, LeftF = 2, RightL = 3, RightF = 4
  Rev PDP = 10
  */
  PowerDistribution PDP = new PowerDistribution(10, ModuleType.kRev);
  Field2d m_field = new Field2d();


  public Robot() {
    
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
    turntable = new SparkMax(5, MotorType.kBrushless);
    armPitch = new SparkMax(6, MotorType.kBrushless);
    armPitch2 = new SparkMax(7, MotorType.kBrushless);
    armExt = new SparkMax(8, MotorType.kBrushless);

    SparkMaxConfig launcherConfig = new SparkMaxConfig();


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
    armExt.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    //Update Pose on field
    m_odometry.update(m_gyro.getRotation2d(),
      getEncoderLeft(),
      getEncoderRight());
     
    m_field.setRobotPose(m_odometry.getPoseMeters());
    // Display the applied output of the left and right side onto the dashboard
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("Teleop Is On", isTeleopEnabled());
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putNumber("Right Encoder", getEncoderRight());
    SmartDashboard.putNumber("Left Encoder", getEncoderLeft());
    SmartDashboard.putNumber("Turntable Encoder", getEncoderTurntable());
    SmartDashboard.putNumber("Speed", m_gyro.getRobotCentricVelocityX());
    SmartDashboard.putNumber("ODOMVelocityL", leftVelocity);
    SmartDashboard.putNumber("ODOMVelocityR", rightVelocity);
    SmartDashboard.putNumber("Pitch", m_gyro.getPitch());
    SmartDashboard.putData("PDP",PDP);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
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
    double multiplierNormal = 0.3;
    double multiplierTable = 0.3; 
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
    pitchturn *= multiplierSLOW;
    rotturn *= multiplierTable;
    if(joystick.getPOV() >= 270 && joystick.getPOV() <= 90){
      rot *= multiplierSLOW;
      fwd *= multiplierSLOW;
      leftOut.Output = 1;
      rightOut.Output = -1;
    }
    else if (joystick.getPOV() <= 270 && joystick.getPOV() >= 90)
    {
      rot *= multiplierSLOW;
      fwd *= multiplierSLOW;
      leftOut.Output = -1;
      rightOut.Output = 1;
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
      armPitch2.set(-pitchturn);
      turntable.set(rotturn);
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
