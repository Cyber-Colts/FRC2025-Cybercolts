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
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.GenericHID;
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
  SparkMax hand;
  
  double encoderRaw;
  double encoderRawL;
  double encoderRawR;
  double wheelPosL;
  double wheelPosR;


  GenericHID joystick;
  PS4Controller joystick1;
  
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

    leftLeader.getConfigurator().apply(leftConfiguration);
    leftFollower.getConfigurator().apply(leftConfiguration);
    rightLeader.getConfigurator().apply(rightConfiguration);
    rightFollower.getConfigurator().apply(rightConfiguration);

    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);
    // Initialize joystick
    joystick = new Joystick(0);
    joystick1 = new PS4Controller(1);
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
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      getEncoderLeft(), getEncoderRight(),
      new Pose2d(5.0, 13.5, new Rotation2d()));
    m_field.setRobotPose(m_odometry.getPoseMeters());
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
    double fwd = -joystick1.getRawAxis(1);
    double rot = joystick1.getRawAxis(2);
    //if(fwd != 0){
    /* Set up followers to follow leaders */
    //leftFollower.setControl(new Follower(leftLeader.getDeviceID(), true));
    //rightFollower.setControl(new Follower(rightLeader.getDeviceID(), true));
    //}
    //if(rot != 0){
    /* Set up followers to follow leaders */
    //leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    //rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));    
    //}

    if(joystick.getRawButton(5)){
      //set limit for turntable
      if(360 / 4 * turntable.getAbsoluteEncoder().getPosition() > -360) {
        turntable.set(-20);
      }
      else {
        turntable.set(0);
      }
    }
    if(!joystick.getRawButton(5)){
      turntable.set(0);
    }
    if(joystick.getRawButton(6)){
      //set limit for turntable
      if(360 / 4 * turntable.getAbsoluteEncoder().getPosition() > 360) {
        turntable.set(20);
      }
      else {
        turntable.set(0);
      }
    }
    if(!joystick.getRawButton(6)){
      turntable.set(0);
    }


    /* Set output to control frames */
    leftOut.Output = fwd + rot;
    rightOut.Output = fwd - rot;
    /* And set them to the motors */
    if (!joystick1.getRawButton(1)) {
      leftLeader.setControl(leftOut);
      rightLeader.setControl(rightOut);
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
