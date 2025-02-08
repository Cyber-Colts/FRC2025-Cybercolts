// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PS4Controller;
//import edu.wpi.first.wpilibj.PowerDistribution;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Robot extends TimedRobot {
  SparkMax leftLeader;
  SparkMax leftFollower;
  SparkMax rightLeader;
  SparkMax rightFollower;
  SparkMax launcher;
  PS4Controller joystick;
  DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);
  DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);
  //PowerDistribution PDP = new PowerDistribution(0, ModuleType.kRev);
  


  public Robot() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    
    camera.setResolution(1280, 720);

    
    // Initialize the SPARKs

    leftLeader = new SparkMax(2, MotorType.kBrushed);
    leftFollower = new SparkMax(4, MotorType.kBrushed);
    rightLeader = new SparkMax(1, MotorType.kBrushed);
    rightFollower = new SparkMax(3, MotorType.kBrushed);
    
    launcher = new SparkMax(5, MotorType.kBrushed);

    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig spareConfig = new SparkMaxConfig();

    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    //SparkMaxConfig launcherConfig = new SparkMaxConfig();
    
    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    
    spareConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kCoast);

    // Apply the global config and invert since it is on the opposite side
    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    // Apply the global config and set the leader SPARK for follower mode
    leftFollowerConfig
        .apply(globalConfig)
        .follow(leftLeader);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(rightLeader);
    
    launcherConfig
        .apply(spareConfig);

    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Initialize joystick
    joystick = new PS4Controller(0);
  }
  
  @Override
  public void robotPeriodic() {
    // Display the applied output of the left and right side onto the dashboard
    SmartDashboard.putNumber("Left Out", leftLeader.getAppliedOutput());
    SmartDashboard.putNumber("Right Out", rightLeader.getAppliedOutput());
    SmartDashboard.putData("Encoder", m_encoder);
    SmartDashboard.putBoolean("Teleop Is On", isTeleopEnabled());
    SmartDashboard.putData("Diff", drive);
    //double voltage = PDP.getVoltage();
    //SmartDashboard.putNumber("Voltage", voltage);
    //SmartDashboard.putData("PDP",PDP);
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
    double forward = -joystick.getRightY();
    double rotation = joystick.getLeftX();
    
    // Define a multiplier
    double multiplier = 0.5; // Adjust this value as needed
    
    // Apply the multiplier to the joystick inputs
    rotation *= multiplier;
    forward *= multiplier;
    
    if (joystick.getRawButtonPressed(8)) {
      launcher.set(10); // When pressed the intake turns counter-clockwise
    }
    if (joystick.getRawButtonPressed(6)) {
      launcher.set(-10); // When pressed the intake turns clockwise
    }
    if (joystick.getRawButtonReleased(8)) {
      launcher.set(0); // When pressed the intake turns off
    }
    if (joystick.getRawButtonReleased(6)) {
      launcher.set(0); // When pressed the intake turns off
    }
    
    /*
     * Apply values to left and right side. We will only need to set the leaders
     * since the other motors are in follower mode.
     */
    leftLeader.set(forward - rotation);
    rightLeader.set(forward + rotation);
  }
    
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
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
}
