/*
 * NOT USED
 * NOT USED
 */
package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import static frc.robot.Constants.MULTIPLIER;
import static frc.robot.Constants.MULTIPLIER_NORMAL;
import static frc.robot.Constants.MULTIPLIER_PITCH;
import static frc.robot.Constants.MULTIPLIER_TABLE;

public class Teleop {
    private final TalonFX leftLeader;
    private final TalonFX rightLeader;
    private final SparkMax armPitch;
    private final SparkMax armPitch2;
    private final SparkMax turntable;
    private final PS4Controller joystick;
    private final Joystick joystick1;
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);

    public Teleop(TalonFX leftLeader, TalonFX rightLeader, SparkMax armPitch, SparkMax armPitch2, SparkMax turntable, PS4Controller joystick, Joystick joystick1) {
        this.leftLeader = leftLeader;
        this.rightLeader = rightLeader;
        this.armPitch = armPitch;
        this.armPitch2 = armPitch2;
        this.turntable = turntable;
        this.joystick = joystick;
        this.joystick1 = joystick1;
    }

    public void periodic() {
        double fwd = -joystick.getRawAxis(1);
        double rot = joystick.getRawAxis(2);

        if (joystick.getR1Button()) {
            rot *= MULTIPLIER;
            fwd *= MULTIPLIER;
        } else {
            rot *= MULTIPLIER_NORMAL;
            fwd *= MULTIPLIER_NORMAL;
        }
        double pitchturn = joystick1.getRawAxis(1);
        double rotturn = -joystick1.getRawAxis(0);
        pitchturn *= MULTIPLIER_PITCH;
        rotturn *= MULTIPLIER_TABLE;
        if (joystick.getPOV() >= 270 && joystick.getPOV() <= 90) {
            leftLeader.set(10);
            rightLeader.set(-10);
        } else if (joystick.getPOV() <= 270 && joystick.getPOV() >= 90) {
            leftLeader.set(-10);
            rightLeader.set(10);
        }
        leftOut.Output = fwd + rot;
        rightOut.Output = fwd - rot;
        if (!joystick1.getRawButton(1)) {
            leftLeader.setControl(leftOut);
            rightLeader.setControl(rightOut);
            armPitch.set(pitchturn);
            turntable.set(rotturn);
            armPitch2.set(-pitchturn);
        }
    }
}
