/*
 * NOT USED
 * NOT USED
 */

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.AUTO_FWD;
import static frc.robot.Constants.AUTO_FWD_SLOW;
import static frc.robot.Constants.AUTO_ROT;
import static frc.robot.Constants.AUTO_STOP;

public class Autonomous {
    private final Timer m_timer = new Timer();
    private final TalonFX leftLeader;
    private final TalonFX rightLeader;
    private final AHRS m_gyro;
    private LimelightHelpers.PoseEstimate mt1;
    private boolean mtCheck;
    private double mtID = 0;
    private double getTagID;
    private double mtID1_2 = 1;
    private double heading;
    private double degrees1 = 0;
    private double degrees2 = 0;

    public Autonomous(TalonFX leftLeader, TalonFX rightLeader, AHRS m_gyro) {
        this.leftLeader = leftLeader;
        this.rightLeader = rightLeader;
        this.m_gyro = m_gyro;
    }

    public void init() {
        m_timer.reset();
        m_timer.start();
        leftLeader.set(0);
        rightLeader.set(0);
        m_gyro.setAngleAdjustment(0);
    }

    public void periodic() {
        if (mtCheck && getTagID == 1) {
            degrees1 = 44.0;
            degrees2 = 46.0;
        } else if (mtCheck && getTagID == 2) {
            degrees1 = -44.0;
            degrees2 = -46.0;
            mtID1_2 = 2;
        }
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
        leftLeader.set(leftSpeed);
        rightLeader.set(rightSpeed);
    }

    private void adjustSpeedBasedOnDistance(LimelightHelpers.PoseEstimate mt1, double autoFwdSlow, double autoStop) {
        if (mt1.rawFiducials[0].distToCamera > 1 && LimelightHelpers.getTA("limelight") > 3) {
            drive(autoFwdSlow, autoFwdSlow);
        } else if (mt1.rawFiducials[0].distToCamera < 0.5 && LimelightHelpers.getTA("limelight") < 5) {
            drive(-autoFwdSlow, -autoFwdSlow);
        }
    }

    private void rotateToTarget(double heading, double degrees1, double degrees2, double autoRot) {
        if (heading < degrees1) {
            drive(autoRot, -autoRot);
        } else if (heading > degrees2) {
            drive(-autoRot, autoRot);
        }
    }
}
