package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Intake extends SubsystemBase {

    public double intakeSpeed = 0.0;

    Talon m_IntakeMotor = new Talon(0);

    public void setIntakeSpeed(double speed) {
        if (speed > 1.0) {
            speed = 1.0;
        } else if (speed < -1.0) {
            speed = -1.0;
        }

        m_IntakeMotor.set(speed);
    }

    @Override

    public void periodic() {
        SmartDashboard.putString("Running", "Yes");
        SmartDashboard.putNumber("PWm Value", m_IntakeMotor.get());
    }
}


