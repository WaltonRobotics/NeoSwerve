package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Swerve extends SubsystemBase {
    CANSparkMax m_motor = new CANSparkMax(4, MotorType.kBrushless);
    AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
    SparkMaxPIDController m_controller = m_motor.getPIDController();

    REVLibError resetErr = m_motor.restoreFactoryDefaults();
    REVLibError encPosConvErr = m_encoder.setPositionConversionFactor(360.0);
    REVLibError ctrlFeedbackErr = m_controller.setFeedbackDevice(m_encoder);

    public void periodic() {
        if (resetErr != REVLibError.kOk) {
            System.out.println("REV CANSparkMax.restoreFactoryDefaults error: " + resetErr);
        } else {
            System.out.println("REV CANSparkMax.restoreFactoryDefaults OK!");
        }

        if (encPosConvErr != REVLibError.kOk) {
            System.out.println("REV AbsoluteEncoder.setEncoderPositionConversion error: " + encPosConvErr);
        } else {
            System.out.println("REV AbsoluteEncoder.setEncoderPositionConversion OK!");
        }

        if (ctrlFeedbackErr != REVLibError.kOk) {
            System.out.println("REV SparkMaxPIDController.setFeedbackDevice error: " + ctrlFeedbackErr);
        } else {
            System.out.println("REV SparkMaxPIDController.setFeedbackDevice OK!");
        }

    }
}
