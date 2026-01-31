package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.examples.rapidreactcommandbot.Constants.ShooterConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase
{
    private SparkMax Shootermotor = new SparkMax (10, MotorType.kBrushless);
    private final Encoder m_shooterEncoder = 
        new Encoder(
            ShooterConstants.kEncoderPorts[0], 
            ShooterConstants.kEncoderPorts[1],
            ShooterConstants.kEncoderReversed);
    private final SimpleMotorFeedforward m_shooterFeedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.kSVolts,
            ShooterConstants.kVVoltSecondsPerRotation);
    private final PIDController m_ShooterFeedback =
        new PIDController(
            ShooterConstants.kP,
            ShooterConstants.kI,
            ShooterConstants.kD);
    public ShooterSubsystem()
    {
    m_ShooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
        // Set default command to turn off both the shooter and feeder motors, and then idle

    setDefaultCommand(
        runOnce(() -> {
            m_ShooterMotor.disable();
            m_feederMotor.disable();
        }) 
          .andThen(run(() -> {}))
          .withName("Idle"));
  }
    
  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   * @param setpointRotationsPerSecond The desired shooter velocity
   */

    public Command runShootCommand(DoubleSupplier power)
    {   
        run_parallel(
            run(
                () -> {
                    m_ShooterMotor.set(m_shooterFeedforward.calculate(setpointRotationsPerSecond) +
                    m_ShooterFeedback.calculate(m_shooterEncoder.getRate(), setpointRotationsPerSecond));
                }
        )
        // Wait until the shooter has reached the setpoint, and then run the feeder
            waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> m_feederMotor.set(1)))
        .withName("Shoot");
        // return this.startEnd(() -> Shootermotor.set(power.getAsDouble()), () -> Shootermotor.set(0));
    }
}
