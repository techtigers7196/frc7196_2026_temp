package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase
{
    private SparkMax Shootermotor = new SparkMax (10, MotorType.kBrushless);
    // Shooter sensors and controllers
    private final Encoder m_shooterEncoder =
        new Encoder(
            ShooterConstants.kEncoderPorts[0],
            ShooterConstants.kEncoderPorts[1],
            ShooterConstants.kEncoderReversed);

    private final SimpleMotorFeedforward m_shooterFeedforward =
        new SimpleMotorFeedforward(0.0, 0.0);

    private final PIDController m_ShooterFeedback =
        new PIDController(0.0, 0.0, 0.0);
    private SparkMax elevatorMotor = new SparkMax (ElevatorSubsystemConstants.kelevatorMotorCanId, MotorType.kBrushless);
    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private SparkMax winchMotor = new SparkMax (winch.kwinchMotorCanId, MotorType.kBrushed);

//PID variables for manual PID
    private final double kP = 0.2;
    private final double kI = 0;
    private final double kD = 0.08;
    private final double kfeedforward = 0.003;

    //PID controller
    private final PIDController pid = new PIDController(kP, kI, kD);
    
    ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorSubsystemConstants.kS, 
    ElevatorSubsystemConstants.kG,
    ElevatorSubsystemConstants.kV,
    ElevatorSubsystemConstants.kA
  );
        // NetworkTables telemetry: encoder RPM and desired setpoint (RPS)

    private double m_setpointRPS = 0.0;
    private final NetworkTable m_nt = NetworkTableInstance.getDefault().getTable("Shooter");
    private final NetworkTableEntry m_rpmEntry = m_nt.getEntry("rpmRPS");
    private final NetworkTableEntry m_setpointEntry = m_nt.getEntry("setpointRPS");
    // NetworkTables telemetry: encoder RPM and desired setpoint (RPS)
    
    public ShooterSubsystem()
    {
    m_ShooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);

        // initialize NetworkTables values

    m_rpmEntry.setDouble(0.0);
    m_setpointEntry.setDouble(0.0);
    
        // Set default command to turn off both the shooter and feeder motors, and then idle

    setDefaultCommand(
        runOnce(() -> {
            m_Shootermotor.disable();
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
        /** Set the desired shooter speed (rotations per second). */

    public void setSetpointRPS(double rps) {
        m_setpointRPS = rps;
        m_setpointEntry.setDouble(rps);
    }
    @Override
    public void periodic() {
        // publish current encoder rate (rotations per second) and current setpoint
        double rpm = m_shooterEncoder.getRate();
        m_rpmEntry.setDouble(rpm);
        m_setpointEntry.setDouble(m_setpointRPS);
    }

}
