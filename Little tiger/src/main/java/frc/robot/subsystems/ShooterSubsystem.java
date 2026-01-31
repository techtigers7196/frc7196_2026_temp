package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shooter subsystem: encoder, PID feedback, feedforward, and NetworkTables telemetry.
 * PID and feedforward are initialized to 0.0 (tunable later in Constants or NetworkTables).
 */
public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_shooterMotor = new SparkMax(10, MotorType.kBrushless);
  private final Encoder m_shooterEncoder = new Encoder(0, 1, false);

  // Feedforward and PID (start at 0.0 per request)
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController m_shooterPID = new PIDController(0.0, 0.0, 0.0);

  // NetworkTables telemetry
  private double m_setpointRPS = 0.0; //Stores the current requested shooter setpoint in rotations per second.
  private final NetworkTable m_nt = NetworkTableInstance.getDefault().getTable("Shooter");
  private final NetworkTableEntry m_rpmEntry = m_nt.getEntry("rpmRPS");
  private final NetworkTableEntry m_setpointEntry = m_nt.getEntry("setpointRPS");

  public ShooterSubsystem() {
    // sensible defaults so code compiles even without external Constants
    m_shooterEncoder.setDistancePerPulse(1.0);
    m_shooterPID.setTolerance(0.0);

    // initialize NetworkTables
    m_rpmEntry.setDouble(0.0);
    m_setpointEntry.setDouble(0.0);

    // default command: disable motor (idle)
    setDefaultCommand(Commands.runOnce(() -> m_shooterMotor.disable()).andThen(Commands.run(() -> {})).withName("ShooterIdle"));
  }

  /**
   * Command that runs the shooter to the provided setpoint (RPS). The command continuously
   * applies feedforward + PID feedback to the motor. The parameter is a DoubleSupplier so
   * callers can provide a constant or a dynamic source.
   */
  public Command runShootCommand(DoubleSupplier setpointSupplier) {
    return Commands.run(() -> {
      double setpoint = setpointSupplier.getAsDouble();
      double ff = m_shooterFeedforward.calculate(setpoint);
      double pidOutput = m_shooterPID.calculate(m_shooterEncoder.getRate(), setpoint);
      m_shooterMotor.set(ff + pidOutput);
    }, this).withName("RunShooter");
  }

  /** Set the desired shooter speed (rotations per second) and publish to NetworkTables. */
  public void setSetpointRPS(double rps) {
    m_setpointRPS = rps;
    m_setpointEntry.setDouble(rps);
  }

  @Override
  public void periodic() {
    // publish encoder rate (rotations per second) and current setpoint
    double rpm = m_shooterEncoder.getRate();
    m_rpmEntry.setDouble(rpm);
    m_setpointEntry.setDouble(m_setpointRPS);
    // Also publish to SmartDashboard for easy graphing
    SmartDashboard.putNumber("Shooter/rpmRPS", rpm);
    SmartDashboard.putNumber("Shooter/setpointRPS", m_setpointRPS);
  }
}

