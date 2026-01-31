package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase
{
    private SparkMax Shootermotor = new SparkMax (10, MotorType.kBrushless);

    public ShooterSubsystem()
    {
        
    }

    public Command runShootCommand(DoubleSupplier power)
    {
        return this.startEnd(() -> Shootermotor.set(power.getAsDouble()), () -> Shootermotor.set(0));
    }
}
