package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignAndDriveCommand extends Command 
{
    private VisionSubsystem m_vision;
    private SwerveSubsystem m_swerve;
    private CommandXboxController m_controller;

    public AlignAndDriveCommand(VisionSubsystem vision, SwerveSubsystem swerve, CommandXboxController controller)
    {
        this.m_vision = vision;
        this.m_swerve = swerve;
        this.m_controller = controller;
    }

    @Override
    public void execute()
    {
        double tx = this.m_vision.getXYA()[0];

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(this.m_controller.getLeftY()*-1, this.m_controller.getLeftX()*-1, -0.10*tx);

        m_swerve.driveFieldOriented(desiredSpeeds);
    }

}
