package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.Swerve;



public class RobotContainer {
  XboxController driver = new XboxController(0);  

  Swerve swerve = new Swerve();

  SwerveCommand command = new SwerveCommand(swerve, driver);

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
  swerve.setDefaultCommand(command);
  }

  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
