package frc.robot.commands;



import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class SwerveCommand extends CommandBase {

  XboxController joystick;
  Swerve swerveSubsystem;

  public SwerveCommand(Swerve sw, XboxController driver){
    this.swerveSubsystem = sw;
    this.joystick = driver;

    addRequirements(swerveSubsystem);
  }


  @Override
  public void execute() {
    double xSpeed =
      (Math.abs(joystick.getLeftY()) < 0.1) ? 0 : joystick.getLeftY()* Constants.Swerve.kMaxSpeed;

    double ySpeed = 
      (Math.abs(joystick.getLeftX()) <  0.1) ? 0 : joystick.getLeftX()
      * Constants.Swerve.kMaxSpeed;
     
    double rot = 
      (Math.abs(joystick.getRightX()) < 0.1) ? 0 : joystick.getRightX()
      * Constants.Swerve.kMaxAngularSpeed ;

    swerveSubsystem.drive(xSpeed, ySpeed, rot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
