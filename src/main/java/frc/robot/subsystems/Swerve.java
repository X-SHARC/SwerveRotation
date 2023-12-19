// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  final boolean invertAllModules = true;
  private double kP = 0.00156;

  SwerveModule module = new SwerveModule();

  SwerveModulePosition swerveModulePosition = module.getModulePosition();

  public Swerve() {
    resetAllEncoders();
    //SmartDashboard.putData("Field", field2D);
  }
  
  public void stopModule(){
    module.stopMotors();
  }
  

  public void resetAllEncoders(){
    module.resetDriveEncoder();
    module.resetRotationEncoder();
  }


  public void drive(double xSpeed, double ySpeed, double rot) {
    SwerveModuleState[] state = Constants.Swerve.kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(state, Constants.Swerve.kMaxSpeed);

    module.setDesiredState(state);
    
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Swerve.kMaxSpeed);
    module.setDesiredState(desiredStates);
    
  }

  @Override
  public void periodic() {
    swerveModulePosition = module.getModulePosition();
  }

}