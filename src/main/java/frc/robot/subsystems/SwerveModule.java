// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.util.Gearbox;


public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private TalonFX angleMotor = new TalonFX(17);
  private TalonFX driveMotor = new TalonFX(18);

  public PIDController rotPID = new PIDController(0.001, 0, 0);

  private CANCoder rotEncoder = new CANCoder(2);

  Gearbox driveRatio = new Gearbox(6.75, 1);
  
  public SwerveModule() {
    
    this.driveMotor.setNeutralMode(NeutralMode.Brake);
    this.angleMotor.setNeutralMode(NeutralMode.Brake);

    rotEncoder.setPositionToAbsolute();

    rotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    rotPID.setTolerance(20);
    rotEncoder.configMagnetOffset(5);

    rotEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

  }

  public double getDegrees(){
    return rotEncoder.getAbsolutePosition();
  }

  public SwerveModulePosition getModulePosition(){
    return new SwerveModulePosition(getPosition(), getAngle());
  }

  public double getPosition(){
    return 
      driveMotor.getSelectedSensorPosition() * (Constants.Swerve.wheelCircumference / (6.75 * 2048.0));
  }

  public double getDriveMotorRate(){
    return 
    driveRatio.calculate(
      ((getDriveRawVelocity() * 10) / 2048.0) * Constants.Swerve.wheelCircumference
    );
  }

  public double getDriveRawVelocity(){
    return  driveMotor.getSelectedSensorVelocity();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
     getDegrees()
    );
  }

  public void resetRotationEncoder(){
    rotEncoder.setPosition(0);
  }
  
  public void resetDriveEncoder(){
    driveMotor.setSelectedSensorPosition(0);    
  }

  public void stopMotors(){
    driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    angleMotor.set(TalonFXControlMode.PercentOutput, 0);
  }



  // ! Open loop drive
  public void setDesiredState(SwerveModuleState[] desiredState) {
    
    if(Math.abs(desiredState[0].speedMetersPerSecond)<0.001){
      stopMotors();
      return;
    }
    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState[0], currentRotation);
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);

    double desiredRotation = 150;//currentRotation.getDegrees() + rotationDelta.getDegrees();

    rotPID.setSetpoint(desiredRotation);
    

    angleMotor.set(TalonFXControlMode.PercentOutput, rotPID.calculate(currentRotation.getDegrees()));

    SmartDashboard.putNumber("1- Current Rotation: ", currentRotation.getDegrees());
    SmartDashboard.putNumber("2- Desired Rotation", desiredRotation); 
    SmartDashboard.putNumber("PID Out", rotPID.calculate(currentRotation.getDegrees()));

    //driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / Constants.Swerve.kMaxSpeed);
  }

}
