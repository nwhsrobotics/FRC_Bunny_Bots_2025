// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.util.RobotCANUtils.CANSparkMaxController;
import frc.robot.util.RobotCANUtils.MotorKind;

public class IntakeOuttakeSubsystem extends SubsystemBase {
  // super cool and awesome IntakeOuttakeSubsystem
  private CANSparkMaxController pivotMotor;
  private SparkMaxConfig pivotCfg = new SparkMaxConfig();
  private SparkClosedLoopController motorPid;
  private double currentPos;



  public IntakeOuttakeSubsystem() {//Parameters are subject to change!
    pivotMotor = new CANSparkMaxController(CANAssignments.ALGAE_MOTOR_ID, MotorKind.NEO30AMP, pivotCfg, IdleMode.kBrake, 0.1, 0, 0, 0, 0, .6, false);
    motorPid = pivotMotor.getClosedLoopController();
  
  }

  public void stow(){
    
    currentPos = degreesToMotorRot(90); // CHANGE DEGREE TO BE MORE ACCURATE
    //ADD STAR WHEELS ROTATOIN
  
  }

  public void intake(){

    currentPos = degreesToMotorRot(0);

  }

  public double degreesToMotorRot(double degree){
    
    return (degree/360);

  }
  

  @Override
  public void periodic() { // method called once per scheduler run
    motorPid.setReference(currentPos, ControlType.kMAXMotionPositionControl);
  }
}
