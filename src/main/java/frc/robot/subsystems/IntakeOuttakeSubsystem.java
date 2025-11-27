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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
  //private ProfiledPIDController pid;
  //private DutyCycleEncoder absEncoder;
  //private ArmFeedforward armFeedforward;
  //private double desiredPos = 0.0;
  //private static final double ZERO_ROT

  




  public IntakeOuttakeSubsystem() {//Parameters are subject to change!
    //pid = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0,0.0));
    //armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    //absEncoder = new DutyCycleEncoder(0);
    pivotMotor = new CANSparkMaxController(CANAssignments.ALGAE_MOTOR_ID, MotorKind.NEO30AMP, pivotCfg, IdleMode.kBrake, 0.1, 0, 0, 0, 0, .6, false);
    motorPid = pivotMotor.getClosedLoopController();
    //desiredPos = getOffsetedCurrentPos();
  
  }

  public void stow(){
    
    currentPos = degreesToMotorRot(90); // CHANGE DEGREE TO BE MORE ACCURATE
    //ADD STAR WHEELS ROTATOIN
    //desiredPos = degreesToMotorRot(90);

  
  }

  public void intake(){

    currentPos = degreesToMotorRot(0);
    //desiredPos = degreesToMotorRot(90);


  }

  public double degreesToMotorRot(double degree){
    
    return (degree/360);

  }

//   public double getOffsetedCurrentPos() {
//     double rot = absEncoder.get() - ZERO_ROT;

//     // wrap into [0,1)
//     if (rot < 0) 
//       rot = 0;
//     return rot;
// }

  @Override
  public void periodic() { // method called once per scheduler run
    //currentPos = getOffsetedCurrentPos();
    //must convert pos and vel to radians
    //double angleRad = pid.getSetpoint.position * 2.0 * Math.PI;
    //double velRad   = pid.getSetpoint.velocity * 2.0 * Math.PI;
    //pivotMotor.setVoltage(pid.calculate(currentPos, desiredPos) + armFeedforward.calculate(angleRad, velRad));
    motorPid.setReference(currentPos, ControlType.kMAXMotionPositionControl);
  }
}
