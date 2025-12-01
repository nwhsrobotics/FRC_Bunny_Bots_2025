// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
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
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.util.RobotCANUtils.CANSparkMaxController;
import frc.robot.util.RobotCANUtils.MotorKind;


public class ScoringSubsystem extends SubsystemBase {// Add FeedForward for the pivot
  
  public static enum States{
    Idle,
    
    LoadStageOne, //Spin intake and indexer wheels
    
    LoafStageTwo, // Spin only the intaje wheels
    
    ShootHigh, //spin the flyWheels at high sleep, feeder wheels and the indexer wheels
    
    ShootLow,//spin the flywheels at
  }
 
 
  // Pivot Motor declerations 
  private CANSparkMaxController pivotMotor;
  private SparkMaxConfig pivotCfg = new SparkMaxConfig();
  private SparkClosedLoopController pivotPid;
  private double currentPos = 1.0; // starting pos
  private DutyCycleEncoder absEncoder;
  private RelativeEncoder motorEnc;
  private static final double ZERO_ROT = 0.803;

  // Indexer Motor Declerations








  public ScoringSubsystem() {
    
    //Pivot Motor Init
    absEncoder = new DutyCycleEncoder(0);
    pivotMotor = new CANSparkMaxController(2, MotorKind.NEO30AMP, pivotCfg, IdleMode.kBrake, 0.1, 0.0, 0.0, 1500, 1500, 0.6);
    pivotPid = pivotMotor.getClosedLoopController();
    motorEnc = pivotMotor.getEncoder();
    motorEnc.setPosition(Math.abs(absEncoder.get() - ZERO_ROT) * ScoringConstants.PIVOT_GEAR_RATIO);


  }

  public void intake(){
    currentPos = degreesToMotorRot(135); // CHANGE DEGREE TO BE MORE ACCURATE
    //ADD STAR WHEELS ROTATOIN

  }

  public void stow(){

    currentPos = degreesToMotorRot(0.8);



  }

  public double degreesToMotorRot(double degree){
    
    return (degree/360.0) * ScoringConstants.PIVOT_GEAR_RATIO;

  }

   public double getOffsetedCurrentPos() {
     double rot = absEncoder.get() - ZERO_ROT;

     // wrap into [0,1)
     

     return rot;
 }

  @Override
  public void periodic() { // method called once per scheduler run

    //System.out.println((int)currentPos + "      " + (int)(currentPos - desiredPos));
    //must convert pos and vel to radians
    //double angleRad = pid.getSetpoint.position * 2.0 * Math.PI;
    //double velRad   = pid.getSetpoint.velocity * 2.0 * Math.PI;
    //pivotMotor.setVoltage(pid.calculate(currentPos, desiredPos) + armFeedforward.calculate(angleRad, velRad));
    //pivotMotor.setVoltage(pid.calculate(currentPos, desiredPos));
    
    pivotPid.setReference(currentPos, ControlType.kMAXMotionPositionControl);
    System.out.println(absEncoder.get() + "   " + motorEnc.getPosition() + "   " + currentPos);
  }
}
