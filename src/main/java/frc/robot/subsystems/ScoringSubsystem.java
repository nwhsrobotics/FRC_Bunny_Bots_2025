// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.ScoringConstants;
import frc.robot.util.RobotCANUtils.CANSparkFlexController;
import frc.robot.util.RobotCANUtils.CANSparkMaxController;
import frc.robot.util.RobotCANUtils.MotorKind;

public class ScoringSubsystem extends SubsystemBase { // Add FeedForward for the pivot

    public static enum ScoringStates {
        Idle,

        Unload,

        LoadStageOne, // Spin intake and indexer wheels

        LoadStageTwo, // Spin only the intake wheels

        ShootHigh, // spin the flyWheels at high speed, feeder wheels and the indexer wheels

        ShootLow,// spin the flywheels at low speed
    }

    private ScoringStates state = ScoringStates.Idle;

    // shooter flywheels
    private final CANSparkFlexController flywheelMotor1;
    private final CANSparkFlexController flywheelMotor2;

    private final RelativeEncoder flywheelEncoder1;
    private final RelativeEncoder flywheelEncoder2;

    // feeder motor 
    private final CANSparkMaxController feederMotor;
    private final RelativeEncoder feederEncoder;

    // common SparkFlex config for both flywheels
    private final SparkFlexConfig flywheelCfg = new SparkFlexConfig();

    // pivot Motor 
    private CANSparkMaxController pivotMotor;
    private SparkMaxConfig pivotCfg = new SparkMaxConfig();
    private SparkClosedLoopController pivotPid;
    private DutyCycleEncoder absEncoder;
    private RelativeEncoder motorEnc;
    private static final double ZERO_ROT = 0.803; // change this!!!!!

    // intake motor
    private CANSparkMaxController intakeMotor;
    private SparkClosedLoopController intakePid;
    private SparkMaxConfig intakeCfg = new SparkMaxConfig();

    // indexer motor
    private CANSparkMaxController indexMotor;
    private SparkClosedLoopController indexPid;



    public ScoringSubsystem() {

        // ----------------- PIVOT CONFIG -----------------------
        absEncoder = new DutyCycleEncoder(0);
        
        pivotMotor = new CANSparkMaxController(
            CANAssignments.PIVOT_ARM_MOTOR_ID, 
            MotorKind.NEO30AMP, 
            pivotCfg, 
            IdleMode.kBrake,
            0.1, 0.0, 0.0, 
            1500, 1500, 0.6);
        
        pivotPid = pivotMotor.getClosedLoopController();
        motorEnc = pivotMotor.getEncoder();
        motorEnc.setPosition(Math.abs(absEncoder.get() - ZERO_ROT) * ScoringConstants.PIVOT_GEAR_RATIO);

        
        
        // ------------------- INTAKE CONFIG -------------------
        intakeMotor = new CANSparkMaxController(
            CANAssignments.INTAKE_MOTOR_ID, 
            MotorKind.NEO80AMP, 
            intakeCfg,
            IdleMode.kCoast,
            0.0001, 0, 0);

        intakePid = intakeMotor.getClosedLoopController();



        // ------------------- INDEXER CONFIG ------------------
        indexMotor = new CANSparkMaxController(
            CANAssignments.INDEX_MOTOR_ID, 
            MotorKind.NEO80AMP, 
            intakeCfg,
            IdleMode.kCoast,
            0.00003, 0.0000001, 0);
            
        indexPid = indexMotor.getClosedLoopController();

        
        
        // ---------------- FLYWHEEL CONFIG --------------------
        // Configure PID + sensor on the SparkFlexConfig
        flywheelCfg.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0002)
                .i(0.0)
                .d(0.0);
        flywheelCfg.idleMode(IdleMode.kCoast);

        // create both flywheel motors using the wrapper constructor
        flywheelMotor1 = new CANSparkFlexController(
                Constants.CANAssignments.FLYWHEEL_ONE_ID,
                MotorKind.VORTEX,
                flywheelCfg,
                IdleMode.kCoast);

        flywheelMotor2 = new CANSparkFlexController(
                Constants.CANAssignments.FLYWHEEL_TWO_ID,
                MotorKind.VORTEX,
                flywheelCfg,
                IdleMode.kCoast);

        flywheelEncoder1 = flywheelMotor1.getEncoder();
        flywheelEncoder2 = flywheelMotor2.getEncoder();

        
        
        // ---------------- FEEDER SETUP ----------------
        feederMotor = new CANSparkMaxController(Constants.CANAssignments.FEEDER_MOTOR_ID,
                MotorKind.NEO550,
                new SparkMaxConfig(),
                IdleMode.kBrake,
                0.00003, 0.0000001, 0, // TUNED
                0, 0, 0, .0);

        feederEncoder = feederMotor.getEncoder();


    }

    //----------------- CONTROL METHODS -----------------------

    private boolean shooterAtSpeed() {
        double rpmOne = Math.abs(flywheelEncoder1.getVelocity());
        double rpmTwo = Math.abs(flywheelEncoder2.getVelocity());

        return Math.abs(rpmOne - Constants.ScoringConstants.targetRPM) < Constants.ScoringConstants.RPM_TOLERANCE
                && Math.abs(rpmTwo - Constants.ScoringConstants.targetRPM) < Constants.ScoringConstants.RPM_TOLERANCE;
    }

    private void setShooterRPM(double rpm) {
        Constants.ScoringConstants.targetRPM = rpm;

        flywheelMotor1.getClosedLoopController().setReference(
                Constants.ScoringConstants.targetRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                Constants.ScoringConstants.targetRPM * Constants.ScoringConstants.OUTPUT_PER_RPM * 12);

        flywheelMotor2.getClosedLoopController().setReference(
                Constants.ScoringConstants.targetRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                -Constants.ScoringConstants.targetRPM * Constants.ScoringConstants.OUTPUT_PER_RPM * 12);
    }

    private void stopShooter() {
        flywheelMotor1.set(0);
        flywheelMotor2.set(0);
        Constants.ScoringConstants.targetRPM = 0;
    }

    // increase RPM and decrease RPM methods
    public void increaseRPM() {
        if (Constants.ScoringConstants.targetRPM < 5500) {
            Constants.ScoringConstants.targetRPM += 100;
        }
    }

    public void decreaseRPM() {
        if (Constants.ScoringConstants.targetRPM > 100) {
            Constants.ScoringConstants.targetRPM -= 100;
        }
    }

    private void runFeeder(double power) {
        feederMotor.set(power);
    }

    private void stopFeeder() {
        feederMotor.set(0);
    }

    public double degreesToMotorRot(double degree) {
        return (degree / 360.0) * ScoringConstants.PIVOT_GEAR_RATIO;
    }


    public void setState(ScoringStates newState) {
        this.state = newState;
        switch (state) {
            case Idle:
                pivotPid.setReference(degreesToMotorRot(0.8), ControlType.kMAXMotionPositionControl);
                indexPid.setReference(0, ControlType.kVelocity);
                intakePid.setReference(0, ControlType.kVelocity);

                stopShooter();
                stopFeeder();
                break;

            case LoadStageOne:

                pivotPid.setReference(degreesToMotorRot(135), ControlType.kMAXMotionPositionControl);
                indexPid.setReference(ScoringConstants.INTAKE_SPEED, ControlType.kVelocity);
                intakePid.setReference(ScoringConstants.INTAKE_SPEED, ControlType.kVelocity);
                break;

            case LoadStageTwo:

                pivotPid.setReference(degreesToMotorRot(135), ControlType.kMAXMotionPositionControl);
                intakePid.setReference(ScoringConstants.INTAKE_SPEED, ControlType.kVelocity);
                indexPid.setReference(0, ControlType.kVelocity);
                break;

            case Unload:

                pivotPid.setReference(degreesToMotorRot(0.8), ControlType.kMAXMotionPositionControl);
                indexPid.setReference(-ScoringConstants.INTAKE_SPEED, ControlType.kVelocity);
                intakePid.setReference(-ScoringConstants.INTAKE_SPEED, ControlType.kVelocity);
                break;

            case ShootHigh:
                setShooterRPM(Constants.ScoringConstants.HIGH_SHOT_RPM);
                stopFeeder();

                if (shooterAtSpeed()) {
                    runFeeder(1.0);
                    indexPid.setReference(ScoringConstants.INTAKE_SPEED, ControlType.kVelocity);

                }
                break;
                

            case ShootLow:
                setShooterRPM(Constants.ScoringConstants.LOW_SHOT_RPM);
                stopFeeder();

                if (shooterAtSpeed()) {
                    runFeeder(1.0);
                    indexPid.setReference(ScoringConstants.INTAKE_SPEED, ControlType.kVelocity);

                }
                break;

            default:
                break;
        }
    }

    @Override
    public void periodic() {
    }

    public double getTargetRPM() {
        return Constants.ScoringConstants.targetRPM;
    }

    public double getCurrentRPM() {
        return flywheelEncoder1.getVelocity();
    }


}