// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Buttons;
import frc.robot.commands.SetScoringCommand;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  //private final SendableChooser<Command> autoChooser;
  
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final ScoringSubsystem scoringSubsystem= new ScoringSubsystem();
  
  
  public final XboxController driver = new XboxController( 0);
  public final XboxController gunner = new XboxController( 1);

  public final SendableChooser<Command> autoChooser;



  public RobotContainer() {
    
    //---------------------------- SETSCORINGCOMMANDS ------------------------------
    SetScoringCommand IntakeStageOne = new SetScoringCommand(scoringSubsystem, ScoringSubsystem.ScoringStates.LoadStageOne, 3);
    SetScoringCommand IntakeStageTwo = new SetScoringCommand(scoringSubsystem, ScoringSubsystem.ScoringStates.LoadStageTwo, 3);
    SetScoringCommand unload = new SetScoringCommand(scoringSubsystem, ScoringSubsystem.ScoringStates.Unload, 3);
    
    SetScoringCommand ShootHigh = new SetScoringCommand(scoringSubsystem, ScoringSubsystem.ScoringStates.ShootHigh, 3);
    SetScoringCommand ShootLow = new SetScoringCommand(scoringSubsystem, ScoringSubsystem.ScoringStates.ShootLow, 3);

    //------------ PATH PLANNER COMMANDS ---------------
    NamedCommands.registerCommand("SHOOT_HIGH", ShootHigh);
    NamedCommands.registerCommand("SHOOT_LOW", ShootHigh);
    NamedCommands.registerCommand("INTAKE_STAGE_ONE", IntakeStageOne);
    NamedCommands.registerCommand("INTAKE_STAGE_TWO", IntakeStageTwo);


    //----------------------- CONTROLLER BINDS ---------------------
    new JoystickButton(driver, Buttons.RIGHT_BUMPER).onTrue(IntakeStageOne);
    new JoystickButton(driver, Buttons.LEFT_BUMPER).onTrue(IntakeStageTwo);
    new JoystickButton(driver, Buttons.Y).onTrue(unload);

    new JoystickButton(gunner, Buttons.A).onTrue(ShootLow);
    new JoystickButton(gunner, Buttons.Y).onTrue(ShootHigh);
    

    //------------------------- AUTO CHOOSER --------------------
    autoChooser = AutoBuilder.buildAutoChooser("null");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    
    //------------------- SWERVE ---------------------------
    swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));


  }

  public Command getAutonomousCommad(){
   return autoChooser.getSelected();
  }

 


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
