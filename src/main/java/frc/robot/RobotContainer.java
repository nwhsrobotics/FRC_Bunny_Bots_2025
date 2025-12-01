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
import frc.robot.subsystems.ScoringSubsystem;


public class RobotContainer {
  //private final SendableChooser<Command> autoChooser;
  public final ScoringSubsystem scoringSubsystem= new ScoringSubsystem();
  public final XboxController driver = new XboxController( 0);



  public RobotContainer() {

  NamedCommands.registerCommand("intake", new InstantCommand(() -> scoringSubsystem.intake(), scoringSubsystem));
  NamedCommands.registerCommand("stow", new InstantCommand(() -> scoringSubsystem.stow(), scoringSubsystem));


  new JoystickButton(driver, Buttons.A).onTrue(NamedCommands.getCommand("intake"));
  new JoystickButton(driver, Buttons.B).onTrue(NamedCommands.getCommand("stow"));











/* 
    autoChooser = AutoBuilder.buildAutoChooser("Converter Start");
    SmartDashboard.putData("Auto Chooser", autoChooser);
*/
  }

  public Command getAutonomousCommad(){
   // return autoChooser.getSelected();
    return null;
  }

 


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
