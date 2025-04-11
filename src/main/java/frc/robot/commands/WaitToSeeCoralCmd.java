//
//  This command will set a triggering Event, and will end when that event has been processed.
//  Thus you can create a SerialCommandGroup full of triggeringEvents and they will execute one at a time.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tower.TowerState;
import frc.robot.subsystems.tower.Tower;

public class WaitToSeeCoralCmd extends Command {
  Tower tower;


  /** Creates a new TriggerEvent. */
  public WaitToSeeCoralCmd(Tower tower) {
    this.tower = tower;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((tower.getState() == TowerState.INTAKE_PAUSE) || 
            (tower.getState() == TowerState.GOING_TO_SAFE)  ||  
            (tower.getState() == TowerState.GOT_CORAL));
  }
}
