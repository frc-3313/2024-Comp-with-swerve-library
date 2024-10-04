package frc.robot.subsystems;

public class StateMachine 
{
    private boolean searchingForTarget;
    private boolean lockedOnTarget;
    private boolean hasNote;
    private boolean intake;
    private boolean shooting;
    private boolean elevatorUp;
    private boolean tilterDown;
    
    public void SetSearchingForTarget(boolean input)
    {
        searchingForTarget = input;
    }
    public boolean GetSearchingForTarget()
    {
        return searchingForTarget;
    }
    public void SetLockedOnTarget(boolean input)
    {
        lockedOnTarget = input;
    }
    public boolean GetLockedOnTarget()
    {
        return lockedOnTarget;
    }
    public void SetHasNote(boolean input)
    {
        hasNote = input;
    }
    public boolean GetHasNote()
    {
        return hasNote;
    }
    public void SetIntake(boolean input)
    {
        intake = input;
    }
    public boolean GetIntake()
    {
        return intake;
    }
    public void SetShooting(boolean input)
    {
        shooting = input;
    }
    public boolean GetShooting()
    {
        return shooting;
    }
    public void SetElevatorUp(boolean input)
    {
        elevatorUp = input;
    }
    public boolean GetElevatorUp()
    {
        return elevatorUp;
    }
    public void SetTilterDown(boolean input)
    {
        tilterDown = input;
    }
    public boolean GetTilterDown()
    {
        return tilterDown;
    }
}
