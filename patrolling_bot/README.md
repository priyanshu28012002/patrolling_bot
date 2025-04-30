# Behaviour Tree Node Descriptions

## Action Nodes

| Node ID                     | Purpose                                                |
|----------------------------|--------------------------------------------------------|
| `UpdateBlackBoard`         | Updates internal/shared data used across the tree      |
| `IdentifyMostRiskyPoint`   | Finds the next high-risk area to patrol                |
| `MoveToGole`               | Commands movement to the assigned goal location        |
| `SetGoleToChargingStation` | Sets the goal to the charging station when needed      |
| `SetGoleToMostRiskypoint`  | Sets the goal to the most risky point identified       |

## Condition Nodes

| Node ID                               | Purpose                                                  |
|---------------------------------------|----------------------------------------------------------|
| `IsBatteryCritical`                   | Checks if battery is critically low                      |
| `ReachToGole`                         | Verifies if the agent has reached its goal               |
| `IsEnoughBattryToPatrollCheckPoint`  | Ensures enough battery remains to patrol next checkpoint |

## Blackboard Data

| Key              | Type                   | Description                                |
|------------------|------------------------|--------------------------------------------|
| `battery_status` | `float`                | Current battery level                       |
| `risk`           | `std::vector<float>`   | Risk levels at various points              |
| `position`       | `std::vector<double>`  | Current position of the robot              |
| `orientation`    | `std::vector<double>`  | Current orientation of the robot           |
| `gole_pose`      | `std::vector<double>`  | Target goal position for navigation        |
