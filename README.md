# Moveit implemenation of the Gplines

- Generatate minimum acceleration, jerk, snap motions on the top of the ompl planner.
- Alloes to implement opstop and have minimum time path-consisten emergancy stop of the robot.


In specific this repo has:
- An adapter plugin to be used with ompl planning pipeline: given the waypoints returned by a planner, this substite
- A cotroller which can interact with the FollowJointTrajectoryGspline action.
    - This controller is useful to implement the opstop emergency stopping trajectoy

# Example
