clc;

% Initialize the interface from matlab to UR5 in ROS2
robot = ur5_interface();

% Get the robot's current joint angles
current_joint = robot.get_current_joints();

% Change the joint angles a little to drive the robot
goal_joint = current_joint;
goal_joint(2) = goal_joint(2) - 0.5;

% Drive the robot to the goal joint angles within given time
t = 5; % give the robot 5 secs to move
robot.move_joints(goal_joint,t);
pause(t)

% Alternatively, if you have a series of joint states that forms a
% whole trajectory

% Here I just have a foobar-style trajectory, but you can insert any
% trajecotries you like
target = [ -0.5 ; -1.57; -0.5 ; -0.5 ; 0.5 ; 0.2];
start = robot.get_current_joints();
step_num = 100;
joint_trajectory = zeros(6,step_num);
for i=1:1:6
    waypoint = linspace(start(i), target(i),step_num);
    joint_trajectory(i,:) = waypoint;
end

for j = 1:1:step_num
robot.move_joints(joint_trajectory(:,j),0.1);
pause(0.1)
end