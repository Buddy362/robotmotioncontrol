import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#params
wheel_radius = 0.05  
robot_radius = 0.1  
wheel_base = 0.2  
max_speed = 4.0  
dt = 0.1  

#init state
robot_state = [0.0, 0.0, 0.0]

#goal points on plt
goal_points = [(2.0, 2.0), (-2.0, -2.0)]
current_goal = 0

# updating state using dead reckoning
def update_robot_state(v_left, v_right):
   
    delta_theta = (wheel_radius / wheel_base) * (v_right - v_left) * dt
    delta_dist = 0.5 * wheel_radius * (v_right + v_left) * dt

    robot_state[0] += delta_dist * math.cos(robot_state[2] + 0.5 * delta_theta)
    robot_state[1] += delta_dist * math.sin(robot_state[2] + 0.5 * delta_theta)
    robot_state[2] += delta_theta

def point_stabilization():
    global current_goal
    goal_x, goal_y = goal_points[current_goal]

    #angle to goal calc
    dx = goal_x - robot_state[0]
    dy = goal_y - robot_state[1]
    goal_theta = math.atan2(dy, dx)

    #angle difference
    diff_theta = goal_theta - robot_state[2]

    #rotation adjust
    if diff_theta > math.pi:
        diff_theta -= 2 * math.pi
    elif diff_theta < -math.pi:
        diff_theta += 2 * math.pi

    #control law
    v = max_speed * math.cos(diff_theta)
    omega = max_speed * math.sin(diff_theta) / robot_radius

    return v - omega * wheel_base / 2, v + omega * wheel_base / 2


#updates robot position
def update(frame):
    global current_goal
    
    v_left, v_right = point_stabilization()
    update_robot_state(v_left, v_right)
    robot_plot.set_offsets([[robot_state[0], robot_state[1]]])  
    if math.sqrt((goal_points[current_goal][0] - robot_state[0])**2 + (goal_points[current_goal][1] - robot_state[1])**2) < 0.1:
        current_goal = (current_goal + 1) % len(goal_points)
        print("Reached goal point ", current_goal)
    return robot_plot,

#plt
fig, ax = plt.subplots()
ax.axis('equal')
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)

#this shows on plot goal points
ax.plot([p[0] for p in goal_points], [p[1] for p in goal_points], 'go', label='Goal Points')


#plotting the robot
robot_plot = ax.scatter(robot_state[0], robot_state[1], marker='o', color='blue', label='Robot')

ax.legend()
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_title('Robot Motion')

#animation
ani = animation.FuncAnimation(fig, update, frames=range(100), interval=dt*1000, blit=True)

plt.show()
