import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline 

class Planar3DOF:
    def __init__(self, L1=1.0, L2=1.0, L3=0.5):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.link_lengths = [L1, L2, L3]

        self.joint_limits = {
            'th1': [-np.pi/2, np.pi/2],
            'th2': [-np.pi/2, np.pi/2],
            'th3': [-np.pi/2, np.pi/2],
        }

    def check_joint_limits(self, q):
        if q is None:
            return False
        
        if q[0] < self.joint_limits['th1'][0] or q[0] > self.joint_limits['th1'][1]:
            return False
        if q[1] < self.joint_limits['th2'][0] or q[1] > self.joint_limits['th2'][1]:
            return False
        if q[2] < self.joint_limits['th3'][0] or q[2] > self.joint_limits['th3'][1]:
            return False
            
        return True

    def forward_kinematics(self, q):
        th1, th2, th3 = q
        
        psi = th1 + th2 + th3
        
        x = self.L1 * np.cos(th1) + self.L2 * np.cos(th1 + th2) + self.L3 * np.cos(th1 + th2 + th3)
        y = self.L1 * np.sin(th1) + self.L2 * np.sin(th1 + th2) + self.L3 * np.sin(th1 + th2 + th3)
        
        return x, y, psi

    def inverse_kinematics(self, x, y, psi, config='elbow_down'):
        try:
            x_w = x - self.L3 * np.cos(psi)
            y_w = y - self.L3 * np.sin(psi)
            
            r2 = x_w**2 + y_w**2

            cos_th2 = (r2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)

            if cos_th2 < -1.0 or cos_th2 > 1.0:
                return None 
            
            th2 = -np.arccos(cos_th2) if config == 'elbow_down' else np.arccos(cos_th2)

            alpha = np.arctan2(y_w, x_w)
            beta = np.arctan2(self.L2 * np.sin(th2), self.L1 + self.L2 * np.cos(th2))
            th1 = alpha - beta

            th3 = psi - th1 - th2

            q = np.array([th1, th2, th3])
            q = np.arctan2(np.sin(q), np.cos(q))
            
            if not self.check_joint_limits(q):
                return np.array(['LIMIT_FAIL'] * 3)
            
            return q
            
        except Exception as e:
            return None

    def plot_workspace(self, num_points=500):
        amplitude = np.pi / 2.0
        period = 4 * np.pi       
        
        t = np.linspace(0, period, num_points)
        
        th1_path = amplitude * np.sin(t)
        th2_path = amplitude * np.sin(t + np.pi / 2.0)
        th3_path = np.zeros_like(t) + np.pi / 4.0

        x_points = []
        y_points = []
        
        for i in range(num_points):
            q = [th1_path[i], th2_path[i], th3_path[i]]
            x, y, _ = self.forward_kinematics(q)
            x_points.append(x)
            y_points.append(y)

        plt.figure(figsize=(8, 8))
        plt.plot(x_points, y_points, linewidth=2, label='Continuous Wave Path')
        
        plt.scatter(x_points[0], y_points[0], marker='o', color='green', s=100, label='Start')
        plt.scatter(x_points[-1], y_points[-1], marker='o', color='red', s=100, label='End')
        
        max_reach = self.L1 + self.L2 + self.L3
        plt.scatter(0, 0, marker='x', color='black', s=100, label='Base')
        circle_max = plt.Circle((0, 0), max_reach, color='gray', fill=False, linestyle='--', alpha=0.5, label='Max Reach Boundary')
        plt.gca().add_patch(circle_max)

        plt.title('3-DOF Manipulator Continuous Path')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.xlim(-max_reach * 1.1, max_reach * 1.1)
        plt.ylim(-max_reach * 1.1, max_reach * 1.1)
        plt.grid(True)
        plt.legend()
        plt.show()

def plot_arm_config(ax, arm, q, color, alpha=1.0):
    th1, th2, th3 = q
    L1, L2, L3 = arm.L1, arm.L2, arm.L3
    
    # Joint 1 position (End of Link 1)
    j1x = L1 * np.cos(th1)
    j1y = L1 * np.sin(th1)
    
    # Joint 2 position (End of Link 2)
    j2x = j1x + L2 * np.cos(th1 + th2)
    j2y = j1y + L2 * np.sin(th1 + th2)
    
    # End-Effector position (End of Link 3)
    eex = j2x + L3 * np.cos(th1 + th2 + th3)
    eey = j2y + L3 * np.sin(th1 + th2 + th3)
    
    ax.plot([0, j1x, j2x, eex], [0, j1y, j2y, eey], 
            linewidth=4, linestyle='-', marker='o', 
            markersize=6, color=color, alpha=alpha, markevery=[0, 1, 2, 3])
    
    ax.scatter(0, 0, marker='x', color='black', s=100)

if __name__ == "__main__":
    
    arm = Planar3DOF(L1=0.142, L2=0.142, L3=0.045)
    
    arm.plot_workspace()
    via_points_cartesian = np.array([
        [0.30, 0.00, 0.0],       # P1: extended straight ahead
        [0.25, 0.05, np.pi/4],   # P2: move slightly back and up
        [0.28, -0.05, 0.0],      # P3: move down
        [0.30, 0.00, -np.pi/4]   # P4: return to max extension with rotation
    ])
    
    T_segment = 4.0
    num_points_per_segment = 150 
    
    via_points_joint = []
    
    print("\n--- IK Solving for Via Points ---")
    for i, (x, y, psi) in enumerate(via_points_cartesian):
        q = arm.inverse_kinematics(x, y, psi, config='elbow_down') 
        
        if q is None:
            print(f"FAILED (Workspace): Point {i+1} ({x:.2f}, {y:.2f}, {psi:.2f}) is physically UNREACHABLE (too far/too close). Trajectory planning will stop.")
            exit()
        
        if q[0] == 'LIMIT_FAIL':
            print(f"FAILED (Joint Limits): Point {i+1} ({x:.2f}, {y:.2f}, {psi:.2f}) is reachable, but requires a joint angle OUTSIDE the defined limits.")
            exit()
            via_points_cartesian = np.array([
        [0.30, 0.00, 0.0],       # P1: extended straight ahead
        [0.25, 0.05, np.pi/4],   # P2: move slightly back and up
        [0.28, -0.05, 0.0],      # P3: move down
        [0.30, 0.00, -np.pi/4]   # P4: return to max extension with rotation
    ])
    

        via_points_joint.append(q)
        print(f"SUCCESS: Point {i+1}: (x={x:.2f}, y={y:.2f}, psi={psi:.2f}) -> q={q.round(3)}")

    via_points_joint = np.array(via_points_joint)

    total_segments = len(via_points_joint) - 1
    t_via = np.arange(total_segments + 1) * T_segment
    
    time_total = np.linspace(0, total_segments * T_segment, total_segments * num_points_per_segment)
    
    q_trajectory = np.zeros((len(time_total), 3))
    q_dot_trajectory = np.zeros((len(time_total), 3))
    q_ddot_trajectory = np.zeros((len(time_total), 3))

    for j in range(3): 
        q_via = via_points_joint[:, j]
        
        cs = CubicSpline(t_via, q_via, bc_type='clamped')
        
        q_trajectory[:, j] = cs(time_total)
        q_dot_trajectory[:, j] = cs(time_total, 1)
        q_ddot_trajectory[:, j] = cs(time_total, 2)

    
    plt.figure(figsize=(15, 5))
    
    labels = ["Joint 1 ($\Theta_1$)", "Joint 2 ($\Theta_2$)", "Joint 3 ($\Theta_3$)"]
    
    plt.subplot(1, 3, 1)
    for j in range(3):
        plt.plot(time_total, q_trajectory[:, j], label=labels[j])
    plt.title('Joint Position (Cubic Spline Trajectory)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.grid(True)
    plt.legend()
    
    plt.subplot(1, 3, 2)
    for j in range(3):
        plt.plot(time_total, q_dot_trajectory[:, j], label=labels[j])
    plt.title('Joint Velocity ($\dot{q}$)')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (rad/s)')
    plt.grid(True)

    plt.subplot(1, 3, 3)
    for j in range(3):
        plt.plot(time_total, q_ddot_trajectory[:, j], label=labels[j])
    plt.title('Joint Acceleration ($\ddot{q}$)')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (rad/s²)')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    x_path, y_path, psi_path = [], [], []
    for q in q_trajectory:
        x, y, psi = arm.forward_kinematics(q)
        x_path.append(x)
        y_path.append(y)
        psi_path.append(psi)

    fig, ax = plt.subplots(figsize=(8, 8))
    
    #the actual EE trajectory (thicker line for visibility)
    ax.plot(x_path, y_path, label='EE Trajectory', color='blue', linewidth=3, alpha=0.7)
    
    #the arm configuration at the via points (P1, P2, P3, P4)
    time_indices = [0, num_points_per_segment, 2 * num_points_per_segment, 3 * num_points_per_segment - 1]
    
    #Start Configuration (P1)
    plot_arm_config(ax, arm, q_trajectory[time_indices[0]], color='green', alpha=1.0)
    
    #Intermediate Configuration (P2, P3)
    plot_arm_config(ax, arm, q_trajectory[time_indices[1]], color='orange', alpha=0.5)
    plot_arm_config(ax, arm, q_trajectory[time_indices[2]], color='orange', alpha=0.5)
    
    #End Configuration (P4)
    plot_arm_config(ax, arm, q_trajectory[time_indices[3]], color='red', alpha=1.0)
    
    for i, point in enumerate(via_points_cartesian):
        ax.scatter(point[0], point[1], marker='o', color='red', s=70, zorder=5)
        ax.text(point[0], point[1], f'P{i+1}', fontsize=12, ha='right')

    max_plot_limit = 0.35 
    ax.set_xlim(0.0, max_plot_limit) 
    ax.set_ylim(-0.15, 0.2) 
    
    ax.set_title('End-Effector Path in Cartesian Space ')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.legend(['EE Trajectory', 'P1 Arm Config', 'P2/P3 Arm Config', 'P4 Arm Config'])
    plt.show()
    print("\n--- Trajectory planning and verification complete. ---")
