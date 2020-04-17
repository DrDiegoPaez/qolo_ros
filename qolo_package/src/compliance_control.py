import os, math
# import rospy

# Set to high priority (max -20, min 19)
try:
    os.nice(-10)
except PermissionError:
    # Need to run via sudo for high priority
    print("Cannot set niceness for the process...")
    print("Run the script as sudo...")


# Parameters
l = 0.5
r = 0.33
Ts = 0.1
C = 1
M = 1

# Global Variables
force  = [0, 0, 0]
moment = [0, 0, 0]


def compliance_control(v_prev, omega_prev, Fmag, theta, h):
    # F = m \Delta \ddot{x} + c \Delta \dot{x} + k \Delta x
    # And set reference to 0 and discretize w/ ZOH
    
    stheta = math.sin(theta)    # Small optimization
    ctheta = math.cos(theta)    # Small optimization

    # Position wrt center of rotatiion
    R = math.sqrt( (r*stheta)**2 + (l + r*ctheta)**2 )
    beta = math.atan2(r * stheta, l)

    sbeta = math.sin(beta)      # Small optimization
    cbeta = math.cos(beta)      # Small optimization
    
    a = ctheta
    b = R*(stheta*cbeta - ctheta*sbeta)

    # Admittance Control
    v_eff_prev = (a * v_prev) + (b * omega_prev)

    v_eff_dot = (Fmag - C*v_eff_prev) / M
    v_eff = v_eff_dot * Ts + v_eff_prev

    # # Calculate new v and omega
    # c_prev = (-b * v_prev) + (a * omega_prev)
    # den = a**2 - b**2
    # v = (a*v_eff - b*c_prev) / den
    # omega = (-b*v_eff + a*c_prev) / den

    # Ensure non-zero 'a' and 'b'
    eps = 0.01
    if (a < eps):
        return (v_prev, v_eff/b)
    if (b < eps):
        return (v_eff/a, omega_prev)

    # Calculate new v and omega in parameterized form
    t = 0.5     # \in [0,1]
    v = t * v_prev + (1-t) * (v_eff - b*omega_prev) / a
    omega = t * (v_eff - a*v_prev) / b + (1-t) * omega_prev

    return (v, omega)


if __name__ == "__main__":

    print(compliance_control(5, 1, 10, 30, 0))