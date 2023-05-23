import numpy as np

K_H_DEFAULT = -0.9
K_P_DEFAULT = -0.9

def getUProportional(vecx_0, vecx_f, xy_tol, theta_tol, xy_err_l2,theta_err_abs,
    K_p = K_P_DEFAULT, K_h = K_H_DEFAULT, debug=False):
    """
    vecx_0 = np.array([x_0, y_0, theta_0])
    vecx_f = np.array([x_f, y_f, theta_f])
    xy_tol
    theta_tol
    xy_err_l2
    theta_err_abs

    Returns u = np.array([V, omega])
    """
    vecx_0 = np.array(vecx_0).flatten()
    vecx_f = np.array(vecx_f).flatten()

    assert vecx_0.shape == (3,)
    assert vecx_f.shape == (3,)

    x_0, y_0, theta_0 = vecx_0
    x_f, y_f, theta_f = vecx_f
    V = 0
    omega = 0
    # Are we within tolerance of position?
    if xy_err_l2 < xy_tol:
        # Yes: adjust heading
        # Are we within tolerance of heading?
        if debug:
            print("\t\t Within pos tol")
        if theta_err_abs < theta_tol:
            # Yes: do nothing
            pass
        else:
            # No: fix heading to match heading target
            omega = K_h*(theta_0 - theta_f)
    else:
        # No: adjust position
        # Compute the heading needed to point to target xy
        if debug:
            print("\t\t Not within pos tol")
        delta_y = y_0 - y_f
        delta_x = x_0 - x_f
        theta_p = np.arctan(delta_y/delta_x)

        # Are we pointing towards the goal?
        delta_theta_p = theta_0 - theta_p
        if debug:
            print("\t\t heading err:", delta_theta_p, "of ", theta_to)
        if abs(delta_theta_p) < theta_tol:
            # Yes: move towards target
            if debug:
                print("\t\t Within heading tol -> make V")
            L = np.sqrt(delta_x**2 + delta_y**2)*np.sign(delta_y/delta_x)
            V = -K_p*L
        else:
            # No: adjust heading to point towards target
            if debug:
                print("\t\t Not in heading tol -> make omega_p")
            omega = K_h*(theta_0 - theta_p)


    u = np.array([V, omega])
    return u

if __name__ == "__main__":
    print("Running unit test")
    x_0 = [0,0,0]
    x_f = [1,1,1]
    xy_tol = 0.1
    theta_tol = 0.1
    xy_err_l2 = 0.2
    theta_err_abs = 0.4

    u = getUProportional(x_0, x_f, xy_tol, theta_tol, xy_err_l2, theta_err_abs)
    print("u = ", u)






