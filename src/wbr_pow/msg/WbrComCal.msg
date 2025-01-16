# Give inertia tensor centor of mass and equilibrium point information

# ibb(0) -> 1, 1
# ibb(1) -> 1, 2
# ibb(2) -> 1, 3
# ibb(3) -> 2, 1
# ... so on

float64[9] ibb      # Inertia tensor (save 3x3 matrix using 1D array)
float64[3] com      # Center of mass (x, y, z)
float64 theta_eq    # Equilibrium point of wbr