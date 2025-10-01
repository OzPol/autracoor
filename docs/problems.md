### Problem Statement: Collision Detection and Avoidance for Two Autonomous Agents on Orthogonal Trajectories

This investigation analyzes the conditions for collision between two autonomous agents, designated Train 1 and Train 2, operating on perpendicular paths. The problem is structured to build from a simplified point-mass model to a more realistic scenario involving finite-length agents and optimal control for collision avoidance.

**System Definition:**
* A 2D Cartesian coordinate system is established with a common intersection point, A, at the origin $(0, 0)$.
* Train 1 travels along the negative x-axis towards the origin. Its initial position is $P_1(0) = (-a, 0)$, where $a > 0$. Its velocity is constant at $\vec{V_1} = (V_1, 0)$, where $V_1 > 0$.
* Train 2 travels along the negative y-axis towards the origin. Its initial position is $P_2(0) = (0, -b)$, where $b > 0$. Its velocity is constant at $\vec{V_2} = (0, V_2)$, where $V_2 > 0$.
* The initial distance between the trains is $d = \sqrt{a^2 + b^2}$.
* The angle $\alpha$ between the vector from Train 1 to the origin and the vector connecting the two trains is given, satisfying $\cos(\alpha) = a/d$.


---

### Problem 1: Point-Mass Collision Condition

This initial problem models the trains as dimensionless point masses to establish the most fundamental collision criterion.

**Context:** This is a classic kinematics problem found in introductory university-level physics and calculus courses.

**Problem Formulation:** Determine the necessary condition for the two point-mass trains to collide at the intersection point A. Provide the mathematical expression governing this condition and formulate a computational procedure (pseudocode) to verify it.

**Mathematical Analysis:** A collision occurs if and only if both trains reach the origin $(0, 0)$ at the exact same time. The time of arrival for each train is:
$$t_1 = \frac{a}{V_1} \quad \text{and} \quad t_2 = \frac{b}{V_2}$$
For a collision to occur, their arrival times must be identical:
$$\frac{a}{V_1} = \frac{b}{V_2}$$

**Pseudocode for Collision Check:**


    function CheckPointCollision(a, b, V1, V2):  

        // Inputs: a, b, V1, V2  
        if V1 <= 0 or V2 <= 0:  
            return ERROR "Speeds must be positive." 
            
        t1_arrival = a / V1  
        t2_arrival = b / V2. 
        
        // Use a small tolerance (epsilon) for float comparison in practice        
        if t1_arrival == t2_arrival:
            return TRUE // Collision is imminent
        else:  
            return FALSE // No collision


---

### Problem 2: Finite-Interval Collision Condition

This problem extends the model to account for the physical length of the trains, represented by the time it takes for them to pass through the intersection.

**Context:** An introductory engineering or applied mathematics problem focusing on the overlap of time intervals.

**Problem Formulation:** Assume both trains require a fixed time period, $t_p$, to fully pass through the intersection. Derive the mathematical condition for collision under this constraint and provide pseudocode.

**Mathematical Analysis:** A collision occurs if the time intervals during which each train occupies the intersection, $I_1 = [t_1, t_1 + t_p]$ and $I_2 = [t_2, t_2 + t_p]$, overlap. The condition for overlap is concisely expressed as:
$$|t_1 - t_2| \le t_p$$
Substituting the expressions for $t_1$ and $t_2$:
$$\left| \frac{a}{V_1} - \frac{b}{V_2} \right| \le t_p$$

**Pseudocode for Interval Collision Check:**

'''sql
    function CheckIntervalCollision(a, b, V1, V2, tp):
        // Inputs: a, b, V1, V2, and pass-through time tp
        if V1 <= 0 or V2 <= 0:
            return ERROR "Speeds must be positive."
        t1_arrival = a / V1
        t2_arrival = b / V2
        if abs(t1_arrival - t2_arrival) <= tp:
            return TRUE // Collision will occur
        else:
            return FALSE // No collision
'''

---


### Problem 3: Optimal Collision Avoidance by Speed Modification

This problem transitions from detection to active avoidance, introducing an optimization criterion.

**Context:** A problem characteristic of undergraduate control systems, robotics, or optimization courses.

**Problem Formulation:** Assuming a collision is predicted, determine the new speed for Train 1, $V_1'$, that prevents the collision while minimizing the change in speed, $|\Delta V_1| = |V_1' - V_1|$.

**Mathematical Analysis:** To avoid collision, we seek the smallest adjustment, which places the system at the boundary of the safe region: $|a/V_1' - b/V_2| = t_p$. This yields two potential solutions:
1.  $V_{1, \text{slow}}' = \frac{a}{(b/V_2) + t_p}$ (Slowing down)
2.  $V_{1, \text{fast}}' = \frac{a}{(b/V_2) - t_p}$ (Speeding up, valid only if $b/V_2 > t_p$)

The optimal speed $V_1'$ is the valid candidate that minimizes $|V_1' - V_1|$.

**Pseudocode for Speed Adjustment:**

'''json
    function CalculateAvoidanceSpeed(a, b, V1, V2, tp):
        if not CheckIntervalCollision(a, b, V1, V2, tp):
            return V1 // No adjustment needed
        t2_arrival = b / V2
        V1_slow = a / (t2_arrival + tp)
        if t2_arrival > tp:
            V1_fast = a / (t2_arrival - tp)
            // Choose the speed that requires the minimum change
            if abs(V1_slow - V1) <= abs(V1_fast - V1):
                return V1_slow
            else:
                return V1_fast
        else:
            return V1_slow // Speeding up is not a valid option
'''


---

### Problem 4: Collision Avoidance under Kinodynamic Constraints

This problem incorporates the physical limitations of the train's propulsion and braking systems.

**Context:** This is an advanced undergraduate or graduate-level problem in Optimal Control Theory or Robotics (Trajectory Planning).

**Problem Formulation:** Given an imminent collision and a desired safe arrival time $t_{1, \text{safe}}'$, determine a feasible constant acceleration $A$ that allows the train to travel the distance $a$ in the required time, subject to the constraint $|A| \le A_{\max}$.

**Mathematical Analysis:** Using the kinematic equation $d = v_0 t + \frac{1}{2} A t^2$, we solve for the required constant acceleration $A$ for a given target time $t$:
$$A = \frac{2(a - V_1 t)}{t^2}$$
We calculate this for the "slow" and "fast" target times derived in Problem 3. A solution is feasible only if the required $|A|$ is less than or equal to $A_{\max}$. The optimal choice is the feasible solution with the minimum $|A|$.

**Pseudocode for Feasible Constant Acceleration:**

'''sql
    function FindFeasibleAcceleration(a, b, V1, V2, tp, A_max):
        if not CheckIntervalCollision(a, b, V1, V2, tp):
            return 0 // No acceleration needed
        t2_arrival = b / V2
        // Evaluate "slow down" option
        t_slow = t2_arrival + tp
        A_slow = 2 * (a - V1 * t_slow) / (t_slow^2)
        is_slow_feasible = (abs(A_slow) <= A_max)
        // Evaluate "speed up" option
        is_fast_feasible = FALSE
        A_fast = INFINITY
        if t2_arrival > tp:
            t_fast = t2_arrival - tp
            A_fast = 2 * (a - V1 * t_fast) / (t_fast^2)
            is_fast_feasible = (abs(A_fast) <= A_max)
        // Determine the optimal feasible action
        if is_slow_feasible and is_fast_feasible:
            if abs(A_slow) < abs(A_fast):
                return A_slow
            else:
                return A_fast
        elif is_slow_feasible:
            return A_slow
        elif is_fast_feasible:
            return A_fast
        else:
            return NOT_FEASIBLE
'''


---


### Problem 5: Multi-Agent Coordinated Collision Avoidance

This problem evolves the scenario to a cooperative solution where both agents participate.

**Context:** This is a graduate-level problem in Multi-Agent Systems, Game Theory, or Distributed Optimization.

**Problem Formulation:** Determine the new pair of speeds $(V_1', V_2')$ that resolves the conflict while minimizing a joint cost function, such as the sum of squared velocity changes: $J = (V_1' - V_1)^2 + (V_2' - V_2)^2$.

**Mathematical Analysis:** This is a constrained optimization problem solved using Lagrange Multipliers. We minimize $J(V_1', V_2')$ subject to the safety constraint $|a/V_1' - b/V_2'| = t_p$. This leads to a system of nonlinear equations that can be solved numerically to find the optimal pair $(V_1', V_2')$ that satisfies one of the two boundary conditions while minimizing the cost $J$.

**Pseudocode for Coordinated Avoidance:**

'''python
    function CoordinatedAvoidance(a, b, V1, V2, tp):
        if not CheckIntervalCollision(a, b, V1, V2, tp):
            return (V1, V2)
        // Define objective and constraint functions for a numerical solver
        objective_function = lambda v_p: (v_p[0] - V1)^2 + (v_p[1] - V2)^2
        constraint1 = {'type': 'eq', 'fun': lambda v_p: a/v_p[0] - b/v_p[1] - tp}
        constraint2 = {'type': 'eq', 'fun': lambda v_p: b/v_p[1] - a/v_p[0] - tp}
        initial_guess = [V1, V2]
        // Call a numerical constrained optimization solver for both cases
        solution1 = solve_constrained_optimization(objective_function, constraint1, initial_guess)
        solution2 = solve_constrained_optimization(objective_function, constraint2, initial_guess)
        // Compare costs and return the optimal pair
        cost1 = objective_function(solution1.velocities)
        cost2 = objective_function(solution2.velocities)
        if cost1 <= cost2:
            return solution1.velocities
        else:
            return solution2.velocities
'''


---

### Problem 6: Collision Risk Assessment under State Uncertainty

This problem introduces the real-world element of uncertainty, shifting from a deterministic to a probabilistic outcome.

**Context:** This is a graduate-level problem central to Probabilistic Robotics, Stochastic Control, and Estimation Theory.

**Problem Formulation:** The initial state variables $(a, b, V_1, V_2)$ are modeled as independent normal (Gaussian) random variables with known means and variances. The task is to compute the probability of collision, $P(\text{collision})$, where collision is defined by $|a/V_1 - b/V_2| \le t_p$.

**Mathematical Analysis:** An exact analytical solution is intractable. The standard approach is **Monte Carlo Simulation**. A large number, $N$, of random state vectors are sampled from their respective distributions. For each sample, the collision condition is checked. The estimated probability is the ratio of the number of samples resulting in a collision to the total number of valid samples.
$$P(\text{collision}) \approx \frac{N_{\text{collision}}}{N_{\text{valid}}}$$

**Pseudocode for Monte Carlo Collision Probability Estimation:**

'''json
    function EstimateCollisionProbability(state_params, tp, num_samples):
        // state_params: structure containing means and variances for a, b, V1, V2
        collision_count = 0
        valid_samples = 0
        for i from 1 to num_samples:
            a_sample = draw_from_normal(state_params.mu_a, state_params.sigma_a)
            b_sample = draw_from_normal(state_params.mu_b, state_params.sigma_b)
            V1_sample = draw_from_normal(state_params.mu_V1, state_params.sigma_V1)
            V2_sample = draw_from_normal(state_params.mu_V2, state_params.sigma_V2)
            if V1_sample > 0 and V2_sample > 0 and a_sample > 0 and b_sample > 0:
                valid_samples += 1
                t1 = a_sample / V1_sample
                t2 = b_sample / V2_sample
                if abs(t1 - t2) <= tp:
                    collision_count += 1
        if valid_samples == 0:
            return 0.0
        return collision_count / valid_samples
'''


