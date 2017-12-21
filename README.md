# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Description
This repo contains the implementation of a Model-Predictive Control (MPC) in C++. The code takes input from a car simulator, computes the optimal solution to make the car follow a road and output the control signals (steering angle and throttle) to the simulator to actually control the car.

Model-Predictive Control is an advanced control technique that uses a model of the object to control to predict the effect of the change in the variables and so tackle the _lag_ problem: the control signals don't affect the car immediately but after certain time has passed. If this is not taken into account, the controller will always work with old data.

As explained in [1]:
>Model Predictive Control (MPC) is an advanced method of process control that has been in use in the process industries in chemical plants and oil refineries since the 1980s. MPC -also known as Receding horizon control or Moving horizon control- uses an explicit dynamic plant model to predict the effect of future reactions of the manipulated variables on the output and the control signal obtained by minimizing the cost function. This predication takes into account, constraints on both the inputs and outputs of the process. An optimal input sequence is calculated. The measurements are then sent back to the controller, and a new optimizing problem is solved.

![Block diagram of an MPC controller][image1]

In the case of the project, 
* the **plant** is the car
* the **manipulated variables** are the steering angle and the throttling

The main control loop proceeds as follows:

1. the current state of the vehicle is read from the simulator. The state includes the 2D position (_x_, _y_) as well as the actual linear velocity _v_, the orientation _psi_, and the errors _cte_ (cross-track error, or distance from the center line of the road) and _ePsi_ (difference from the desired orientation).

This state comes from the simulator in global coordinates, and it is much more simpler to work if they are in car frame's, so we convert them

```c++
// j[1] is the data JSON object that comes from the simulator

  // actual waypoints in global coordinates
  vector<double> ptsx = j[1]["ptsx"];
  vector<double> ptsy = j[1]["ptsy"];
  // actual pose of the car
  double px = j[1]["x"];
  double py = j[1]["y"];
  double psi = j[1]["psi"];
  double v = j[1]["speed"];

  // these vectors will store the waypoints in car's frame
  vector<double> ptsx_car;
  vector<double> ptsy_car;

  for (int i = 0; i < ptsx.size(); i++) {
    double dx = ptsx[i] - px;
    double dy = ptsy[i] - py;
    ptsx_car.push_back(dx * cos(-psi) - dy * sin(-psi));
    ptsy_car.push_back(dx * sin(-psi) + dy * cos(-psi));
  }
```

2. Using the state, we fit a polynomial of degree 3 to them. This will allow us to interpolate points along the trajectory:

```c++
  // polyfit works with Eigen::VectorXd structures, so convert from the simple vector<double>
  Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptsx_car.data(), 6);
  Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptsy_car.data(), 6);

  auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
```

Next, we compute the cross-track error and the error in orientation of the vehicle using the polynomial: the cross-track error is the value of the polynomial at 0, and the error of the orientation is the arctan of the 2nd order coefficient:

```c++
  double cte = polyeval(coeffs, 0);
  double epsi = -atan(coeffs[1]);
```

We change the sign of the arctan because the simulator flip the sides.

3. the state is fed into the solver, which computes the cost associated and tries to minimize it using the model equations, while respecting the constraints:

```c++
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // number of variables and constraints
  size_t n_vars = N * 6 + (N - 1) * 2;
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;

  vector<double> result;
  // actuators for the first waypoint
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  // add the rest of the computed points, to show in the simulator
  for (int i = 0; i < N-1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}
```

Of paramount importance here is the _cost function_. The solver will compute the values of the actuators that minimize this function, so it will determine the behaviour of the controller.
After several tries, I settle for the following cost function:

![cost function][costfn]

Where

* _cte_ is the cross-track error
* _ePsi_ is the error in orientation
* _v_ is the linear velocity
* _lambda_ is the steering angle
* _a_ is the linear acceleration

The factors that multiply each term give the relative weight of that term: the larger the coefficient, the larger the importance of that term on the cost.
Tuning those factors we can get the solver to perform better on different circumstances. For the chosen reference speed (60), the following values give a good performance across the whole circuit:

```c++
const double cCte = 3000;
const double cEig = 3000;
const double cVel = 5;
const double cDelta = 5;
const double cAccel = 5;
const double cVDelta = 1000;
const double cSuddenDelta = 200;
const double cSuddenAccel = 10;
```

4. the result of the solver is a vector of the values that minimizes the cost. This vector contains the values of the actuators (steering angle, throttle) as well as the waypoint coordinates computed.

![control equations][image2]

5. the result of the solver is returned to the main loop where it is fed into the simulator. The actuators are used to control the vehicle, while the waypoints are displayed along with the reference trajectory (in main.cpp):

```c++
  Eigen::VectorXd state(6);
  state << 0, 0, 0, v, cte, epsi;
  auto vars = mpc.Solve(state, coeffs);
  // result from solving the optimization problem
  steer_value = vars[0];
  throttle_value = vars[1];
  json msgJson;
  // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
  // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
  msgJson["steering_angle"] = steer_value / 0.436332;
  msgJson["throttle"] = throttle_value;
```

The reference trajectory is plotted in yellow, while the computed one is displayed in green:

![Left curve][curva_izquierda]

![Right curve][curva_derecha]

## Important points
* The number of points that are calculated by the solver determines the adjustment to the trajectory. But more points imply more computation so it is a balance. Also, the more points are calculated, the far in time we are looking; that will be detrimental because as time passes the conditions change and the computed trajectory is not valid anymore.
I tried computing 20 points at 0.1 sec interval (double than the actual value) and got a large instability after the curves. The car overcompensated and started oscillating until it went out of the route.
Also tried reducing the time spacing of the points. In this case I got a different effect that leaded to the same result: the car was unable to follow the route.
Some of these bad effects can be ameliorated by choosing other weights for the cost function, but the best result was obtained with the actual values in the code.
* Playing with the different weights we get different reactions of the car. The solver tries to minimize different functions, giving more importance to one aspect or the other according to the weights. For example, if the velocity term weight is large, the solver will try to keep the velocity and eventually will be thrown out of the route in a curve. The same with the term that expresses the correlation between the velocity and the steering angle.
* the model has a delay of 100 ms, implemented explicitly in main.cpp. This attempts to account for the normal delay we'll get in a real car. If we reduce the time between the waypoints, it may happen that the controller will try to compensate for a change that does not happen yet due to the delay, leading to instability. And if the period between the waypoints is too large, the compensation will be for a point that may not be in the real path, specially in tight curves where the position changes faster. The value of 0.1 sec matches the delay and thus produces a good result.

## Conclusion
This project is very interesting and challenging. The math of the control model is heavy (we don't have to worry about that, thankfully!) but the results are worth the complexity: the car behaves in a way similar of what we'll get with a human driver.

One aspect I didn't have time to put to trial is the generality of the parameters of the cost function. It would be nice to run the same code in a different circuit, with different difficulties. If I find time, I would like to try to fit this code to another of the circuits in the simulator. It would be nice if that possibility is contemplated in future editions of the course.



## References
[1]: *Design and Development of Model Predictive Controller for Binary Distillation Column*. R. Sivakumar, Shennes Mathew. International Journal of Science and Research (IJSR). ISSN (Online): 2319-7064


[image1]: ./images/mpc_blocks.png "Block diagram of an MPC controller"
[image2]: ./images/ecuaciones.png "Ecuaciones de control"
[curva_izquierda]: ./images/curva_izquierda.png "Left curve"
[curva_derecha]: ./images/curva_derecha.png "Right curve"
[costfn]: ./images/cost_function.png "Cost function"