#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros2/ros_conversions.h>

bool SfcOptimizer::solveMinimumSnapQP(const vec_Vecf<3>& eigen_path, 
                                      double speed, 
                                      std::vector<std::vector<double>>& final_traj) {
    using namespace mav_trajectory_generation;

    // 1. Setup the optimization vertices (Waypoints)
    Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = Derivative::SNAP;

    for (const auto& point : eigen_path) {
        Vertex v(dimension);
        v.addConstraint(Derivative::POSITION, point);
        vertices.push_back(v);
    }

    // 2. Estimate initial segment times based on desired speed
    std::vector<double> segment_times = estimateSegmentTimes(vertices, speed, 10.0);

    // 3. Perform the optimization
    // This minimizes the integral of the squared 4th derivative (Snap)
    const int N = 10; // Polynomial order (10 is standard for Snap)
    PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solve();

    // 4. Sample the trajectory at 100Hz for your PID controller
    Trajectory trajectory;
    opt.getTrajectory(&trajectory);

    double sampling_interval = 0.01; // 100Hz
    for (double t = 0.0; t <= trajectory.getMaxTime(); t += sampling_interval) {
        Eigen::Vector3d pos = trajectory.evaluate(t, Derivative::POSITION);
        final_traj.push_back({pos.x(), pos.y(), pos.z()});
    }

    return true;
}