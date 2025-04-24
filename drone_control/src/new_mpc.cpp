#include <casadi/casadi.hpp>
#include <vector>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>

using namespace casadi;

MX getRotationMatrix(const MX& state) {
    MX theta = state(0);
    MX phi   = state(1);
    MX omega = state(2);

    MX R_x = MX::vertcat({
        MX::horzcat({1, 0, 0}),
        MX::horzcat({0, cos(theta), -sin(theta)}),
        MX::horzcat({0, sin(theta), cos(theta)})
    });

    MX R_y = MX::vertcat({
        MX::horzcat({cos(phi), 0, sin(phi)}),
        MX::horzcat({0, 1, 0}),
        MX::horzcat({-sin(phi), 0, cos(phi)})
    });

    MX R_z = MX::vertcat({
        MX::horzcat({cos(omega), -sin(omega), 0}),
        MX::horzcat({sin(omega), cos(omega), 0}),
        MX::horzcat({0, 0, 1})
    });

    return mtimes(R_z, mtimes(R_y, R_x));
}

int main(){

    Opti opti;
    float kf = 0.000001;
    float km = 0.0000001;
    float length_of_prop = 0.13181;

    Eigen::Matrix4d A;
    A <<  kf,  kf,  kf,  kf,
    -length_of_prop * kf, 0,  length_of_prop * kf,  0,
    0,  -length_of_prop * kf,  0, length_of_prop * kf,
    km,  -km, km,  -km;

    SX mass = SX::sym("mass");
    SX g = SX::sym("g");
    SX dt = SX::sym("dt");
    SX Inertial_matrix = SX::sym("I", 3, 3);

    int N = 10;

    MX X0 = opti.parameter(12, 1);
    opti.set_value(X0, DM::zeros(12, 1));
    opti.set_value(X0(2), 1.0);

    MX X = opti.variable(4 * 3, N);
    MX u = opti.variable(4, N);
    MX cost = MX(0);

    for (int k = 0; k < N - 1; ++k) {

        MX g_vector = MX::vertcat({MX(0), MX(0), MX(-g * mass)});
        MX prop_force = MX::vertcat({0, 0, u(0, k)});
        MX rotMat = getRotationMatrix(X(Slice(6, 9), k));

        MX lin_acc = (g_vector + mtimes(rotMat, prop_force)) / mass;
        MX ang_vel = X(Slice(9, 12), k);

        // MX Iw = mtimes(Inertial_matrix, ang_vel);
        // MX cross_term = cross(ang_vel, Iw);

        // MX moment = u(Slice(1, 4), k);
        // MX ang_acc = solve(Inertial_matrix, moment - cross_term);

        X(Slice(0, 3), k+1) = X(Slice(0, 3), k) + X(Slice(3, 6), k) * dt;
        X(Slice(3, 6), k+1) = X(Slice(3, 6), k) + lin_acc * dt;
        // X(Slice(6, 9), k+1) = mtimes(inv(rotMat), X(Slice(6, 9), k)) + ang_vel * dt;
        // X(Slice(9, 12), k+1) = X(Slice(9, 12), k) + ang_acc * dt;

        cost += sumsqr(X(Slice(0, 3), k) - Xref_dm(Slice(), k)) + 0.1 * sumsqr(u(Slice(), k));
    }


    opti.subject_to(X(Slice(), 0) == X0);
    opti.subject_to(X(2, Slice()) >= 0);
    opti.subject_to(0 <= solve(A, u) <= 10);
    opti.minimize(cost);

    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opti.solver("ipopt", opts);

    Function solver = opti.to_function("mpc_solver", {X0, Xref_dm, Inertial_matrix}, {u});

    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 100; i++){
        auto sol = opti.solve();
        auto lam_g0 = sol.value(u);
        auto result = sol.value(sumsqr(X(Slice(0, 3), Slice()) - Xref_dm));
        std::cout << result(Slice(), 0) << "\n";
        std::cout << lam_g0(Slice(), 0) << "\n";
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "MPC solve time: " << elapsed.count() / 100 << " seconds" << std::endl;
    return 0;
    
}