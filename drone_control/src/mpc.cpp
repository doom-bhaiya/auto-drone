// #include <casadi/casadi.hpp>
// #include <vector>
// #include <iostream>
// #include <chrono>
// #include <Eigen/Dense>

// Eigen::Matrix4d A;


// using namespace casadi;

// MX getRotationMatrix(const MX& state) {
//     MX theta = state(0);
//     MX phi   = state(1);
//     MX omega = state(2);

//     MX R_x = MX::vertcat({
//         MX::horzcat({1, 0, 0}),
//         MX::horzcat({0, cos(theta), -sin(theta)}),
//         MX::horzcat({0, sin(theta), cos(theta)})
//     });

//     MX R_y = MX::vertcat({
//         MX::horzcat({cos(phi), 0, sin(phi)}),
//         MX::horzcat({0, 1, 0}),
//         MX::horzcat({-sin(phi), 0, cos(phi)})
//     });

//     MX R_z = MX::vertcat({
//         MX::horzcat({cos(omega), -sin(omega), 0}),
//         MX::horzcat({sin(omega), cos(omega), 0}),
//         MX::horzcat({0, 0, 1})
//     });

//     return mtimes(R_z, mtimes(R_y, R_x));
// }

// int main() {

//     float kf = 0.000001;
//     float km = 0.0000001;
//     float length_of_prop = 0.13181;

//     A <<  kf,  kf,  kf,  kf,
//     -length_of_prop * kf, 0,  length_of_prop * kf,  0,
//     0,  -length_of_prop * kf,  0, length_of_prop * kf,
//     km,  -km, km,  -km;


//     Opti opti;

//     int N = 10;
//     float dt = 0.01;

//     MX mass = opti.parameter();
//     MX g = opti.parameter();
//     opti.set_value(mass, 1.0);
//     opti.set_value(g, 9.81);

//     MX Inertial_matrix = opti.parameter(3, 3);
//     DM I_val = DM::eye(3); 
//     opti.set_value(Inertial_matrix, I_val);

//     MX Xref_dm = opti.parameter(3, N);
//     for (int k = 0; k < N; ++k) {
//         opti.set_value(Xref_dm(0, k), 0.1 * k );
//         opti.set_value(Xref_dm(1, k), -0.1 * k);
//         opti.set_value(Xref_dm(2, k), k + 1.0);
//     }

//     MX X0 = opti.parameter(12, 1);
//     opti.set_value(X0, DM::zeros(12, 1));
//     opti.set_value(X0(2), 1.0);

//     MX X = opti.variable(4 * 3, N);
//     MX u = opti.variable(4, N);
//     MX cost = MX(0);

//     for (int k = 0; k < N - 1; ++k) {

//         MX g_vector = MX::vertcat({0, 0, -g * mass});
//         MX prop_force = MX::vertcat({0, 0, u(0, k)});
//         MX rotMat = getRotationMatrix(X(Slice(6, 9), k));

//         MX lin_acc = (g_vector + mtimes(rotMat, prop_force)) / mass;
//         MX ang_vel = X(Slice(9, 12), k);

//         MX Iw = mtimes(Inertial_matrix, ang_vel);
//         MX cross_term = cross(ang_vel, Iw);

//         MX moment = u(Slice(1, 4), k);
//         MX ang_acc = solve(Inertial_matrix, moment - cross_term);

//         X(Slice(0, 3), k+1) = X(Slice(0, 3), k) + X(Slice(3, 6), k) * dt;
//         X(Slice(3, 6), k+1) = X(Slice(3, 6), k) + lin_acc * dt;
//         // X(Slice(6, 9), k+1) = X(Slice(6, 9), k);
//         // X(Slice(9, 12), k+1) = X(Slice(9, 12), k);

//         X(Slice(6, 9), k+1) = mtimes(inv(rotMat), X(Slice(6, 9), k)) + ang_vel * dt;
//         X(Slice(9, 12), k+1) = X(Slice(9, 12), k) + ang_acc * dt;

//         cost += sumsqr(X(Slice(0, 3), k) - Xref_dm(Slice(), k));
//     }

//     opti.subject_to(X(Slice(), 0) == X0);
//     opti.subject_to(X(2, Slice()) >= 0);
//     // opti.subject_to(0 <= solve(A, u) <= 10);

//     opti.minimize(cost);



//     Dict opts;
//     opts["ipopt.print_level"] = 0;
//     opts["print_time"] = 0;
//     opti.solver("ipopt", opts);

//     Function solver = opti.to_function("mpc_solver", {X0, Xref_dm, Inertial_matrix}, {u});

//     auto start = std::chrono::high_resolution_clock::now();
    
//     for (int i = 0; i < 100; i++){
//         auto sol = opti.solve();
//         auto lam_g0 = sol.value(u);
//         auto lam_x0 = sol.value(X);
//         auto result = sol.value(sumsqr(X(Slice(0, 3), Slice()) - Xref_dm));
//         // std::cout << result(Slice(), 0) << "\n";
//         // std::cout << lam_g0(Slice(), 0) << "\n";
//         std::cout << "X Axis : " << lam_x0(Slice(0), Slice()) << "\n";
//         std::cout << "Y Axis : " << lam_x0(Slice(1), Slice()) << "\n";
//         std::cout << "Z Axis : " << lam_x0(Slice(2), Slice()) << "\n";
//     }

//     auto end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed = end - start;

//     std::cout << "MPC solve time: " << elapsed.count() / 100 << " seconds" << std::endl;
//     return 0;
// }



#include <casadi/casadi.hpp>
#include <vector>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>

Eigen::Matrix4d A;

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

int main() {
    float kf = 0.000001;
    float km = 0.0000001;
    float length_of_prop = 0.13181;

    A <<  kf,  kf,  kf,  kf,
         -length_of_prop * kf, 0,  length_of_prop * kf,  0,
          0,  -length_of_prop * kf,  0, length_of_prop * kf,
          km,  -km, km,  -km;

    A << A.inverse();



    Opti opti;
    int N = 10;
    float dt = 0.01;

    DM A_val = DM(4, 4);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            A_val(i, j) = A(i, j);

    MX mass = opti.parameter();
    MX g = opti.parameter();
    opti.set_value(mass, 1.0);
    opti.set_value(g, 9.81);

    MX Inertial_matrix = opti.parameter(3, 3);
    DM I_val = DM::eye(3); 
    opti.set_value(Inertial_matrix, I_val);

    MX Xref_dm = opti.parameter(3, N);
    for (int k = 0; k < N; ++k) {
        opti.set_value(Xref_dm(0, k), 0.0001 * k);
        opti.set_value(Xref_dm(1, k), -0.0001 * k);
        opti.set_value(Xref_dm(2, k), 0.001 * k + 1.0);
    }

    MX X0 = opti.parameter(12, 1);
    opti.set_value(X0, DM::zeros(12, 1));
    opti.set_value(X0(2), 1.0);

    MX X = opti.variable(12, N);
    MX u = opti.variable(4, N);
    MX cost = MX(0);

    for (int k = 0; k < N - 1; ++k) {
        MX g_vector = MX::vertcat({0, 0, -g * mass});
        MX prop_force = MX::vertcat({0, 0, u(0, k)});
        MX rotMat = getRotationMatrix(X(Slice(6, 9), k));

        MX lin_acc = (g_vector + mtimes(rotMat, prop_force)) / mass;
        MX ang_vel = X(Slice(9, 12), k);
        MX Iw = mtimes(Inertial_matrix, ang_vel);
        MX cross_term = cross(ang_vel, Iw);

        MX moment = u(Slice(1, 4), k);
        MX ang_acc = solve(Inertial_matrix, moment - cross_term);

        X(Slice(0, 3), k+1) = X(Slice(0, 3), k) + X(Slice(3, 6), k) * dt;
        X(Slice(3, 6), k+1) = X(Slice(3, 6), k) + lin_acc * dt;
        X(Slice(6, 9), k+1) = mtimes(inv(rotMat), X(Slice(6, 9), k)) + ang_vel * dt;
        X(Slice(9, 12), k+1) = X(Slice(9, 12), k) + ang_acc * dt;

        cost += sumsqr(X(Slice(0, 3), k) - Xref_dm(Slice(), k)) + 0.00000000000000000001 * sumsqr(u(Slice(), k));
    }

    opti.subject_to(X(Slice(), 0) == X0);
    opti.subject_to(X(2, Slice()) >= 0);
    opti.subject_to(0 < u(0, Slice()));
    // opti.subject_to(0 < (mtimes(A_val, u)) < 2000000);


    opti.minimize(cost);

    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.tol"] = 1e-2;
    opts["ipopt.constr_viol_tol"] = 1e-2;
    opts["ipopt.acceptable_tol"] = 1e-2;
    opts["ipopt.max_iter"] = 50;
    opti.solver("ipopt", opts);

    Function mpc_solver = opti.to_function("mpc_solver", {X0, Xref_dm, Inertial_matrix}, {u, X});

    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 100; ++i) {
        std::vector<DM> result = mpc_solver(std::vector<DM>{
            opti.value(X0),
            opti.value(Xref_dm),
            I_val
        });
        
        DM u_opt = result[0];
        DM X_opt = result[1];

        std::cout << "X Axis : " << X_opt(Slice(0), Slice()) << "\n";
        std::cout << "Y Axis : " << X_opt(Slice(1), Slice()) << "\n";
        std::cout << "Z Axis : " << X_opt(Slice(2), Slice()) << "\n";
        std::cout << "Thrust_values : " << u_opt << "\n";
        std::cout << "Prop values : " << sqrt(mtimes(A_val, u_opt)) << "\n";
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Average solve time: " << elapsed.count() / 100 << " seconds" << std::endl;
    return 0;
}
