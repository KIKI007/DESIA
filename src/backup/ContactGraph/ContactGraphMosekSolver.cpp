//
// Created by ziqwang on 14.01.19.
//

#include "ContactGraphMosekSolver.h"

ContactGraphMosekSolver::ContactGraphMosekSolver(double eps)
{
    zero_eps = eps;
}

void ContactGraphMosekSolver::computeTranslationalInterlockingMatrix(mosek::fusion::Matrix::t &mat, Eigen::Vector2i &size)
{
    vector<EigenTriple> triplist;
    ContactGraph::computeTranslationalInterlockingMatrix(triplist, size);

    size_t n_value = triplist.size();
    shared_ptr<monty::ndarray<int, 1>> row_index (new monty::ndarray<int,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].row();}));
    shared_ptr<monty::ndarray<int, 1>> col_index (new monty::ndarray<int,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].col();}));
    shared_ptr<monty::ndarray<double, 1>> m_content (new monty::ndarray<double,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].value();}));
    mat = mosek::fusion::Matrix::sparse(size[0], size[1], row_index, col_index, m_content);
    return;
}

void ContactGraphMosekSolver::computeRotationalInterlockingMatrix(mosek::fusion::Matrix::t &mat, Eigen::Vector2i &size) {
    vector<EigenTriple> triplist;
    ContactGraph::computeRotationalInterlockingMatrix(triplist, size);

    size_t n_value = triplist.size();
    shared_ptr<monty::ndarray<int, 1>> row_index (new monty::ndarray<int,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].row();}));
    shared_ptr<monty::ndarray<int, 1>> col_index (new monty::ndarray<int,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].col();}));
    shared_ptr<monty::ndarray<double, 1>> m_content (new monty::ndarray<double,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].value();}));
    mat = mosek::fusion::Matrix::sparse(size[0], size[1], row_index, col_index, m_content);
    return;
}

bool ContactGraphMosekSolver::isTranslationalInterlocking(string &log)
{
    /////////////////////////////////////////////////////////
    // 1. Build linear programming matrix
    Eigen::Vector2i size;
    mosek::fusion::Matrix::t A;
    computeTranslationalInterlockingMatrix(A, size);

    /////////////////////////////////////////////////////////
    // 2. Set Constraints

    mosek::fusion::Model::t M = new mosek::fusion::Model("isTranslationalInterlocking");
    auto _M = monty::finally([&]() { M->dispose(); });
    auto x = M->variable(size[1], mosek::fusion::Domain::unbounded());
    auto t = M->variable(size[0], mosek::fusion::Domain::inRange(0.,10.));

    M -> constraint("no collision constraint", mosek::fusion::Expr::sub(mosek::fusion::Expr::mul(A, x), t), mosek::fusion::Domain::greaterThan(0.0));

    /////////////////////////////////////////////////////////
    // 3. Solve the linear programming

    M -> objective("obj", mosek::fusion::ObjectiveSense::Maximize, mosek::fusion::Expr::sum(t));
    M -> setSolverParam("intpntCoTolRelGap", 1e-12);
    //M ->setLogHandler([=](const std::string &msg) { std::cout << msg << std::flush; });
    M -> solve();

    if(M->getProblemStatus(mosek::fusion::SolutionType::Interior) != mosek::fusion::ProblemStatus::PrimalAndDualFeasible)
    {
        return -1;
    }

    /////////////////////////////////////////////////////////
    // 4. Analysis optimization result
    auto aux_var = t->level();
    double aux_sum = 0;
    for(auto& num : *aux_var) aux_sum += num;
    std::cout << "Total Sum:\t" << aux_sum << std::endl;
    if (aux_sum > zero_eps)
        return false;
    return true;
}

bool ContactGraphMosekSolver::isRotationalInterlocking(string &log) {
    /////////////////////////////////////////////////////////
    // 1. Build linear programming matrix
    Eigen::Vector2i size;
    mosek::fusion::Matrix::t A;
    computeRotationalInterlockingMatrix(A, size);

    /////////////////////////////////////////////////////////
    // 2. Set Constraints

    mosek::fusion::Model::t M = new mosek::fusion::Model("isRotationalInterlocking");
    auto _M = monty::finally([&]() { M->dispose(); });
    auto x = M->variable(size[1], mosek::fusion::Domain::unbounded());
    auto t = M->variable(size[0], mosek::fusion::Domain::inRange(0.,10.));

    M -> constraint("no collision constraint", mosek::fusion::Expr::sub(mosek::fusion::Expr::mul(A, x), t), mosek::fusion::Domain::greaterThan(0.0));

    /////////////////////////////////////////////////////////
    // 3. Solve the linear programming

    M -> objective("obj", mosek::fusion::ObjectiveSense::Maximize, mosek::fusion::Expr::sum(t));
    M -> setSolverParam("intpntCoTolRelGap", 1e-12);
    M ->setLogHandler([=](const std::string &msg) { std::cout << msg << std::flush; });
    M -> solve();

    if(M->getProblemStatus(mosek::fusion::SolutionType::Interior) != mosek::fusion::ProblemStatus::PrimalAndDualFeasible)
    {
        return -1;
    }

    /////////////////////////////////////////////////////////
    // 4. Analysis optimization result
    auto aux_var = t->level();
    double aux_sum = 0;
    for(auto& num : *aux_var) aux_sum += num;
    std::cout << "Total Sum:\t" << aux_sum << std::endl;
    if (aux_sum > zero_eps)
        return false;
    return true;
}
