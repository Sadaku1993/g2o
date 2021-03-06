#include <iostream>
#include <cmath>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;
using namespace g2o;

int main()
{
    std::cout<<"start"<<std::endl;

    // bfr.csv and aft.csv is saved at home directory
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string bfr_file_path = std::string(homedir) + "/bfr.csv";
    std::string aft_file_path = std::string(homedir) + "/aft.csv";
    
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    SparseOptimizer optimizer;
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);

    optimizer.load(bfr_file_path.c_str());
    // optimizer.load(file_path);

    // prepare and run the optimization
    // fix the first robot pose to account for gauge freedom
    VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);
    optimizer.setVerbose(true);

    std::cerr << "Optimizing" << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::cerr << "done." << std::endl;

    optimizer.save(aft_file_path.c_str());
    // optimizer.save("aft.csv");

    // freeing the graph memory
    optimizer.clear();

    // destoroy all the singletons
    // Factory::destroy();
    // OptimizationAlgorithmFactory::destroy();

    return 0;
}
