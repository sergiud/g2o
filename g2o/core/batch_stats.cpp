// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/batch_stats.h>
#include <cstring>

G2O_START_NAMESPACE
  using namespace std;

  G2OBatchStatistics* G2OBatchStatistics::_globalStats=nullptr;

  #ifndef PTHING
  #define PTHING(s) \
    #s << "= " << (st.s) << "\t "
  #endif

  G2OBatchStatistics::G2OBatchStatistics(){
    // zero all.
    memset (this, 0, sizeof(G2OBatchStatistics));

    // set the iteration to -1 to show that it isn't valid
    iteration = -1;
  }

  std::ostream& operator << (std::ostream& os , const G2OBatchStatistics& st)
  {
    os << PTHING(iteration);

    os << PTHING( numVertices ); // how many vertices are involved
    os << PTHING( numEdges ); // hoe many edges
    os << PTHING(  chi2 );  // total chi2
    
    /** timings **/
    // nonlinear part
    os << PTHING(  timeResiduals );  
    os << PTHING(  timeLinearize );   // jacobians
    os << PTHING(  timeQuadraticForm ); // construct the quadratic form in the graph
    
    // block_solver (constructs Ax=b, plus maybe schur);
    os << PTHING(  timeSchurComplement ); // compute schur complement (0 if not done);
    
    // linear solver (computes Ax=b); );
    os << PTHING(  timeSymbolicDecomposition ); // symbolic decomposition (0 if not done);
    os << PTHING(  timeNumericDecomposition ); // numeric decomposition  (0 if not done);
    os << PTHING(  timeLinearSolution );             // total time for solving Ax=b
    os << PTHING(  iterationsLinearSolver );  // iterations of PCG
    os << PTHING(  timeUpdate ); // oplus
    os << PTHING(  timeIteration ); // total time );

    os << PTHING( levenbergIterations );
    os << PTHING( timeLinearSolver);

    os << PTHING(hessianDimension);
    os << PTHING(hessianPoseDimension);
    os << PTHING(hessianLandmarkDimension);
    os << PTHING(choleskyNNZ);
    os << PTHING(timeMarginals);

    return os;
  };

  void G2OBatchStatistics::setGlobalStats(G2OBatchStatistics* b)
  {
    _globalStats = b;
  }

G2O_END_NAMESPACE
