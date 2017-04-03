#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using namespace std;

class Tools {
public:
   /**
  * Debug stuff
  */ 
  static string tracelog;
  static bool trace_tag;
  static ofstream  traceStream;  
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};

class StringException : public std::exception
{
public:
  std::string s;
  StringException(std::string ss) : s(ss) {}
  ~StringException() throw () {} // Updated
  const char* what() const throw() { return s.c_str(); }
};
#endif /* TOOLS_H_ */
