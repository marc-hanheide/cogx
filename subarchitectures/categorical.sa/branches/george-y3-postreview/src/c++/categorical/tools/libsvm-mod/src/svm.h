#ifndef _LIBSVM_H
#define _LIBSVM_H

#include "predict.h"

/** Shows debug messages if defined. */
//#define DEBUG

/** Infinity. */
#define INF HUGE_VAL

/** More convenient malloc */
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))


#ifdef __cplusplus
extern "C" {
#endif


typedef signed char schar;



struct svm_node
{
  int index;
  double value;
};

struct svm_problem
{
  int l;  // Number of training samples
  double *y;  // Target values (class labels in case of classification)
  struct svm_node **x; // Pointers to sparse sample vectors
};

enum { C_SVC, NU_SVC, ONE_CLASS, EPSILON_SVR, NU_SVR, ONE_AGAINST_ALL };  /* svm_type */
enum { LINEAR, POLY, RBF, SIGMOID, LOCALKERNEL1, CHISQUARED, GENERALIZEDGAUSS, INTERSECTION, LOCALKERNEL2 };  /* kernel_type */

struct svm_parameter
{
  /** General params */
  int svm_type;
  int kernel_type;
  double degree;  /* for poly */
  double gamma; /* for poly/rbf/sigmoid */
  double coef0; /* for poly/sigmoid */

  /** local features parameters */
  int n_features;
  int featuredim;
  int nummax;
  double simthresh;
  int localKernelType;
  int posType;
  double sigmaPos;
  int distHistNeighbourhood;
  int distHistBins;

  /** fast sift local kernel params. */
  double distCompensationCoeff;
  int surfEnhancements;
  int surfLaplacians;

  /** these are for training only */
  double cache_size; /* in MB */
  double eps; /* stopping criteria */
  double C; /* for C_SVC, EPSILON_SVR and NU_SVR */
  int nr_weight;    /* for C_SVC */
  int *weight_label;  /* for C_SVC */
  double* weight;   /* for C_SVC */
  double nu;  /* for NU_SVC, ONE_CLASS, and NU_SVR */
  double p; /* for EPSILON_SVR */
  int shrinking;  /* use the shrinking heuristics */

  /** Parallel training */
  int trainOnlyClass; /* train only one class for parallelization */
  const char* tmpModelFilename; /* filename to store a temporary modelfile for parallelized training */

  /** RLDSV parameters */
  int RLDSV; /* reduce support vectors linearly dependent in feature space */
  double RLDSVthreshold;  /* threshold below which we treat values as if they were equal to zero */
  char RLDSValgorithm; /* algorithm: 1 - QR Factorization    2 - Gauss-Jordan Elimination */
  char RLDSVrankdef; /* numerical rank definition: 1 - ||R22||2    2 - rii    3 - inf(R11) */
  char RLDSVnormalize; /* normalize columns before factorization: 1 - No    2 - Yes */

};



/**
 * Parameters of a decision function.
 */
struct decision_function
{
  double *alpha;
  double rho;
};

typedef float Qfloat;


/**
 * The Kernel class used to compute the values of K(xi, xj).
 * Does not store the computed values, only provides tools to perform
 * the computations.
 */
class Kernel
{

public:

  /** Constructor. */
  Kernel(int l, svm_node * const * x, const svm_parameter& param);

  /** Destructor. */
  virtual ~Kernel();

  /** Performs single single kernel evaluation given two samples.
   * Used during prediction. */
  static double k_function(const svm_node *x, const svm_node *y,
         const svm_parameter& param);

  /** Gets one column from the Q Matrix. */
  virtual Qfloat *get_Q(int column, int len) const = 0;

  /** */
  virtual void swap_index(int i, int j) const; // no so const...


protected:

  /** Pointer to a function calculating K(xi, xj) during training. */
  double (Kernel::*kernel_function)(int i, int j) const;

private:

  /** A deep copy of the prob->x table. Contains pointers to the sparse
   * representations of the training samples. */
  const svm_node **x;

  /** Buffer containing precomputed dot(x[i],x[i]) to speedup gaussian kernel. */
  double *x_square;

  /** Some of the parameters. Kept separately for compatibility with the
   * original code. */
  const int kernel_type;
  const double degree;
  const double gamma;
  const double coef0;

  /** All parameters. */
  svm_parameter svmparam;

  /** Calculates the dot product. */
  static double dot(const svm_node *px, const svm_node *py);

  /** Functions computing K(xi,xj) during training. */
  double kernel_linear(int i, int j) const;
  double kernel_poly(int i, int j) const;
  double kernel_rbf(int i, int j) const;
  double kernel_sigmoid(int i, int j) const;
  double kernel_localkernel1(int i, int j) const;
  double kernel_localkernel2(int i, int j) const;
  double kernel_chiSquared(int i, int j) const;
  double kernel_generalizedGauss(int i, int j) const;
  double kernel_intersection(int i, int j) const;
};



/**
 * SVM model
 */
struct svm_model
{
  /** Parameters. */
  svm_parameter param;

  /** Number of classes, = 2 in regression/one class svm. */
  int nr_class;

  /** Number of hyperplanes (=nr_class for OaA and nr_class*(nr_class-1)/2 for OaO.*/
  int nr_hyp;

  /** Total no. of SVs. */
  int l;

  /** SVs (SV[l]). */
  svm_node **SV;

  /** Coefficients for SVs in decision functions (sv_coef[nr_class-1 (OaO) or nr_class (OaA)][l]). */
  double **sv_coef;

  /** Constants in decision functions (rho[nr_hyp]) */
  double *rho;

  /** ||w|| for each classifier (omegaNorm[nr_hyp]*/
  double *omegaNorm;

  /** Average distance of positive samples to the hyperplane
   * for each classifier (avgDistP[nr_hyp]*/
  double *avgDistP;

  /** Average distance of negative samples to the hyperplane
   * for each classifier (avgDistN[nr_hyp]*/
  double *avgDistN;

  /** Average value of the decision function for positive samples
   * for each classifier (avgFP[nr_hyp]*/
  double *avgFP;

  /** Average value of the decision function for negative samples
   * for each classifier (avgFN[nr_hyp]*/
  double *avgFN;


  /** For classification only */

  /** Label of each class (label[nr_class]) */
  int *label;

  /**
   * Number of SVs for each class (nSV[nr_class]).
   * nSV[0] + nSV[1] + ... + nSV[n-1] = l
   */
  int *nSV;

  /**
   * 1 if svm_model is created by svm_load_model
   * 0 if svm_model is created by svm_train
   */
  int free_sv;
};

/* </PRONOBIS> */



/**
* Generalized SMO+SVMlight algorithm
* Solves:
*
*  min 0.5(\alpha^T Q \alpha) + b^T \alpha
*
*    y^T \alpha = \delta
*    y_i = +1 or -1
*    0 <= alpha_i <= Cp for y_i = 1
*    0 <= alpha_i <= Cn for y_i = -1
*
* Given:
*
*  Q, b, y, Cp, Cn, and an initial feasible point \alpha
*  l is the size of vectors and matrices
*  eps is the stopping criterion
*
* solution will be put in \alpha, objective value will be put in obj
*/
class Solver {
public:

  /** Constructor. */
  Solver() {};

  /** Destructor. */
  virtual ~Solver() {};

  /** Stores some info about the computed solution. */
  struct SolutionInfo {
    double obj;
    double rho;
    double upper_bound_p;
    double upper_bound_n;
    double r; // for Solver_NU
    double omegaNorm; // ||w||=sqrt(wTw)
    double avgDistP; // Average distance between the positive samples and the hyperplane
    double avgDistN; // Average distance between the negative samples and the hyperplane
    double avgFP; // Average value of the decision function for positive samples
    double avgFN; // Average value of the decision function for negative samples
  };

  /** Solves the optimization problem. */
  void Solve(int l, const Kernel& Q, const double *b_, const schar *y_,
       double *alpha_, double Cp, double Cn, double eps,
       SolutionInfo* si, const svm_parameter* param);


protected:
  int active_size;
  schar *y;
  double *G;    // gradient of objective function
  enum { LOWER_BOUND, UPPER_BOUND, FREE };
  char *alpha_status; // LOWER_BOUND, UPPER_BOUND, FREE
  double *alpha;
  const Kernel *Q;
  double eps;
  double Cp,Cn;
  double *b;
  int *active_set;
  double *G_bar;    // gradient, if we treat free variables as 0
  int l;
  bool unshrinked;  // XXX


  /** Returns C for the sample no. i */
  inline double get_C(int i);

  inline void update_alpha_status(int i);

  bool is_upper_bound(int i) { return alpha_status[i] == UPPER_BOUND; }
  bool is_lower_bound(int i) { return alpha_status[i] == LOWER_BOUND; }
  bool is_free(int i) { return alpha_status[i] == FREE; }
  void swap_index(int i, int j);
  void reconstruct_gradient();
  virtual int select_working_set(int &i, int &j);
  virtual double calculate_rho();
  virtual void do_shrinking();
};





struct svm_model *svm_train(const struct svm_problem *prob,
          const struct svm_parameter *param);

int svm_save_model(const char *model_file_name, const struct svm_model *model);

struct svm_model *svm_load_model(const char *model_file_name, bool noSVs=false);

void svm_reorganize_model(struct svm_model *model, const int* classOrder);

// bool svm_predict(const svm_model *model, const svm_node *x,
//                  const predict_parameter *pred_param, double target,
//                  double *v, int *classes=0, double *confidence=0);


void svm_destroy_model(struct svm_model *model);

const char *svm_check_parameter(const struct svm_problem *prob, const struct svm_parameter *param);

void saveVoteMatrix(const svm_model* model, char* filename);

#ifdef __cplusplus
}
#endif

#include "localkernels.h"
#include "somekernels.h"

#endif /* _LIBSVM_H */
