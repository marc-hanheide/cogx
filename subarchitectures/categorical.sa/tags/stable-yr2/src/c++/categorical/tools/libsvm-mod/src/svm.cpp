#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <float.h>
#include <string.h>
#include <stdarg.h>
#include "svm.h"
#include "allocMatrix.h"
#include "confidence.h"
#include "predict.h"
#include "localkernel2.h"

// <PRONOBIS 2005-02-10>
#include "svreduction.h"

// </PRONOBIS>

#ifndef min
template <class T> inline T min(T x,T y) { return (x<y)?x:y; }
#endif
#ifndef max
template <class T> inline T max(T x,T y) { return (x>y)?x:y; }
#endif
template <class T> inline void swap(T& x, T& y) { T t=x; x=y; y=t; }
template <class S, class T> inline void clone(T*& dst, S* src, int n)
{
  dst = new T[n];
  memcpy((void *)dst,(void *)src,sizeof(T)*n);
}
#if 1
void info(const char *fmt,...)
{
  va_list ap;
  va_start(ap,fmt);
  vprintf(fmt,ap);
  va_end(ap);
}
void info_flush()
{
  fflush(stdout);
}
#else
void info(char *fmt,...) {}
void info_flush() {}
#endif


/**
 * Kernel Cache. It is an Least Recently Used (LRU) cache.
 * Description of the cache algorithm:
 * LRU discards the least recently used items first. This algorithm requires keeping track
 * of what was used when, which is expensive if one wants to make sure the algorithm always
 * discards the least recently used item. If a probabilistic scheme that almost always
 * discards one of the least recently used items is sufficient, the Pseudo-LRU algorithm
 * can be used which only needs one bit per cache item to work.
 */
class Cache
{
public:

  /**
   * Constructor. Creates an empty circular list.
   * l - the number of total data items (not main param->l!)
   * size - cache size limit in bytes
   */
  Cache(int l,int size);

  /** Destructor. Destroys the cache and all its data.*/
  ~Cache();

  /** request data [0,len)
  * return some position p where [p,len) need to be filled
  * (p >= len if nothing needs to be filled) */
  int get_data(const int index, Qfloat **data, int len);

  /** */
  void swap_index(int i, int j);  // future_option

private:

  /** Number of total data items. */
  int l;

  /** */
  int size;

  /** Structure for creating a circular list. */
  struct head_t
  {
    /** Pointers to the next and previous item of the circular list. */
    head_t *prev, *next;

    /** Column of the Q matrix. */
    Qfloat *data;

    /** In this item data[0,len) is cached. */
    int len;
  };

  head_t* head;
  head_t lru_head;
  void lru_delete(head_t *h);
  void lru_insert(head_t *h);
};

Cache::Cache(int l_,int size_):l(l_),size(size_)
{
  #ifdef DEBUG
  printf(">>> Constructing Cache class (l_=%d, size_=%d).\n", l_, size_);
  #endif
  head = (head_t *)calloc(l,sizeof(head_t));  // initialized to 0
  size /= sizeof(Qfloat);
  size -= l * sizeof(head_t) / sizeof(Qfloat);

  lru_head.next = lru_head.prev = &lru_head;
}

Cache::~Cache()
{
  #ifdef DEBUG
    printf(">>> Destroying Cache class and all its data.\n");
  #endif
  for(head_t *h = lru_head.next; h != &lru_head; h=h->next)
    free(h->data);
  free(head);
}

void Cache::lru_delete(head_t *h)
{
  // delete from current location
  h->prev->next = h->next;
  h->next->prev = h->prev;
}

void Cache::lru_insert(head_t *h)
{
  // insert to last position
  h->next = &lru_head;
  h->prev = lru_head.prev;
  h->prev->next = h;
  h->next->prev = h;
}

int Cache::get_data(const int index, Qfloat **data, int len)
{
  head_t *h = &head[index];
  if(h->len) lru_delete(h);
  int more = len - h->len;

  if(more > 0)
  {
    // free old space
    while(size < more)
    {
      #ifdef DEBUG
      printf(">>> Freeing old space in the cache.\n");
      #endif
      head_t *old = lru_head.next;
      lru_delete(old);
      free(old->data);
      size += old->len;
      old->data = 0;
      old->len = 0;
    }

    // allocate new space
    h->data = (Qfloat *)realloc(h->data,sizeof(Qfloat)*len);
    size -= more;
    swap(h->len,len);
  }

  lru_insert(h);
  *data = h->data;
  return len;
}

void Cache::swap_index(int i, int j)
{
  if(i==j) return;

  if(head[i].len) lru_delete(&head[i]);
  if(head[j].len) lru_delete(&head[j]);
  swap(head[i].data,head[j].data);
  swap(head[i].len,head[j].len);
  if(head[i].len) lru_insert(&head[i]);
  if(head[j].len) lru_insert(&head[j]);

  if(i>j) swap(i,j);
  for(head_t *h = lru_head.next; h!=&lru_head; h=h->next)
  {
    if(h->len > i)
    {
      if(h->len > j)
        swap(h->data[i],h->data[j]);
      else
      {
        // give up
        lru_delete(h);
        free(h->data);
        size += h->len;
        h->data = 0;
        h->len = 0;
      }
    }
  }
}


/**
 * Definition of the Kernel class. Declaration moved to the header file svm.h.
 */
inline void Kernel::swap_index(int i, int j) const  // no so const...
{
  swap(x[i],x[j]);
  if(x_square) swap(x_square[i],x_square[j]);
}


inline double Kernel::kernel_linear(int i, int j) const
{
  return dot(x[i],x[j]);
}
inline double Kernel::kernel_poly(int i, int j) const
{
  return pow(gamma*dot(x[i],x[j])+coef0,degree);
}
inline double  Kernel::kernel_rbf(int i, int j) const
{
  return exp(-gamma*(x_square[i]+x_square[j]-2*dot(x[i],x[j])));
}
inline double Kernel::kernel_sigmoid(int i, int j) const
{
  return tanh(gamma*dot(x[i],x[j])+coef0);
}
inline double Kernel::kernel_localkernel1(int i, int j) const
{
  return localkernel1(x[i], x[j], svmparam);
}
inline double Kernel::kernel_localkernel2(int i, int j) const
{
  return localkernel2(x[i], x[j], svmparam);
}
inline double Kernel::kernel_chiSquared(int i, int j) const
{
  return chiSquared(x[i],x[j], gamma);
}
inline double Kernel::kernel_generalizedGauss(int i, int j) const
{
  return generalizedGauss(x[i],x[j], degree, coef0, gamma);
}
inline double Kernel::kernel_intersection(int i, int j) const
{
  return intersection(x[i],x[j]);
}


/**
 * Constructor.
 */
Kernel::Kernel(int l, svm_node * const * x_, const svm_parameter& param)
:kernel_type(param.kernel_type), degree(param.degree),
 gamma(param.gamma), coef0(param.coef0), svmparam(param)

{
  switch(kernel_type)
  {
    case LINEAR:
      kernel_function = &Kernel::kernel_linear;
      break;
    case POLY:
      kernel_function = &Kernel::kernel_poly;
      break;
    case RBF:
    kernel_function = &Kernel::kernel_rbf;
      break;
    case SIGMOID:
      kernel_function = &Kernel::kernel_sigmoid;
      break;
    case LOCALKERNEL1:
      kernel_function = &Kernel::kernel_localkernel1;
      break;
    case LOCALKERNEL2:
      kernel_function = &Kernel::kernel_localkernel2;
      break;
    case CHISQUARED:
      kernel_function = &Kernel::kernel_chiSquared;
      break;
    case GENERALIZEDGAUSS:
      kernel_function = &Kernel::kernel_generalizedGauss;
      break;
    case INTERSECTION:
      kernel_function = &Kernel::kernel_intersection;
      break;
  }

  clone(x,x_,l);

  if(kernel_type == RBF)
  {
    x_square = new double[l];
    for(int i=0;i<l;i++)
      x_square[i] = dot(x[i],x[i]);
  }
  else
    x_square = 0;
}


/** Destructor. */
Kernel::~Kernel()
{
  delete[] x;
  delete[] x_square;
}

/** Calculates a dot product. */
double Kernel::dot(const svm_node *px, const svm_node *py)
{
  double sum = 0;
  while(px->index != -1 && py->index != -1)
  {
    if(px->index == py->index)
    {
      sum += px->value * py->value;
      ++px;
      ++py;
    }
    else
    {
      if(px->index > py->index)
        ++py;
      else
        ++px;
    }
  }
  return sum;
}


/**
 * Performs single single kernel evaluation given two samples.
 * Used during prediction.
*/
double Kernel::k_function(const svm_node *x, const svm_node *y,
        const svm_parameter& param)
{
  switch(param.kernel_type)
  {
    case LINEAR:
      return dot(x,y);
    case POLY:
      return pow(param.gamma*dot(x,y)+param.coef0,param.degree);
    case RBF:
    {
      double sum = 0;
      while(x->index != -1 && y->index !=-1)
      {
        if(x->index == y->index)
        {
          double d = x->value - y->value;
          sum += d*d;
          ++x;
          ++y;
        }
        else
        {
          if(x->index > y->index)
          {
            sum += y->value * y->value;
            ++y;
          }
          else
          {
            sum += x->value * x->value;
            ++x;
          }
        }
      }

      while(x->index != -1)
      {
        sum += x->value * x->value;
        ++x;
      }

      while(y->index != -1)
      {
        sum += y->value * y->value;
        ++y;
      }

      return exp(-param.gamma*sum);
    }
    case SIGMOID:
      return tanh(param.gamma*dot(x,y)+param.coef0);
    case LOCALKERNEL1:
      return localkernel1(x, y, param);
    case LOCALKERNEL2:
      return localkernel2(x, y, param);
    case CHISQUARED:
      return chiSquared(x, y, param.gamma);
    case GENERALIZEDGAUSS:
      return generalizedGauss(x, y, param.degree, param.coef0, param.gamma);
    case INTERSECTION:
      return intersection(x,y);
    default:
      return 0; /* Unreachable */
  }
}







/**
 * Definition of the Solver class
 */
inline double Solver::get_C(int i)
{
  return (y[i] > 0)? Cp : Cn;
}


inline void Solver::update_alpha_status(int i)
{
  if(alpha[i] >= get_C(i))
    alpha_status[i] = UPPER_BOUND;
  else if(alpha[i] <= 0)
    alpha_status[i] = LOWER_BOUND;
  else alpha_status[i] = FREE;
}



void Solver::swap_index(int i, int j)
{
  #ifdef DEBUG
  //printf(">>> Solver::swap_index(i=%d, j=%d).\n", i,j);
  #endif
  Q->swap_index(i,j);
  swap(y[i],y[j]);
  swap(G[i],G[j]);
  swap(alpha_status[i],alpha_status[j]);
  swap(alpha[i],alpha[j]);
  swap(b[i],b[j]);
  swap(active_set[i],active_set[j]);
  swap(G_bar[i],G_bar[j]);
}

void Solver::reconstruct_gradient()
{
  // reconstruct inactive elements of G from G_bar and free variables

  if(active_size == l) return;

  int i;
  for(i=active_size;i<l;i++)
    G[i] = G_bar[i] + b[i];

  for(i=0;i<active_size;i++)
    if(is_free(i))
    {
      const Qfloat *Q_i = Q->get_Q(i,l);
      double alpha_i = alpha[i];
      for(int j=active_size;j<l;j++)
        G[j] += alpha_i * Q_i[j];
    }
}


/**
 * l - [in] Number of training samples in the whole problem (prob->l)
 * Q - []
 * b_ - [in] For C-SVC: b_[i]=-1
 * y_ - [in] For C-SVC: indicates the class label (-1 or 1)
 * alpha_ - [in] For C-SVC: alpha_[i]=0
 * Cp - [in] C for samples with y=+1
 * Cn - [in] C for samples with y=-1
 * eps - [in] Stopping criteria (epsilon)
 * si - [out] Info about the computed solution
 * shrinking [in] Use shrinking heuristics?
 */
void Solver::Solve(int l, const Kernel& Q, const double *b_, const schar *y_,
       double *alpha_, double Cp, double Cn, double eps,
       SolutionInfo* si, const svm_parameter* param)
{
  this->l = l;
  this->Q = &Q;
  clone(b, b_,l);  // Creates a deep copy of the table b_
  clone(y, y_,l); // Creates a deep copy of the table y_
  clone(alpha,alpha_,l);
  this->Cp = Cp;
  this->Cn = Cn;
  this->eps = eps;
  unshrinked = false;

  // initialize alpha_status
  {
    alpha_status = new char[l];
    for(int i=0;i<l;i++)
      update_alpha_status(i);
  }

  // initialize active set (for shrinking)
  {
    active_set = new int[l];
    for(int i=0;i<l;i++)
      active_set[i] = i;
    active_size = l;
  }

  // initialize gradient
  {
    G = new double[l];
    G_bar = new double[l];
    int i;
    for(i=0;i<l;i++)
    {
      G[i] = b[i];
      G_bar[i] = 0;
    }
    for(i=0;i<l;i++)
      if(!is_lower_bound(i))
      {
        Qfloat *Q_i = Q.get_Q(i,l);
        double alpha_i = alpha[i];
        int j;
        for(j=0;j<l;j++)
          G[j] += alpha_i*Q_i[j];
        if(is_upper_bound(i))
          for(j=0;j<l;j++)
            G_bar[j] += get_C(i) * Q_i[j];
      }
  }

  // optimization step

  int iter = 0;
  int counter = min(l,1000)+1;

  while(1)
  {
    // show progress and do shrinking

    if(--counter == 0)
    {
      counter = min(l,1000);
      if(param->shrinking) do_shrinking();
      info("."); info_flush();
    }

    int i,j;
    if(select_working_set(i,j)!=0)
    {
      // reconstruct the whole gradient
      reconstruct_gradient();
      // reset active set size and check
      active_size = l;
      info("*"); info_flush();
      if(select_working_set(i,j)!=0)
        break;
      else
        counter = 1;  // do shrinking next iteration
    }

    ++iter;

    // update alpha[i] and alpha[j], handle bounds carefully

    const Qfloat *Q_i = Q.get_Q(i,active_size);
    const Qfloat *Q_j = Q.get_Q(j,active_size);

    double C_i = get_C(i);
    double C_j = get_C(j);

    double old_alpha_i = alpha[i];
    double old_alpha_j = alpha[j];

    if(y[i]!=y[j])
    {
      double delta = (-G[i]-G[j])/max(Q_i[i]+Q_j[j]+2*Q_i[j],(Qfloat)0);
      double diff = alpha[i] - alpha[j];
      alpha[i] += delta;
      alpha[j] += delta;

      if(diff > 0)
      {
        if(alpha[j] < 0)
        {
          alpha[j] = 0;
          alpha[i] = diff;
        }
      }
      else
      {
        if(alpha[i] < 0)
        {
          alpha[i] = 0;
          alpha[j] = -diff;
        }
      }
      if(diff > C_i - C_j)
      {
        if(alpha[i] > C_i)
        {
          alpha[i] = C_i;
          alpha[j] = C_i - diff;
        }
      }
      else
      {
        if(alpha[j] > C_j)
        {
          alpha[j] = C_j;
          alpha[i] = C_j + diff;
        }
      }
    }
    else
    {
      double delta = (G[i]-G[j])/max(Q_i[i]+Q_j[j]-2*Q_i[j],(Qfloat)0);
      double sum = alpha[i] + alpha[j];
      alpha[i] -= delta;
      alpha[j] += delta;
      if(sum > C_i)
      {
        if(alpha[i] > C_i)
        {
          alpha[i] = C_i;
          alpha[j] = sum - C_i;
        }
      }
      else
      {
        if(alpha[j] < 0)
        {
          alpha[j] = 0;
          alpha[i] = sum;
        }
      }
      if(sum > C_j)
      {
        if(alpha[j] > C_j)
        {
          alpha[j] = C_j;
          alpha[i] = sum - C_j;
        }
      }
      else
      {
        if(alpha[i] < 0)
        {
          alpha[i] = 0;
          alpha[j] = sum;
        }
      }
    }

    // update G

    double delta_alpha_i = alpha[i] - old_alpha_i;
    double delta_alpha_j = alpha[j] - old_alpha_j;

    for(int k=0;k<active_size;k++)
    {
      G[k] += Q_i[k]*delta_alpha_i + Q_j[k]*delta_alpha_j;
    }

    // update alpha_status and G_bar

    {
      bool ui = is_upper_bound(i);
      bool uj = is_upper_bound(j);
      update_alpha_status(i);
      update_alpha_status(j);
      int k;
      if(ui != is_upper_bound(i))
      {
        Q_i = Q.get_Q(i,l);
        if(ui)
          for(k=0;k<l;k++)
            G_bar[k] -= C_i * Q_i[k];
        else
          for(k=0;k<l;k++)
            G_bar[k] += C_i * Q_i[k];
      }

      if(uj != is_upper_bound(j))
      {
        Q_j = Q.get_Q(j,l);
        if(uj)
          for(k=0;k<l;k++)
            G_bar[k] -= C_j * Q_j[k];
        else
          for(k=0;k<l;k++)
            G_bar[k] += C_j * Q_j[k];
      }
    }
  }

  // calculate rho

  si->rho = calculate_rho();

  // Calculate objective value
  {
    double v = 0;
    int i;
    for(i=0;i<l;i++)
      v += alpha[i] * (G[i] + b[i]);

    si->obj = v/2;
  }

  // Calculate:
  // - ||w||
  // - avg distances between the positive and negative samples and the hyperplane
  // - avg values of the decision function for the negative and positive samples
  // This is used for confidence estimation
  calcAvgDists(Q, alpha, si->rho, y, l, si);

  // Put back the solution
  {
    for(int i=0;i<l;i++)
      alpha_[active_set[i]] = alpha[i];
  }

  // Juggle everything back
  /*{
    for(int i=0;i<l;i++)
      while(active_set[i] != i)
        swap_index(i,active_set[i]);
        // or Q.swap_index(i,active_set[i]);
  }*/

  // Get upper bounds
  si->upper_bound_p = Cp;
  si->upper_bound_n = Cn;

  // Print info
  info("\nOptimization finished, #iter = %d\n",iter);

  // Clean-up
  delete[] b;
  delete[] y;
  delete[] alpha;
  delete[] alpha_status;
  delete[] active_set;
  delete[] G;
  delete[] G_bar;
}

// return 1 if already optimal, return 0 otherwise
int Solver::select_working_set(int &out_i, int &out_j)
{
  // return i,j which maximize -grad(f)^T d , under constraint
  // if alpha_i == C, d != +1
  // if alpha_i == 0, d != -1

  double Gmax1 = -INF;    // max { -grad(f)_i * d | y_i*d = +1 }
  int Gmax1_idx = -1;

  double Gmax2 = -INF;    // max { -grad(f)_i * d | y_i*d = -1 }
  int Gmax2_idx = -1;

  for(int i=0;i<active_size;i++)
  {
    if(y[i]==+1)  // y = +1
    {
      if(!is_upper_bound(i))  // d = +1
      {
        if(-G[i] > Gmax1)
        {
          Gmax1 = -G[i];
          Gmax1_idx = i;
        }
      }
      if(!is_lower_bound(i))  // d = -1
      {
        if(G[i] > Gmax2)
        {
          Gmax2 = G[i];
          Gmax2_idx = i;
        }
      }
    }
    else    // y = -1
    {
      if(!is_upper_bound(i))  // d = +1
      {
        if(-G[i] > Gmax2)
        {
          Gmax2 = -G[i];
          Gmax2_idx = i;
        }
      }
      if(!is_lower_bound(i))  // d = -1
      {
        if(G[i] > Gmax1)
        {
          Gmax1 = G[i];
          Gmax1_idx = i;
        }
      }
    }
  }

  if(Gmax1+Gmax2 < eps)
    return 1;

  out_i = Gmax1_idx;
  out_j = Gmax2_idx;
  return 0;
}

void Solver::do_shrinking()
{
  int i,j,k;
  if(select_working_set(i,j)!=0) return;
  double Gm1 = -y[j]*G[j];
  double Gm2 = y[i]*G[i];

  // shrink

  for(k=0;k<active_size;k++)
  {
    if(is_lower_bound(k))
    {
      if(y[k]==+1)
      {
        if(-G[k] >= Gm1) continue;
      }
      else  if(-G[k] >= Gm2) continue;
    }
    else if(is_upper_bound(k))
    {
      if(y[k]==+1)
      {
        if(G[k] >= Gm2) continue;
      }
      else  if(G[k] >= Gm1) continue;
    }
    else continue;

    --active_size;
    swap_index(k,active_size);
    --k;  // look at the newcomer
  }

  // unshrink, check all variables again before final iterations

  if(unshrinked || -(Gm1 + Gm2) > eps*10) return;

  unshrinked = true;
  reconstruct_gradient();

  for(k=l-1;k>=active_size;k--)
  {
    if(is_lower_bound(k))
    {
      if(y[k]==+1)
      {
        if(-G[k] < Gm1) continue;
      }
      else  if(-G[k] < Gm2) continue;
    }
    else if(is_upper_bound(k))
    {
      if(y[k]==+1)
      {
        if(G[k] < Gm2) continue;
      }
      else  if(G[k] < Gm1) continue;
    }
    else continue;

    swap_index(k,active_size);
    active_size++;
    ++k;  // look at the newcomer
  }
}

double Solver::calculate_rho()
{
  double r;
  int nr_free = 0;
  double ub = INF, lb = -INF, sum_free = 0;
  for(int i=0;i<active_size;i++)
  {
    double yG = y[i]*G[i];

    if(is_lower_bound(i))
    {
      if(y[i] > 0)
        ub = min(ub,yG);
      else
        lb = max(lb,yG);
    }
    else if(is_upper_bound(i))
    {
      if(y[i] < 0)
        ub = min(ub,yG);
      else
        lb = max(lb,yG);
    }
    else
    {
      ++nr_free;
      sum_free += yG;
    }
  }

  if(nr_free>0)
    r = sum_free/nr_free;
  else
    r = (ub+lb)/2;

  return r;
}

//
// Solver for nu-svm classification and regression
//
// additional constraint: e^T \alpha = constant
//
class Solver_NU : public Solver
{
public:
  Solver_NU() {}
  void Solve(int l, const Kernel& Q, const double *b, const schar *y,
       double *alpha, double Cp, double Cn, double eps,
       SolutionInfo* si, const svm_parameter* param)
  {
    this->si = si;
    Solver::Solve(l,Q,b,y,alpha,Cp,Cn,eps,si,param);
  }
private:
  SolutionInfo *si;
  int select_working_set(int &i, int &j);
  double calculate_rho();
  void do_shrinking();
};

int Solver_NU::select_working_set(int &out_i, int &out_j)
{
  // return i,j which maximize -grad(f)^T d , under constraint
  // if alpha_i == C, d != +1
  // if alpha_i == 0, d != -1

  double Gmax1 = -INF;  // max { -grad(f)_i * d | y_i = +1, d = +1 }
  int Gmax1_idx = -1;

  double Gmax2 = -INF;  // max { -grad(f)_i * d | y_i = +1, d = -1 }
  int Gmax2_idx = -1;

  double Gmax3 = -INF;  // max { -grad(f)_i * d | y_i = -1, d = +1 }
  int Gmax3_idx = -1;

  double Gmax4 = -INF;  // max { -grad(f)_i * d | y_i = -1, d = -1 }
  int Gmax4_idx = -1;

  for(int i=0;i<active_size;i++)
  {
    if(y[i]==+1)  // y == +1
    {
      if(!is_upper_bound(i))  // d = +1
      {
        if(-G[i] > Gmax1)
        {
          Gmax1 = -G[i];
          Gmax1_idx = i;
        }
      }
      if(!is_lower_bound(i))  // d = -1
      {
        if(G[i] > Gmax2)
        {
          Gmax2 = G[i];
          Gmax2_idx = i;
        }
      }
    }
    else    // y == -1
    {
      if(!is_upper_bound(i))  // d = +1
      {
        if(-G[i] > Gmax3)
        {
          Gmax3 = -G[i];
          Gmax3_idx = i;
        }
      }
      if(!is_lower_bound(i))  // d = -1
      {
        if(G[i] > Gmax4)
        {
          Gmax4 = G[i];
          Gmax4_idx = i;
        }
      }
    }
  }

  if(max(Gmax1+Gmax2,Gmax3+Gmax4) < eps)
    return 1;

  if(Gmax1+Gmax2 > Gmax3+Gmax4)
  {
    out_i = Gmax1_idx;
    out_j = Gmax2_idx;
  }
  else
  {
    out_i = Gmax3_idx;
    out_j = Gmax4_idx;
  }
  return 0;
}

void Solver_NU::do_shrinking()
{
  double Gmax1 = -INF;  // max { -grad(f)_i * d | y_i = +1, d = +1 }
  double Gmax2 = -INF;  // max { -grad(f)_i * d | y_i = +1, d = -1 }
  double Gmax3 = -INF;  // max { -grad(f)_i * d | y_i = -1, d = +1 }
  double Gmax4 = -INF;  // max { -grad(f)_i * d | y_i = -1, d = -1 }

  int k;
  for(k=0;k<active_size;k++)
  {
    if(!is_upper_bound(k))
    {
      if(y[k]==+1)
      {
        if(-G[k] > Gmax1) Gmax1 = -G[k];
      }
      else  if(-G[k] > Gmax3) Gmax3 = -G[k];
    }
    if(!is_lower_bound(k))
    {
      if(y[k]==+1)
      {
        if(G[k] > Gmax2) Gmax2 = G[k];
      }
      else  if(G[k] > Gmax4) Gmax4 = G[k];
    }
  }

  double Gm1 = -Gmax2;
  double Gm2 = -Gmax1;
  double Gm3 = -Gmax4;
  double Gm4 = -Gmax3;

  for(k=0;k<active_size;k++)
  {
    if(is_lower_bound(k))
    {
      if(y[k]==+1)
      {
        if(-G[k] >= Gm1) continue;
      }
      else  if(-G[k] >= Gm3) continue;
    }
    else if(is_upper_bound(k))
    {
      if(y[k]==+1)
      {
        if(G[k] >= Gm2) continue;
      }
      else  if(G[k] >= Gm4) continue;
    }
    else continue;

    --active_size;
    swap_index(k,active_size);
    --k;  // look at the newcomer
  }

  // unshrink, check all variables again before final iterations

  if(unshrinked || max(-(Gm1+Gm2),-(Gm3+Gm4)) > eps*10) return;

  unshrinked = true;
  reconstruct_gradient();

  for(k=l-1;k>=active_size;k--)
  {
    if(is_lower_bound(k))
    {
      if(y[k]==+1)
      {
        if(-G[k] < Gm1) continue;
      }
      else  if(-G[k] < Gm3) continue;
    }
    else if(is_upper_bound(k))
    {
      if(y[k]==+1)
      {
        if(G[k] < Gm2) continue;
      }
      else  if(G[k] < Gm4) continue;
    }
    else continue;

    swap_index(k,active_size);
    active_size++;
    ++k;  // look at the newcomer
  }
}

double Solver_NU::calculate_rho()
{
  int nr_free1 = 0,nr_free2 = 0;
  double ub1 = INF, ub2 = INF;
  double lb1 = -INF, lb2 = -INF;
  double sum_free1 = 0, sum_free2 = 0;

  for(int i=0;i<active_size;i++)
  {
    if(y[i]==+1)
    {
      if(is_lower_bound(i))
        ub1 = min(ub1,G[i]);
      else if(is_upper_bound(i))
        lb1 = max(lb1,G[i]);
      else
      {
        ++nr_free1;
        sum_free1 += G[i];
      }
    }
    else
    {
      if(is_lower_bound(i))
        ub2 = min(ub2,G[i]);
      else if(is_upper_bound(i))
        lb2 = max(lb2,G[i]);
      else
      {
        ++nr_free2;
        sum_free2 += G[i];
      }
    }
  }

  double r1,r2;
  if(nr_free1 > 0)
    r1 = sum_free1/nr_free1;
  else
    r1 = (ub1+lb1)/2;

  if(nr_free2 > 0)
    r2 = sum_free2/nr_free2;
  else
    r2 = (ub2+lb2)/2;

  si->r = (r1+r2)/2;
  return (r1-r2)/2;
}

/**
 * Q matrix for SVC. Stores values in its internal cache.
 * Q(i,j)=yi*yj*K(xi,xj)
 * Used for solving subproblems consisting of two classes.
 */
class SVC_Q: public Kernel
{

public:

  /**
   * Constructor. Creates a new cache.
   */
  SVC_Q(const svm_problem& prob, const svm_parameter& param, const schar *y_)
  :Kernel(prob.l, prob.x, param)
  {
    #ifdef DEBUG
      printf(">>> Constructing SVC_Q class for problem of prob->l=%d.\n", prob.l);
    #endif

    clone(y,y_,prob.l);
    cache = new Cache(prob.l,(int)(param.cache_size*(1<<20)));
  }

  /** Returns a column of the Q matrix. Q(i,j)=yi*yj*K(xi,xj) */
  Qfloat *get_Q(int i, int len) const
  {
    Qfloat *data;
    int start;
    if((start = cache->get_data(i,&data,len)) < len)
    {
      #ifdef DEBUG
      //printf(">>> SVC_Q.get_Q(i=%d, len=%d): computing K(%d-%d,%d).\n", i, len, start, len-1, i);
      #endif
      for(int j=start;j<len;j++)
        data[j] = (Qfloat)(y[i]*y[j]*(this->*kernel_function)(i,j));
    }
    return data;
  }

  void swap_index(int i, int j) const
  {
    cache->swap_index(i,j);
    Kernel::swap_index(i,j);
    swap(y[i],y[j]);
  }

  ~SVC_Q()
  {
    #ifdef DEBUG
      printf(">>> Destroying SVC_Q class.\n");
    #endif
    delete[] y;
    delete cache;
  }


private:

  /** Class label. */
  schar *y;

  /** Cache. */
  Cache *cache;
};


/**
 * Q matrix for one class SVM.
 */
class ONE_CLASS_Q: public Kernel
{
public:
  ONE_CLASS_Q(const svm_problem& prob, const svm_parameter& param)
  :Kernel(prob.l, prob.x, param)
  {
    cache = new Cache(prob.l,(int)(param.cache_size*(1<<20)));
  }

  Qfloat *get_Q(int i, int len) const
  {
    Qfloat *data;
    int start;
    if((start = cache->get_data(i,&data,len)) < len)
    {
      for(int j=start;j<len;j++)
        data[j] = (Qfloat)(this->*kernel_function)(i,j);
    }
    return data;
  }

  void swap_index(int i, int j) const
  {
    cache->swap_index(i,j);
    Kernel::swap_index(i,j);
  }

  ~ONE_CLASS_Q()
  {
    delete cache;
  }
private:
  Cache *cache;
};



/**
 * Q matrix for SVR.
 */
class SVR_Q: public Kernel
{
public:
  SVR_Q(const svm_problem& prob, const svm_parameter& param)
  :Kernel(prob.l, prob.x, param)
  {
    l = prob.l;
    cache = new Cache(l,(int)(param.cache_size*(1<<20)));
    sign = new schar[2*l];
    index = new int[2*l];
    for(int k=0;k<l;k++)
    {
      sign[k] = 1;
      sign[k+l] = -1;
      index[k] = k;
      index[k+l] = k;
    }
    buffer[0] = new Qfloat[2*l];
    buffer[1] = new Qfloat[2*l];
    next_buffer = 0;
  }

  void swap_index(int i, int j) const
  {
    swap(sign[i],sign[j]);
    swap(index[i],index[j]);
  }

  Qfloat *get_Q(int i, int len) const
  {
    Qfloat *data;
    int real_i = index[i];
    if(cache->get_data(real_i,&data,l) < l)
    {
      for(int j=0;j<l;j++)
        data[j] = (Qfloat)(this->*kernel_function)(real_i,j);
    }

    // reorder and copy
    Qfloat *buf = buffer[next_buffer];
    next_buffer = 1 - next_buffer;
    schar si = sign[i];
    for(int j=0;j<len;j++)
      buf[j] = si * sign[j] * data[index[j]];
    return buf;
  }

  ~SVR_Q()
  {
    delete cache;
    delete[] sign;
    delete[] index;
    delete[] buffer[0];
    delete[] buffer[1];
  }
private:
  int l;
  Cache *cache;
  schar *sign;
  int *index;
  mutable int next_buffer;
  Qfloat* buffer[2];
};



/**
 * Solves classification subproblem for two classes only (y=-1 or y=1).
 * prob - [in] Subproblem
 * param - [in] SVM parameters
 * alpha - [out] Computed alpha coeffitients for the subproblem
 * si - [out] Some info about the found solution
 * Cp - [in] C for samples with y=+1
 * Cn - [in] C for samples with y=-1
 */
static void solve_c_svc(
  const svm_problem *prob, const svm_parameter* param,
  double *alpha, Solver::SolutionInfo* si, double Cp, double Cn)
{
#ifdef DEBUG
  printf(">>> Beginning solve_c_svc() for problem of prob->l=%d\n", prob->l);
#endif

  int l = prob->l;
  double *minus_ones = new double[l];
  schar *y = new schar[l];

  int i;

  for(i=0;i<l;i++)
  {
    alpha[i] = 0;
    minus_ones[i] = -1;
    if(prob->y[i] > 0) y[i] = +1; else y[i]=-1;
  }

  // Solve the optimization problem
  Solver s;
  if (param->svm_type == ONE_AGAINST_ALL)
    s.Solve(l, SVC_Q(*prob,*param,y), minus_ones, y,
    alpha, param->C, param->C, param->eps, si, param);
  else
    s.Solve(l, SVC_Q(*prob,*param,y), minus_ones, y,
    alpha, Cp, Cn, param->eps, si, param);


  // Compute nu
  double sum_alpha=0;
  for(i=0;i<l;i++)
    sum_alpha += alpha[i];
  info("nu = %f\n", sum_alpha/(param->C*prob->l));

  // Multiply alpha[i] by y[i]
  for(i=0;i<l;i++)
    alpha[i] *= y[i];

  // Freeing memory
  delete[] minus_ones;
  delete[] y;

  // Debug info
  #ifdef DEBUG
  printf(">>> Ending solve_c_svc()\n");
  #endif
}

static void solve_nu_svc(
  const svm_problem *prob, const svm_parameter *param,
  double *alpha, Solver::SolutionInfo* si)
{
  int i;
  int l = prob->l;
  double nu = param->nu;

  schar *y = new schar[l];

  for(i=0;i<l;i++)
    if(prob->y[i]>0)
      y[i] = +1;
    else
      y[i] = -1;

  double sum_pos = nu*l/2;
  double sum_neg = nu*l/2;

  for(i=0;i<l;i++)
    if(y[i] == +1)
    {
      alpha[i] = min(1.0,sum_pos);
      sum_pos -= alpha[i];
    }
    else
    {
      alpha[i] = min(1.0,sum_neg);
      sum_neg -= alpha[i];
    }

  double *zeros = new double[l];

  for(i=0;i<l;i++)
    zeros[i] = 0;

  Solver_NU s;
  s.Solve(l, SVC_Q(*prob,*param,y), zeros, y,
    alpha, 1.0, 1.0, param->eps, si,  param);
  double r = si->r;

  info("C = %f\n",1/r);

  for(i=0;i<l;i++)
    alpha[i] *= y[i]/r;

  si->rho /= r;
  si->obj /= (r*r);
  si->upper_bound_p = 1/r;
  si->upper_bound_n = 1/r;

  delete[] y;
  delete[] zeros;
}

static void solve_one_class(
  const svm_problem *prob, const svm_parameter *param,
  double *alpha, Solver::SolutionInfo* si)
{
  int l = prob->l;
  double *zeros = new double[l];
  schar *ones = new schar[l];
  int i;

  int n = (int)(param->nu*prob->l); // # of alpha's at upper bound

  for(i=0;i<n;i++)
    alpha[i] = 1;
  alpha[n] = param->nu * prob->l - n;
  for(i=n+1;i<l;i++)
    alpha[i] = 0;

  for(i=0;i<l;i++)
  {
    zeros[i] = 0;
    ones[i] = 1;
  }

  Solver s;
  s.Solve(l, ONE_CLASS_Q(*prob,*param), zeros, ones,
    alpha, 1.0, 1.0, param->eps, si, param);

  delete[] zeros;
  delete[] ones;
}

static void solve_epsilon_svr(
  const svm_problem *prob, const svm_parameter *param,
  double *alpha, Solver::SolutionInfo* si)
{
  int l = prob->l;
  double *alpha2 = new double[2*l];
  double *linear_term = new double[2*l];
  schar *y = new schar[2*l];
  int i;

  for(i=0;i<l;i++)
  {
    alpha2[i] = 0;
    linear_term[i] = param->p - prob->y[i];
    y[i] = 1;

    alpha2[i+l] = 0;
    linear_term[i+l] = param->p + prob->y[i];
    y[i+l] = -1;
  }

  Solver s;
  s.Solve(2*l, SVR_Q(*prob,*param), linear_term, y,
    alpha2, param->C, param->C, param->eps, si, param);

  double sum_alpha = 0;
  for(i=0;i<l;i++)
  {
    alpha[i] = alpha2[i] - alpha2[i+l];
    sum_alpha += fabs(alpha[i]);
  }
  info("nu = %f\n",sum_alpha/(param->C*l));

  delete[] alpha2;
  delete[] linear_term;
  delete[] y;
}

static void solve_nu_svr(
  const svm_problem *prob, const svm_parameter *param,
  double *alpha, Solver::SolutionInfo* si)
{
  int l = prob->l;
  double C = param->C;
  double *alpha2 = new double[2*l];
  double *linear_term = new double[2*l];
  schar *y = new schar[2*l];
  int i;

  double sum = C * param->nu * l / 2;
  for(i=0;i<l;i++)
  {
    alpha2[i] = alpha2[i+l] = min(sum,C);
    sum -= alpha2[i];

    linear_term[i] = - prob->y[i];
    y[i] = 1;

    linear_term[i+l] = prob->y[i];
    y[i+l] = -1;
  }

  Solver_NU s;
  s.Solve(2*l, SVR_Q(*prob,*param), linear_term, y,
    alpha2, C, C, param->eps, si, param);

  info("epsilon = %f\n",-si->r);

  for(i=0;i<l;i++)
    alpha[i] = alpha2[i] - alpha2[i+l];

  delete[] alpha2;
  delete[] linear_term;
  delete[] y;
}



decision_function svm_train_one(const svm_problem *prob, const svm_parameter *param,
                                double Cp, double Cn, Solver::SolutionInfo *si)
{
  #ifdef DEBUG
  printf(">>> Beginning svm_train_one() for problem prob->l=%d\n", prob->l);
  #endif

  // Solve optimization problem
  double *alpha = Malloc(double,prob->l);
  switch(param->svm_type)
  {
    case C_SVC:
      solve_c_svc(prob,param,alpha,si,Cp,Cn);
      break;
    case NU_SVC:
      solve_nu_svc(prob,param,alpha,si);
      break;
    case ONE_CLASS:
      solve_one_class(prob,param,alpha,si);
      break;
    case EPSILON_SVR:
      solve_epsilon_svr(prob,param,alpha,si);
      break;
    case NU_SVR:
      solve_nu_svr(prob,param,alpha,si);
      break;
    case ONE_AGAINST_ALL:
      solve_c_svc(prob,param,alpha,si,Cp,Cn);
      break;
  }

  // Print some info
  info("obj = %f, rho = %f\n",si->obj,si->rho);
  info("||w||=%f, avgFP=%f, avgFN=%f, avgDistP=%f, avgDistN=%f \n", si->omegaNorm, si->avgFP, si->avgFN, si->avgDistP, si->avgDistN);

  // Create decision function
  decision_function f;
  f.alpha = alpha;
  f.rho = si->rho;

  // Reduce SVs lin. dep. in feature spece
  if (param->RLDSV) reduceLinDepSV(&f, prob, param);


  // output SVs

  int nSV = 0;
  int nBSV = 0;
  for(int i=0;i<prob->l;i++)
  {
    if(fabs(alpha[i]) > 0)
    {
      ++nSV;
      if(prob->y[i] > 0)
      {
        if(fabs(alpha[i]) >= si->upper_bound_p)
          ++nBSV;
      }
      else
      {
        if(fabs(alpha[i]) >= si->upper_bound_n)
          ++nBSV;
      }
    }
  }

  info("nSV = %d, nBSV = %d\n",nSV,nBSV);

#ifdef DEBUG
  printf(">>> Ending svm_train_one()\n\n");
#endif
  return f;
}




//
// Interface functions
//
svm_model *svm_train(const svm_problem *prob, const svm_parameter *param)
{
  #ifdef DEBUG
    printf("\n>>> Beginning svm_train()\n");
  #endif

  svm_model *model = Malloc(svm_model,1);
  model->param = *param;
  model->free_sv = 0; // XXX

  if(param->svm_type == ONE_CLASS ||
     param->svm_type == EPSILON_SVR ||
     param->svm_type == NU_SVR)
  {
    // --------------------------
    // Regression or one-class-svm
    // --------------------------
    #ifdef DEBUG
      printf(">>> Running regression or one-class-svm\n");
    #endif
    model->nr_class = 2;
    model->label = NULL;
    model->nSV = NULL;
    model->sv_coef = Malloc(double *,1);
    Solver::SolutionInfo solInfo;
    decision_function f = svm_train_one(prob,param,0,0, &solInfo);
    model->rho = Malloc(double,1);
    model->rho[0] = f.rho;

    int nSV = 0;
    int i;
    for(i=0;i<prob->l;i++)
      if(fabs(f.alpha[i]) > 0) ++nSV;
    model->l = nSV;
    model->SV = Malloc(svm_node *,nSV);
    model->sv_coef[0] = Malloc(double,nSV);
    int j = 0;
    for(i=0;i<prob->l;i++)
      if(fabs(f.alpha[i]) > 0)
      {
        model->SV[j] = prob->x[i];
        model->sv_coef[0][j] = f.alpha[i];
        ++j;
      }

    free(f.alpha);
  }
  else
  {
    // --------------------------
    // C-SVC
    // --------------------------

    // --------------------------
    // Find out the number of classes
    // --------------------------
    #ifdef DEBUG
      printf(">>> Running C-SVC\n");
    #endif
    int l = prob->l;
    int max_nr_class = 16;
    int nr_class = 0;
    int *label = Malloc(int,max_nr_class);
    int *count = Malloc(int,max_nr_class);
    int *index = Malloc(int,l);

    int i;
    for(i=0;i<l;i++)
    {
      int this_label = (int)prob->y[i];
      int j;
      for(j=0;j<nr_class;j++)
        if(this_label == label[j])
        {
          ++count[j];
          break;
        }
      index[i] = j;
      if(j == nr_class)
      {
        if(nr_class == max_nr_class)
        {
          max_nr_class *= 2;
          label = (int *)realloc(label,max_nr_class*sizeof(int));
          count = (int *)realloc(count,max_nr_class*sizeof(int));
        }
        label[nr_class] = this_label;
        count[nr_class] = 1;
        ++nr_class;
      }
    }


    // --------------------------
    // Group training data of the same class
    // --------------------------
    int *start = Malloc(int,nr_class);
    start[0] = 0;
    for(i=1;i<nr_class;i++)
      start[i] = start[i-1]+count[i-1];

    svm_node **x = Malloc(svm_node *,l);

    for(i=0;i<l;i++)
    {
      x[start[index[i]]] = prob->x[i];
      ++start[index[i]];
    }

    start[0] = 0;
    for(i=1;i<nr_class;i++)
      start[i] = start[i-1]+count[i-1];


    // --------------------------
    // Calculate weighted C
    // --------------------------
    double *weighted_C = Malloc(double, nr_class);
    for(i=0;i<nr_class;i++)
      weighted_C[i] = param->C;
    for(i=0;i<param->nr_weight;i++)
    {
      int j;
      for(j=0;j<nr_class;j++)
        if(param->weight_label[i] == label[j])
          break;
      if(j == nr_class)
        fprintf(stderr,"warning: class label %d specified in weight is not found\n", param->weight_label[i]);
      else
        weighted_C[j] *= param->weight[i];
    }


    // --------------------------
    // Train  models
    // --------------------------
    bool *nonzero = Malloc(bool,l);
    for(i=0;i<l;i++)
      nonzero[i] = false;

    if(param->svm_type == ONE_AGAINST_ALL)
    {
      // --------------------------
      // One against all
      // --------------------------
      #ifdef DEBUG
        printf(">>> Using one-against-all multi-class algorithm.\n");
      #endif

      decision_function *f = Malloc(decision_function, nr_class);
      Solver::SolutionInfo *solInfo=Malloc(Solver::SolutionInfo, nr_class);

      int p = 0;
      svm_problem sub_prob;
      sub_prob.l = l;
      sub_prob.x = x;
      sub_prob.y = Malloc(double,l);
      for (i=start[1];i<l;i++)
        sub_prob.y[i] = -1;

      if (param->trainOnlyClass == -1)
      {
        // --------------------------
        // Standard one-against-all
        // --------------------------
        for(i=0;i<nr_class;i++)
        {
          int j;
          for (j=start[i];j<start[i]+count[i];j++)
            sub_prob.y[j] = 1;
          //cerr<<endl<<"Training class "<<i<<" against the others..."<<endl;
          info("\nTraining class %d (positive) against the others...\n", label[i]);
          f[i] = svm_train_one(&sub_prob, param, 0, 0, &(solInfo[i]));
          for (j=start[i];j<start[i]+count[i];j++)
            sub_prob.y[j] = -1;
          for (j=0;j<l;j++)
            if (fabs(f[i].alpha[j]) > 0)
              nonzero[j] = true;
        }

        free(sub_prob.y);
      }
      else if (param->trainOnlyClass == -2)
      {
        // --------------------------
        // Gather all the models which where computed in parallel
        // --------------------------

        printf("CLASSES: %d\n", nr_class);


        char* filename= Malloc(char, 512);
          for(i=0;i<nr_class;i++)
          {
          printf("reading model %d from disk\n", i);
          fflush(stdout);
          sprintf(filename, param->tmpModelFilename, i);
          FILE* tmpFile= fopen(filename, "r");
          if (tmpFile == NULL)
          {
            printf("Couldn't open %s\n", filename);
            exit(0);
          }
          fscanf(tmpFile, "%lf", &f[i].rho);
          f[i].alpha= Malloc(double, l);
          for (int j=0;j<l;j++)
          {
            fscanf(tmpFile, "%lf", &f[i].alpha[j]);
            if (fabs(f[i].alpha[j]) > 0)
              nonzero[j] = true;
          }
          fclose(tmpFile);
        }
      }
      else
      {
        i= param->trainOnlyClass;
        int j;
        for (j=start[i];j<start[i]+count[i];j++)
          sub_prob.y[j] = 1;
        //    cerr<<endl<<"Training class "<<i<<" against the others in parallelized MODE..."<<endl;
        f[0] = svm_train_one(&sub_prob, param, 0, 0, &(solInfo[0]));
        for (j=start[i];j<start[i]+count[i];j++)
          sub_prob.y[j] = -1;
        for (j=0;j<l;j++)
          if (fabs(f[0].alpha[j]) > 0)
            nonzero[j] = true;

        /* write rho and alphas to disk for parallel training; they are collected by the next call with "-o -2" */
        char* filename= Malloc(char, 512);
        sprintf(filename, param->tmpModelFilename, param->trainOnlyClass);
        FILE* tmpFile= fopen(filename, "w");
        fprintf(tmpFile, "%g\n", f[0].rho);
        for (j=0;j<l;j++)
        {
          fprintf(tmpFile, "%.16g ", f[0].alpha[j]);
        }
        fclose(tmpFile);
        free(filename);

        /* set all the model to the one which was computed to let the program end without crash */
        for(i=0;i<nr_class;i++)
          {
          f[i]= f[0];
        }

          free(sub_prob.y);
      }


      // --------------------------
      // Build output
      // --------------------------
      model->nr_class = nr_class;
      model->nr_hyp = nr_class;

      model->label = Malloc(int,nr_class);
      for(i=0;i<nr_class;i++)
        model->label[i] = label[i];

      model->rho = Malloc(double,nr_class);
      for(i=0;i<nr_class;i++)
        model->rho[i] = f[i].rho;

      model->omegaNorm = Malloc(double,nr_class);
      for(i=0;i<nr_class;i++)
        model->omegaNorm[i] = solInfo[i].omegaNorm;

      model->avgDistP = Malloc(double,nr_class);
      for(i=0;i<nr_class;i++)
        model->avgDistP[i] = solInfo[i].avgDistP;

      model->avgDistN = Malloc(double,nr_class);
      for(i=0;i<nr_class;i++)
        model->avgDistN[i] = solInfo[i].avgDistN;

      model->avgFP = Malloc(double,nr_class);
      for(i=0;i<nr_class;i++)
        model->avgFP[i] = solInfo[i].avgFP;

      model->avgFN = Malloc(double,nr_class);
      for(i=0;i<nr_class;i++)
        model->avgFN[i] = solInfo[i].avgFN;


      int total_sv = 0;
      model->nSV = Malloc(int,nr_class);
      for(i=0;i<nr_class;i++)
      {
        int nSV = 0;
        for(int j=0;j<count[i];j++)
          if(nonzero[start[i]+j])
          {
            ++nSV;
            ++total_sv;
          }
        model->nSV[i] = nSV;
      }

      info("Total nSV = %d\n",total_sv);

      model->l = total_sv;
      model->SV = Malloc(svm_node *,total_sv);
      p = 0;
      for(i=0;i<l;i++)
        if(nonzero[i]) model->SV[p++] = x[i];

      model->sv_coef = Malloc(double *,nr_class);
      for(i=0;i<nr_class;i++)
        model->sv_coef[i] = Malloc(double,total_sv);

      p = 0;
      for (i=0;i<l;i++)
        if (nonzero[i])
        {
          for (int j=0;j<nr_class;j++)
            model->sv_coef[j][p] = f[j].alpha[i];
          p++;
        }


      // --------------------------
      // Clean up
      // --------------------------
      free(label);
      free(count);
      free(index);
      free(start);
      free(x);
      free(nonzero);
      for(i=0;i<nr_class;i++)
        free(f[i].alpha);
      free(f);

    }
    else
    {
      // --------------------------
      // One against one
      // --------------------------
      #ifdef DEBUG
        printf(">>> Using one-against-one multi-class algorithm.\n");
      #endif

      decision_function *f = Malloc(decision_function, nr_class*(nr_class-1)/2);
      Solver::SolutionInfo *solInfo=Malloc(Solver::SolutionInfo, nr_class*(nr_class-1)/2);

      int p = 0;

      if (param->trainOnlyClass == -1)
      {
        // --------------------------
        // Standard one-against-one
        // --------------------------
        for(i=0;i<nr_class;i++)
          for(int j=i+1;j<nr_class;j++)
          {
            svm_problem sub_prob;
            int si = start[i], sj = start[j];
            int ci = count[i], cj = count[j];
            sub_prob.l = ci+cj;
            sub_prob.x = Malloc(svm_node *,sub_prob.l);
            sub_prob.y = Malloc(double,sub_prob.l);
            int k;
            for(k=0;k<ci;k++)
              {
                sub_prob.x[k] = x[si+k];
                sub_prob.y[k] = +1;
              }
            for(k=0;k<cj;k++)
              {
                sub_prob.x[ci+k] = x[sj+k];
                sub_prob.y[ci+k] = -1;
              }

            info("\nTraining class %d (positive) against class %d (negative)...\n", label[i], label[j]);
            f[p] = svm_train_one(&sub_prob,param,weighted_C[i],weighted_C[j],&(solInfo[p]));

            for(k=0;k<ci;k++)
              if(!nonzero[si+k] && fabs(f[p].alpha[k]) > 0)
                nonzero[si+k] = true;
            for(k=0;k<cj;k++)
              if(!nonzero[sj+k] && fabs(f[p].alpha[ci+k]) > 0)
                nonzero[sj+k] = true;
            free(sub_prob.x);
            free(sub_prob.y);
            ++p;
          }
      }
      else if (param->trainOnlyClass == -2)
      {
        // --------------------------
        // Gather all the models which where computed in parallel
        // --------------------------
        printf("CLASSES: %d\n", nr_class);

        char* filename= Malloc(char, 512);

          for(i=0;i<nr_class;i++)
        {
            for(int j=i+1;j<nr_class;j++)
          {
              int ci = count[i], cj = count[j];
              int si = start[i], sj = start[j];
            printf("reading model %d from disk\n", p);
            fflush(stdout);
            sprintf(filename, param->tmpModelFilename, p);
            FILE* tmpFile= fopen(filename, "r");
            if (tmpFile == NULL)
            {
              printf("Couldn't open %s\n", filename);
              exit(0);
            }
            fscanf(tmpFile, "%lf", &f[p].rho);
            f[p].alpha= Malloc(double, l);
            for (int k=0;k<ci+cj;k++)
            {
              fscanf(tmpFile, "%lf", &f[p].alpha[k]);
            }
              for(int k=0;k<ci;k++)
                if(!nonzero[si+k] && fabs(f[p].alpha[k]) > 0)
                nonzero[si+k] = true;
              for(int k=0;k<cj;k++)
                if(!nonzero[sj+k] && fabs(f[p].alpha[ci+k]) > 0)
                  nonzero[sj+k] = true;
            fclose(tmpFile);
            ++p;
          }
        }
      }
      else
      {
        for(i=0;i<nr_class;i++)
          for(int j=i+1;j<nr_class;j++)
        {
          if (p == param->trainOnlyClass)
          {
            svm_problem sub_prob;
            int si = start[i], sj = start[j];
            int ci = count[i], cj = count[j];
            sub_prob.l = ci+cj;
            sub_prob.x = Malloc(svm_node *,sub_prob.l);
            sub_prob.y = Malloc(double,sub_prob.l);
            int k;
            for(k=0;k<ci;k++)
              {
                sub_prob.x[k] = x[si+k];
                sub_prob.y[k] = +1;
                }
            for(k=0;k<cj;k++)
              {
                sub_prob.x[ci+k] = x[sj+k];
                sub_prob.y[ci+k] = -1;
              }

            f[p] = svm_train_one(&sub_prob,param,weighted_C[i],weighted_C[j], &(solInfo[p]));
            for(k=0;k<ci;k++)
              if(!nonzero[si+k] && fabs(f[p].alpha[k]) > 0)
                nonzero[si+k] = true;
            for(k=0;k<cj;k++)
              if(!nonzero[sj+k] && fabs(f[p].alpha[ci+k]) > 0)
                nonzero[sj+k] = true;
            free(sub_prob.x);
            free(sub_prob.y);

              /* write rho and alphas to disk for parallel training; they are collected by the next call with "-o -2" */
            char* filename= Malloc(char, 512);
            sprintf(filename, param->tmpModelFilename, param->trainOnlyClass);
            FILE* tmpFile= fopen(filename, "w");
            fprintf(tmpFile, "%g\n", f[p].rho);
            for (k=0;k<ci+cj;k++)
            {
              fprintf(tmpFile, "%.16g ", f[p].alpha[k]);
            }
            fclose(tmpFile);
            free(filename);

            exit(1); // training of one classifier finished -> quit (parallel mode)
          }
          ++p;
        }

      }


      // -------------------------
      // Build output
      // -------------------------
      model->nr_class = nr_class;
      model->nr_hyp = nr_class*(nr_class-1)/2;

      model->label = Malloc(int,nr_class);
      for(i=0;i<nr_class;i++)
        model->label[i] = label[i];

      model->rho = Malloc(double,nr_class*(nr_class-1)/2);
      for(i=0;i<nr_class*(nr_class-1)/2;i++)
        model->rho[i] = f[i].rho;

      model->omegaNorm = Malloc(double,nr_class*(nr_class-1)/2);
      for(i=0;i<nr_class*(nr_class-1)/2;i++)
        model->omegaNorm[i] = solInfo[i].omegaNorm;

      model->avgDistP = Malloc(double,nr_class*(nr_class-1)/2);
      for(i=0;i<nr_class*(nr_class-1)/2;i++)
        model->avgDistP[i] = solInfo[i].avgDistP;

      model->avgDistN = Malloc(double,nr_class*(nr_class-1)/2);
      for(i=0;i<nr_class*(nr_class-1)/2;i++)
        model->avgDistN[i] = solInfo[i].avgDistN;

      model->avgFP = Malloc(double,nr_class*(nr_class-1)/2);
      for(i=0;i<nr_class*(nr_class-1)/2;i++)
        model->avgFP[i] = solInfo[i].avgFP;

      model->avgFN = Malloc(double,nr_class*(nr_class-1)/2);
      for(i=0;i<nr_class*(nr_class-1)/2;i++)
        model->avgFN[i] = solInfo[i].avgFN;


      int total_sv = 0;
      int *nz_count = Malloc(int,nr_class);
      model->nSV = Malloc(int,nr_class);
      for(i=0;i<nr_class;i++)
      {
        int nSV = 0;
        for(int j=0;j<count[i];j++)
          if(nonzero[start[i]+j])
          {
            ++nSV;
            ++total_sv;
          }
        model->nSV[i] = nSV;
        nz_count[i] = nSV;
      }

      info("Total nSV = %d\n",total_sv);

      model->l = total_sv;
      model->SV = Malloc(svm_node *,total_sv);
      p = 0;
      for(i=0;i<l;i++)
        if(nonzero[i]) model->SV[p++] = x[i];

      int *nz_start = Malloc(int,nr_class);
      nz_start[0] = 0;
      for(i=1;i<nr_class;i++)
        nz_start[i] = nz_start[i-1]+nz_count[i-1];

      model->sv_coef = Malloc(double *,nr_class-1);
      for(i=0;i<nr_class-1;i++)
        model->sv_coef[i] = Malloc(double,total_sv);

      p = 0;
      for(i=0;i<nr_class;i++)
        for(int j=i+1;j<nr_class;j++)
        {
          // classifier (i,j): coefficients with
          // i are in sv_coef[j-1][nz_start[i]...],
          // j are in sv_coef[i][nz_start[j]...]

          int si = start[i];
          int sj = start[j];
          int ci = count[i];
          int cj = count[j];

          int q = nz_start[i];
          int k;
          for(k=0;k<ci;k++)
            if(nonzero[si+k])
              model->sv_coef[j-1][q++] = f[p].alpha[k];
          q = nz_start[j];
          for(k=0;k<cj;k++)
            if(nonzero[sj+k])
              model->sv_coef[i][q++] = f[p].alpha[ci+k];
          ++p;
        }


      // --------------------------
      // Clean up
      // --------------------------
      free(label);
      free(count);
      free(index);
      free(start);
      free(x);
      free(weighted_C);
      free(nonzero);
      for(i=0;i<nr_class*(nr_class-1)/2;i++)
        free(f[i].alpha);
      free(f);
      free(nz_count);
      free(nz_start);
    } // End One against one
  } // End C-SVC


#ifdef DEBUG
  printf(">>> Ending svm_train()\n");
#endif

  return model;
}




/**
 * Table of SVM type names.
 */
const char *svm_type_table[] =
{
  "c_svc","nu_svc","one_class","epsilon_svr","nu_svr","one_against_all", NULL
};


/**
 * Table of kernel type names.
 */
const char *kernel_type_table[]=
{
  "linear","polynomial","rbf","sigmoid","localkernel1" ,"chiSquared","generalizedGauss", "intersection", "localkernel2", NULL
};


/**
 * Saves the model to a file.
 */
int svm_save_model(const char *model_file_name, const svm_model *model)
{
  FILE *fp = fopen(model_file_name,"w");
  if(fp==NULL) return -1;

  const svm_parameter& param = model->param;

  // -------------------------------
  // Save SVM and kernel type
  // -------------------------------
  fprintf(fp,"svm_type %s\n", svm_type_table[param.svm_type]);
  fprintf(fp,"kernel_type %s\n", kernel_type_table[param.kernel_type]);
  fprintf(fp,"trainOnlyClass %d\n", param.trainOnlyClass);

  // -------------------------------
  // Save kernel parameters
  // -------------------------------
  if(param.kernel_type == POLY || param.kernel_type == GENERALIZEDGAUSS)
    fprintf(fp,"degree %g\n", param.degree);

  if(param.kernel_type == POLY || param.kernel_type == RBF || param.kernel_type == SIGMOID ||
     param.kernel_type == LOCALKERNEL1 || param.kernel_type == LOCALKERNEL2 ||
     param.kernel_type == CHISQUARED || param.kernel_type == GENERALIZEDGAUSS)
    fprintf(fp,"gamma %g\n", param.gamma);

  if(param.kernel_type == POLY || param.kernel_type == SIGMOID || param.kernel_type == GENERALIZEDGAUSS)
    fprintf(fp,"coef0 %g\n", param.coef0);

  // -------------------------------
  // Save local kernel parameters
  // -------------------------------
  if (param.kernel_type == LOCALKERNEL1)
  {
    fprintf(fp,"n_features %d\n", param.n_features);
    fprintf(fp,"featuredim %d\n", param.featuredim);
    fprintf(fp,"nummax %d\n", param.nummax);
    fprintf(fp,"simthresh %g\n", param.simthresh);
    fprintf(fp,"localKernelType %d\n", param.localKernelType);
    fprintf(fp,"posType %d\n", param.posType);
    fprintf(fp,"sigmaPos %g\n", param.sigmaPos);
    fprintf(fp,"distHistNeighbourhood %d\n", param.distHistNeighbourhood);
    fprintf(fp,"distHistBins %d\n", param.distHistBins);
    fprintf(fp,"surf_enhancements %d\n", param.surfEnhancements);
    fprintf(fp,"surf_laplacians %d\n", param.surfLaplacians);
  }
  if (param.kernel_type == LOCALKERNEL2)
  {
    fprintf(fp,"n_features %d\n", param.n_features);
    fprintf(fp,"nummax %d\n", param.nummax);
    fprintf(fp,"featuredim %d\n", param.featuredim);
    fprintf(fp,"dist_compensation_coeff %g\n", param.distCompensationCoeff);
    fprintf(fp,"surf_enhancements %d\n", param.surfEnhancements);
    fprintf(fp,"surf_laplacians %d\n", param.surfLaplacians);
  }

  // -------------------------------
  // Save number of classes and SVs
  // -------------------------------
  int nr_class = model->nr_class;
  int l = model->l;
  fprintf(fp, "nr_class %d\n", nr_class);
  fprintf(fp, "nr_hyp %d\n", model->nr_hyp);
  fprintf(fp, "total_sv %d\n",l);

  // -------------------------------
  // Save rhos
  // -------------------------------
  {
    fprintf(fp, "rho");
    int z;
    if(param.svm_type == ONE_AGAINST_ALL)
      z=nr_class;
    else
      z=nr_class*(nr_class-1)/2;

    for(int i=0;i<z;i++)
      fprintf(fp," %g",model->rho[i]);
    fprintf(fp, "\n");
  }


  // -------------------------------
  // Save ||w|| and average distances to the hyperplane and values of dec. func.
  // -------------------------------
  {
    int z;
    if(param.svm_type == ONE_AGAINST_ALL)
      z=nr_class;
    else
      z=nr_class*(nr_class-1)/2;

    fprintf(fp, "omegaNorm");
    for(int i=0;i<z;i++)
      fprintf(fp," %g",model->omegaNorm[i]);
    fprintf(fp, "\n");

    fprintf(fp, "avgDistP");
    for(int i=0;i<z;i++)
      fprintf(fp," %g",model->avgDistP[i]);
    fprintf(fp, "\n");

    fprintf(fp, "avgDistN");
    for(int i=0;i<z;i++)
      fprintf(fp," %g",model->avgDistN[i]);
    fprintf(fp, "\n");

    fprintf(fp, "avgFP");
    for(int i=0;i<z;i++)
      fprintf(fp," %g",model->avgFP[i]);
    fprintf(fp, "\n");

    fprintf(fp, "avgFN");
    for(int i=0;i<z;i++)
      fprintf(fp," %g",model->avgFN[i]);
    fprintf(fp, "\n");
  }


  // -------------------------------
  // Save class labels
  // -------------------------------
  if(model->label)
  {
    fprintf(fp, "label");
    for(int i=0;i<nr_class;i++)
      fprintf(fp," %d",model->label[i]);
    fprintf(fp, "\n");
  }

  // -------------------------------
  // Save numbers of SVs
  // -------------------------------
  if(model->nSV)
  {
    fprintf(fp, "nr_sv");
    for(int i=0;i<nr_class;i++)
      fprintf(fp," %d",model->nSV[i]);
    fprintf(fp, "\n");
  }

  // -------------------------------
  // Save SVs
  // -------------------------------
  fprintf(fp, "SV\n");
  const double * const *sv_coef = model->sv_coef;
  const svm_node * const *SV = model->SV;
  int z;
  if(param.svm_type == ONE_AGAINST_ALL)
    z=nr_class;
  else
    z=nr_class-1;

  for(int i=0;i<l;i++)
  {
    for(int j=0;j<z;j++)
      fprintf(fp, "%.16g ",sv_coef[j][i]);

    const svm_node *p = SV[i];
    while(p->index != -1)
    {
      fprintf(fp,"%d:%.8g ",p->index,p->value);
      p++;
    }
    fprintf(fp, "\n");
  }

  fclose(fp);
  return 0;
}


/**
 * Modifies order of classes in the model according to classOrder
 */
void svm_reorganize_model(svm_model *model, const int* classOrder)
{
  int nrClass=model->nr_class;
  int nrHyp=model->nr_hyp;
  int l=model->l;

  // Print info message
  printf("Reorganizing model according to the scheme: [ ");
  for (int i=0; i<nrClass; ++i)
    printf("%d ", classOrder[i]);
  printf("]\n");

  // Get inverted classOrder
  int *classOrderInv = Malloc(int, nrClass);
  for(int i=0; i<nrClass; ++i)
    for(int j=0; j<nrClass; ++j)
      if (classOrder[j]==i)
        classOrderInv[i]=j;

  // Check if we deal with one-against-one or one-against-all
  if (model->param.svm_type==ONE_AGAINST_ALL)
  {
    // Create new arrays
    double *newRho = Malloc(double, nrClass);
    double *newOmegaNorm = Malloc(double, nrClass);
    double *newAvgDistP = Malloc(double, nrClass);
    double *newAvgDistN = Malloc(double, nrClass);
    double *newAvgFP = Malloc(double, nrClass);
    double *newAvgFN = Malloc(double, nrClass);
    int *newLabel = Malloc(int, nrClass);
    int *newNSV = Malloc(int, nrClass);
    double **newSvCoeff=0;
    if (model->sv_coef)
    {
      newSvCoeff= Malloc(double *, nrClass);
      for(int i=0; i<nrClass; ++i)
        newSvCoeff[i] = Malloc(double, l);
    }
    svm_node **newSV=0;
    svm_node *xspace=0;
    svm_node *newXSpace=0;
    if (model->SV)
    {
      xspace=model->SV[0];
      newSV=Malloc(svm_node*,l);
      //Find xspace size
      int xSpaceSize=0;
      {
        int j=0;
        for(int i=0; i<l; ++i)
        {
          for( ;xspace[j].index!=-1; ++j)
            ++xSpaceSize;
          ++xSpaceSize; //-1 elem
          ++j;
        }
      }
      // Allocate new xspace
      newXSpace = Malloc(svm_node, xSpaceSize);
      // Set the beginning of SV to be pointing to xspace
      newSV[0]=newXSpace;
    }

    // Handling rho, omegaNorm, avgDistP, avgDistN, avgFP, avgFN, label, nSV
    for (int i=0; i<nrClass; ++i)
    {
      newRho[classOrder[i]] = model->rho[i];
      newOmegaNorm[classOrder[i]]=model->omegaNorm[i];
      newAvgDistP[classOrder[i]]=model->avgDistP[i];
      newAvgDistN[classOrder[i]]=model->avgDistN[i];
      newAvgFP[classOrder[i]]=model->avgFP[i];
      newAvgFN[classOrder[i]]=model->avgFN[i];
      newLabel[classOrder[i]]=model->label[i];
      newNSV[classOrder[i]]=model->nSV[i];
    }
    // Handling sv_coef
    int *newStart=0;
    if (model->sv_coef)
    {
      newStart=Malloc(int, nrClass);
      newStart[0]=0;
      for(int i=1; i<nrClass; ++i)
        newStart[i]=newStart[i-1]+newNSV[i-1];
      int st=0;
      for(int i=0; i<nrClass; ++i)
      {
        for(int j=0; j<model->nSV[i]; ++j)
        {
          for(int k=0; k<nrClass; ++k)
            newSvCoeff[classOrder[k]][newStart[classOrder[i]]+j]=model->sv_coef[k][st+j];
        }
        st+=model->nSV[i];
      }
    }
    // Handling SV
    int *start=0;
    if (model->SV)
    {
      start=Malloc(int,nrClass);
      start[0]=0;
      for(int i=1; i<nrClass; ++i)
        start[i]=start[i-1]+model->nSV[i-1];
      int newSt=0;
      int xSpaceIndex=0;
      for(int i=0; i<nrClass; ++i)
      {
        for(int j=0; j<newNSV[i]; ++j)
        {
          newSV[newSt+j]=newXSpace+xSpaceIndex;
          for(int k=0; (model->SV[start[classOrderInv[i]]+j][k].index)>=0; ++k)
          {
            newXSpace[xSpaceIndex]=model->SV[start[classOrderInv[i]]+j][k];
            ++xSpaceIndex;
          }
          newXSpace[xSpaceIndex].index=-1;
          ++xSpaceIndex;
        }
        newSt+=newNSV[i];
      }
    }

    // Free temporary arrays
    free(model->rho);
    model->rho=newRho;
    free(model->omegaNorm);
    model->omegaNorm=newOmegaNorm;
    free(model->avgDistP);
    model->avgDistP=newAvgDistP;
    free(model->avgDistN);
    model->avgDistN=newAvgDistN;
    free(model->avgFP);
    model->avgFP=newAvgFP;
    free(model->avgFN);
    model->avgFN=newAvgFN;
    free(model->label);
    model->label=newLabel;
    free(model->nSV);
    model->nSV=newNSV;
    if (model->sv_coef)
    {
      for(int i=0; i<nrClass; ++i)
        free(model->sv_coef[i]);
      free(model->sv_coef);
      model->sv_coef=newSvCoeff;
    }
    if (model->SV)
    {
      free(model->SV[0]);
      free(model->SV);
      model->SV=newSV;
    }
    free(start);
    free(newStart);
  }
  // ------------------------------------------------------------------
  else if (model->param.svm_type==C_SVC)
  { // ONE AGAINST ONE
    // Create new arrays
    double *newRho = Malloc(double, nrHyp);
    double *newOmegaNorm = Malloc(double, nrHyp);
    double *newAvgDistP = Malloc(double, nrHyp);
    double *newAvgDistN = Malloc(double, nrHyp);
    double *newAvgFP = Malloc(double, nrHyp);
    double *newAvgFN = Malloc(double, nrHyp);
    int *newLabel = Malloc(int, nrClass);
    int *newNSV = Malloc(int, nrClass);
    double **newSvCoeff = 0;
    if (model->sv_coef)
    {
      newSvCoeff = Malloc(double *, nrClass-1);
      for(int i=0; i<nrClass-1; ++i)
        newSvCoeff[i] = Malloc(double, l);
    }
    svm_node **newSV=0;
    svm_node *xspace=0;
    svm_node *newXSpace=0;
    if (model->SV)
    {
      newSV=Malloc(svm_node*,l);
      //Find xspace size
      int xSpaceSize=0;
      xspace=model->SV[0];
      {
        int j=0;
        for(int i=0; i<l; ++i)
        {
          for( ;xspace[j].index!=-1; ++j)
            ++xSpaceSize;
          ++xSpaceSize; //-1 elem
          ++j;
        }
      }
      // Allocate new xspace
      newXSpace = Malloc(svm_node, xSpaceSize);
      // Set the beginning of SV to be pointing to xspace
      newSV[0]=newXSpace;
    }

    // Handling rho, omegaNorm, avgDistP, avgDistN, avgFP, avgFN,
    int p=0;
    for(int i=0; i<nrClass; ++i)
      for(int j=i+1; j<nrClass; ++j)
      {
        // Translate the class numbers
        int a = classOrder[i];
        int b = classOrder[j];
        bool reverse=false;
        if (a>b)
        { int tmp=a; a=b; b=tmp; reverse=true; }
        // Get new p value according to eq: p=a(n-(1+a)/2)+b-a-1 (assumption a<b)
        int newP=(a*nrClass-(a+a*a)/2)+b-a-1;
        // Do conversion
        newOmegaNorm[newP]=model->omegaNorm[p];
        if (reverse)
        {
          newRho[newP] = -model->rho[p];
          newAvgDistN[newP]=model->avgDistP[p];
          newAvgDistP[newP]=model->avgDistN[p];
          newAvgFN[newP]=model->avgFP[p];
          newAvgFP[newP]=model->avgFN[p];
        }
        else
        {
          newRho[newP] = model->rho[p];
          newAvgDistP[newP]=model->avgDistP[p];
          newAvgDistN[newP]=model->avgDistN[p];
          newAvgFP[newP]=model->avgFP[p];
          newAvgFN[newP]=model->avgFN[p];
        }
        ++p;
      }
    // Handling label, nSV
    for (int i=0; i<nrClass; ++i)
    {
      newLabel[classOrder[i]]=model->label[i];
      newNSV[classOrder[i]]=model->nSV[i];
    }
    // Handling sv_coef
    int *newStart=0;
    if (model->sv_coef)
    {
      newStart=Malloc(int, nrClass);
      newStart[0]=0;
      for(int i=1; i<nrClass; ++i)
        newStart[i]=newStart[i-1]+newNSV[i-1];
      int st=0;
      for(int i=0; i<nrClass; ++i)
      {
        for(int j=0; j<model->nSV[i]; ++j)
        {
          for(int p=0; p<nrClass-1; ++p)
          {
            // Find classes for the coefficient
            int a=-1, b=-1;
            int tmp=0;
            for(int u=0; u<nrClass; ++u)
              for(int v=u+1; v<nrClass; ++v)
              {
                if ((u==i) || (v==i))
                {
                  if (tmp==p) {a=u; b=v;}
                  ++tmp;
                }
              }
            // Convert
            int newA=classOrder[a];
            int newB=classOrder[b];
            double multiplier=1.0;
            if (newA>newB)
            { int exch=newA; newA=newB; newB=exch; multiplier=-1.0; }
            // Find newP
            tmp=0;
            int newP=-1;
            for(int u=0; u<nrClass; ++u)
              for(int v=u+1; v<nrClass; ++v)
              {
                if ((u==classOrder[i]) || (v==classOrder[i]))
                {
                  if ((u==newA) && (v==newB))
                    newP=tmp;
                  ++tmp;
                }
              }
            //          printf("asasd p%d newp%d a%d b%d newA%d newB%d\n", p, newP, a, b, newA, newB);
            newSvCoeff[newP][newStart[classOrder[i]]+j]=multiplier*model->sv_coef[p][st+j];
          }
        }
        st+=model->nSV[i];
      }
    }
    // Handling SV
    int *start=0;
    if (model->SV)
    {
      start=Malloc(int,nrClass);
      start[0]=0;
      for(int i=1; i<nrClass; ++i)
        start[i]=start[i-1]+model->nSV[i-1];
      int newSt=0;
      int xSpaceIndex=0;
      for(int i=0; i<nrClass; ++i)
      {
        for(int j=0; j<newNSV[i]; ++j)
        {
          newSV[newSt+j]=newXSpace+xSpaceIndex;
          for(int k=0; (model->SV[start[classOrderInv[i]]+j][k].index)>=0; ++k)
          {
            newXSpace[xSpaceIndex]=model->SV[start[classOrderInv[i]]+j][k];
            ++xSpaceIndex;
          }
          newXSpace[xSpaceIndex].index=-1;
          ++xSpaceIndex;
        }
        newSt+=newNSV[i];
      }
    }

    // Free temporary arrays
    free(model->rho);
    model->rho=newRho;
    free(model->omegaNorm);
    model->omegaNorm=newOmegaNorm;
    free(model->avgDistP);
    model->avgDistP=newAvgDistP;
    free(model->avgDistN);
    model->avgDistN=newAvgDistN;
    free(model->avgFP);
    model->avgFP=newAvgFP;
    free(model->avgFN);
    model->avgFN=newAvgFN;
    free(model->label);
    model->label=newLabel;
    free(model->nSV);
    model->nSV=newNSV;
    if (model->sv_coef)
    {
      for(int i=0; i<nrClass-1; ++i)
        free(model->sv_coef[i]);
      free(model->sv_coef);
      model->sv_coef=newSvCoeff;
    }
    if (model->SV)
    {
      free(model->SV[0]);
      free(model->SV);
      model->SV=newSV;
    }
    free(start);
    free(newStart);

  }
  else
  {
    fprintf(stderr, "ERROR: Model of this type cannot be reorganized!");
    exit(-1);
  }

  // Free
  free(classOrderInv);
}


/**
 * Loads an SVM model from file.
 * If inputLabel is not equal to 0, the order of labels in the loaded model
 * will be modified according to inputLabel.
 */
svm_model *svm_load_model(const char *model_file_name, bool noSVs)
{
  FILE *fp = fopen(model_file_name,"rb");
  if(fp==NULL) return NULL;

  // -------------------------------
  // Read parameters
  // -------------------------------
  svm_model *model = Malloc(svm_model,1);
  svm_parameter& param = model->param;
  model->rho = NULL;
  model->omegaNorm = NULL;
  model->avgDistP = NULL;
  model->avgDistN = NULL;
  model->avgFP = NULL;
  model->avgFN = NULL;
  model->label = NULL;
  model->nSV = NULL;
  model->nr_hyp = 0;

  char cmd[81];
  while(1)
  {
    fscanf(fp,"%80s",cmd);

    // -------------------------------
    // SVM and kernel type
    // -------------------------------
    if(strcmp(cmd,"svm_type")==0)
    {
      fscanf(fp,"%80s",cmd);
      int i;
      for(i=0;svm_type_table[i];i++)
      {
        if(strcmp(svm_type_table[i],cmd)==0)
        {
          param.svm_type=i;
          break;
        }
      }
      if(svm_type_table[i] == NULL)
      {
        fprintf(stderr,"unknown svm type.\n");
        free(model->rho);
        free(model->label);
        free(model->nSV);
        free(model);
        return NULL;
      }
    }
    else if(strcmp(cmd,"kernel_type")==0)
    {
      fscanf(fp,"%80s",cmd);
      int i;
      for(i=0;kernel_type_table[i];i++)
      {
        if(strcmp(kernel_type_table[i],cmd)==0)
        {
          param.kernel_type=i;
          break;
        }
      }
      if(kernel_type_table[i] == NULL)
      {
        fprintf(stderr,"unknown kernel function.\n");
        free(model->rho);
        free(model->label);
        free(model->nSV);
        free(model);
        return NULL;
      }
    }

    // -------------------------------
    // Kernel parameters
    // -------------------------------
    else if(strcmp(cmd,"degree")==0)
      fscanf(fp,"%lf",&param.degree);
    else if(strcmp(cmd,"gamma")==0)
      fscanf(fp,"%lf",&param.gamma);
    else if(strcmp(cmd,"coef0")==0)
      fscanf(fp,"%lf",&param.coef0);

    // -------------------------------
    // Local kernel parameters
    // -------------------------------
    else if(strcmp(cmd,"n_features")==0)
      fscanf(fp,"%d",&param.n_features);
    else if(strcmp(cmd,"featuredim")==0)
      fscanf(fp,"%d",&param.featuredim);
    else if(strcmp(cmd,"nummax")==0)
      fscanf(fp,"%d",&param.nummax);
    else if(strcmp(cmd,"simthresh")==0)
      fscanf(fp,"%lf",&param.simthresh);
    else if(strcmp(cmd,"localKernelType")==0)
      fscanf(fp,"%d",&param.localKernelType);
    else if(strcmp(cmd,"posType")==0)
      fscanf(fp,"%d",&param.posType);
    else if(strcmp(cmd,"sigmaPos")==0)
      fscanf(fp,"%lf",&param.sigmaPos);
    else if(strcmp(cmd,"distHistNeighbourhood")==0)
      fscanf(fp,"%d",&param.distHistNeighbourhood);
    else if(strcmp(cmd,"distHistBins")==0)
      fscanf(fp,"%d",&param.distHistBins);

    // -------------------------------
    // Fast SIFT Local kernel parameters
    // -------------------------------
    else if(strcmp(cmd,"dist_compensation_coeff")==0)
      fscanf(fp,"%lf",&param.distCompensationCoeff);
    else if(strcmp(cmd,"surf_enhancements")==0)
      fscanf(fp,"%d",&param.surfEnhancements);
    else if(strcmp(cmd,"surf_laplacians")==0)
      fscanf(fp,"%d",&param.surfLaplacians);

    // -------------------------------
    // Number of classes and SVs
    // -------------------------------
    else if(strcmp(cmd,"nr_class")==0)
      fscanf(fp,"%d",&model->nr_class);
    else if(strcmp(cmd,"nr_hyp")==0)
      fscanf(fp,"%d",&model->nr_hyp);
    else if(strcmp(cmd,"total_sv")==0)
      fscanf(fp,"%d",&model->l);

    // -------------------------------
    // Rhos
    // -------------------------------
    else if(strcmp(cmd,"rho")==0)
    {
      int n;
      if(param.svm_type == ONE_AGAINST_ALL)
        n = model->nr_class;
      else
        n = model->nr_class * (model->nr_class-1)/2;
      model->rho = Malloc(double,n);
      for(int i=0;i<n;i++)
        fscanf(fp,"%lf",&model->rho[i]);
    }

    // -------------------------------
    // ||w|| and average distances to hyperplanes and avg. dec. func values
    // -------------------------------
    else if(strcmp(cmd,"omegaNorm")==0)
    {
      int n;
      if(param.svm_type == ONE_AGAINST_ALL)
        n = model->nr_class;
      else
        n = model->nr_class * (model->nr_class-1)/2;
      model->omegaNorm = Malloc(double,n);
      for(int i=0;i<n;i++)
        fscanf(fp,"%lf",&model->omegaNorm[i]);
    }
    else if(strcmp(cmd,"avgDistP")==0)
    {
      int n;
      if(param.svm_type == ONE_AGAINST_ALL)
        n = model->nr_class;
      else
        n = model->nr_class * (model->nr_class-1)/2;
      model->avgDistP = Malloc(double,n);
      for(int i=0;i<n;i++)
        fscanf(fp,"%lf",&model->avgDistP[i]);
    }
    else if(strcmp(cmd,"avgDistN")==0)
    {
      int n;
      if(param.svm_type == ONE_AGAINST_ALL)
        n = model->nr_class;
      else
        n = model->nr_class * (model->nr_class-1)/2;
      model->avgDistN = Malloc(double,n);
      for(int i=0;i<n;i++)
        fscanf(fp,"%lf",&model->avgDistN[i]);
    }
    else if(strcmp(cmd,"avgFP")==0)
    {
      int n;
      if(param.svm_type == ONE_AGAINST_ALL)
        n = model->nr_class;
      else
        n = model->nr_class * (model->nr_class-1)/2;
      model->avgFP = Malloc(double,n);
      for(int i=0;i<n;i++)
        fscanf(fp,"%lf",&model->avgFP[i]);
    }
    else if(strcmp(cmd,"avgFN")==0)
    {
      int n;
      if(param.svm_type == ONE_AGAINST_ALL)
        n = model->nr_class;
      else
        n = model->nr_class * (model->nr_class-1)/2;
      model->avgFN = Malloc(double,n);
      for(int i=0;i<n;i++)
        fscanf(fp,"%lf",&model->avgFN[i]);
    }


    // -------------------------------
    // Class labels
    // -------------------------------
    else if(strcmp(cmd,"label")==0)
    {
      int n = model->nr_class;
      model->label = Malloc(int,n);
      for(int i=0;i<n;i++)
        fscanf(fp,"%d",&model->label[i]);
    }

    // -------------------------------
    // Numbers of SVs
    // -------------------------------
    else if(strcmp(cmd,"nr_sv")==0)
    {
      int n = model->nr_class;
      model->nSV = Malloc(int,n);
      for(int i=0;i<n;i++)
        fscanf(fp,"%d",&model->nSV[i]);
    }

    // -------------------------------
    // SVs
    // -------------------------------
    else if(strcmp(cmd,"SV")==0)
    {
      while(1)
      {
        int c = getc(fp);
        if(c==EOF || c=='\n') break;
      }
      break;
    }

    // -------------------------------
    // Parallelized training
    // -------------------------------
    else if(strcmp(cmd,"trainOnlyClass")==0)
    {
      fscanf(fp,"%d",&param.trainOnlyClass);
      //printf("trainOnlyClass: %d\n", param.trainOnlyClass);
      if (!(param.trainOnlyClass == -1 || param.trainOnlyClass == -2))
      {
        fprintf(stderr,"This model file only one trained sub-problem from a parallelized training.\n");
        free(model->rho);
        free(model->label);
        free(model->nSV);
        free(model);
        return NULL;
      }
    }
    else
    {
      fprintf(stderr,"unknown text in model file\n");
      free(model->rho);
      free(model->label);
      free(model->nSV);
      free(model);
      return NULL;
    }
  }

  // Patch for models that do not have nr_hyp inside
  if (model->nr_hyp==0)
  {
    if (param.svm_type==ONE_AGAINST_ALL)
      model->nr_hyp=model->nr_class;
    else if (param.svm_type==C_SVC)
      model->nr_hyp=(model->nr_class*(model->nr_class-1))/2;
  }

  // Skip reading SVs if we don't need it
  if (noSVs)
  {
    model->l=0;
    model->sv_coef=0;
    model->SV=0;
    model->free_sv = 0; // Do not try to free SVs
    return model;
  }

  // Read sv_coef and SV
  int elements = 0;
  long pos = ftell(fp);

  // Count number of elements
  while(1)
  {
    int c = fgetc(fp);
    switch(c)
    {
      case '\n':
        // count the '-1' element
      case ':':
        ++elements;
        break;
      case EOF:
        goto out;
      default:
        ;
    }
  }
out:
  fseek(fp,pos,SEEK_SET);
  int m;
  if(param.svm_type == ONE_AGAINST_ALL)
    m = model->nr_class;
  else
    m = model->nr_class - 1; // Each training vector can be a SV in nr_class-1 classifiers only
  int l = model->l;
  model->sv_coef = Malloc(double *,m);
  int i;
  for(i=0;i<m;i++)
    model->sv_coef[i] = Malloc(double,l);
  model->SV = Malloc(svm_node*,l);
  svm_node *x_space = Malloc(svm_node,elements);

  int j=0;
  for(i=0;i<l;i++)
  {
    model->SV[i] = &x_space[j];
    for(int k=0;k<m;k++)
      fscanf(fp,"%lf",&model->sv_coef[k][i]);
    while(1)
    {
      int c;
      do {
        c = getc(fp);
        if(c=='\n') goto out2;
      } while(isspace(c));
      ungetc(c,fp);
      fscanf(fp,"%d:%lf",&(x_space[j].index),&(x_space[j].value));
      ++j;
    }
out2:
    x_space[j++].index = -1;
  }

  fclose(fp);

  model->free_sv = 1; // XXX
  return model;
}

void svm_destroy_model(svm_model* model)
{
  if(model->free_sv)
    free((void *)(model->SV[0]));
  if (model->sv_coef)
  {
    for(int i=0;i<model->nr_class-1;i++)
      free(model->sv_coef[i]);
    free(model->sv_coef);
  }
  if (model->SV)
    free(model->SV);
  free(model->rho);
  free(model->label);
  free(model->omegaNorm);
  free(model->avgDistP);
  free(model->avgDistN);
  free(model->avgFP);
  free(model->avgFN);
  free(model->nSV);
  free(model);
}

const char *svm_check_parameter(const svm_problem *prob, const svm_parameter *param)
{
  // svm_type

  int svm_type = param->svm_type;
  if(svm_type != C_SVC &&
     svm_type != NU_SVC &&
     svm_type != ONE_CLASS &&
     svm_type != EPSILON_SVR &&
     svm_type != NU_SVR &&
     svm_type != ONE_AGAINST_ALL)
    return "unknown svm type";

  // kernel_type

  int kernel_type = param->kernel_type;
  if(kernel_type != LINEAR &&
     kernel_type != POLY &&
     kernel_type != RBF &&
     kernel_type != SIGMOID &&
     kernel_type != LOCALKERNEL1 &&
     kernel_type != LOCALKERNEL2 &&
     kernel_type != CHISQUARED &&
     kernel_type != INTERSECTION &&
     kernel_type != GENERALIZEDGAUSS)
    return "unknown kernel type";

  // cache_size,eps,C,nu,p,shrinking

  if(param->cache_size <= 0)
    return "cache_size <= 0";

  if(param->eps <= 0)
    return "eps <= 0";

  if(svm_type == C_SVC ||
     svm_type == EPSILON_SVR ||
     svm_type == NU_SVR)
    if(param->C <= 0)
      return "C <= 0";

  if(svm_type == NU_SVC ||
     svm_type == ONE_CLASS ||
     svm_type == NU_SVR)
    if(param->nu < 0 || param->nu > 1)
      return "nu < 0 or nu > 1";

  if(svm_type == EPSILON_SVR)
    if(param->p < 0)
      return "p < 0";

  if(param->shrinking != 0 &&
     param->shrinking != 1)
    return "shrinking != 0 and shrinking != 1";


  // check whether nu-svc is feasible

  if(svm_type == NU_SVC)
  {
    int l = prob->l;
    int max_nr_class = 16;
    int nr_class = 0;
    int *label = Malloc(int,max_nr_class);
    int *count = Malloc(int,max_nr_class);

    int i;
    for(i=0;i<l;i++)
    {
      int this_label = (int)prob->y[i];
      int j;
      for(j=0;j<nr_class;j++)
        if(this_label == label[j])
        {
          ++count[j];
          break;
        }
      if(j == nr_class)
      {
        if(nr_class == max_nr_class)
        {
          max_nr_class *= 2;
          label = (int *)realloc(label,max_nr_class*sizeof(int));
          count = (int *)realloc(count,max_nr_class*sizeof(int));
        }
        label[nr_class] = this_label;
        count[nr_class] = 1;
        ++nr_class;
      }
    }

    for(i=0;i<nr_class;i++)
    {
      int n1 = count[i];
      for(int j=i+1;j<nr_class;j++)
      {
        int n2 = count[j];
        if(param->nu*(n1+n2)/2 > min(n1,n2))
        {
          free(label);
          free(count);
          return "specified nu is infeasible";
        }
      }
    }
  }

  return NULL;
}



/*
 // Check if we deal with one-against-one or one-against-all
  if (model->param.svm_type==ONE_AGAINST_ALL)
  {
    // Create temporary arrays
    double *tmpRho = Malloc(double, nrClass);
    memcpy(tmpRho, model->rho, sizeof(double)*nrClass);
    double *tmpOmegaNorm = Malloc(double, nrClass);
    memcpy(tmpOmegaNorm, model->omegaNorm, sizeof(double)*nrClass);
    double *tmpAvgDistP = Malloc(double, nrClass);
    memcpy(tmpAvgDistP, model->avgDistP, sizeof(double)*nrClass);
    double *tmpAvgDistN = Malloc(double, nrClass);
    memcpy(tmpAvgDistN, model->avgDistN, sizeof(double)*nrClass);
    double *tmpAvgFP = Malloc(double, nrClass);
    memcpy(tmpAvgFP, model->avgFP, sizeof(double)*nrClass);
    double *tmpAvgFN = Malloc(double, nrClass);
    memcpy(tmpAvgFN, model->avgFN, sizeof(double)*nrClass);
    int *tmpLabel = Malloc(int, nrClass);
    memcpy(tmpLabel, model->label, sizeof(int)*nrClass);
    int *tmpNSV = Malloc(int, nrClass);
    memcpy(tmpNSV, model->nSV, sizeof(int)*nrClass);
    double **tmpSvCoeff = Malloc(double *, nrClass);
    for(int i=0; i<nrClass; ++i)
    {
      tmpSvCoeff[i] = Malloc(double, l);
      memcpy(tmpSvCoeff[i], model->sv_coef[i], sizeof(double)*l);
    }
    svm_node **tmpSV=Malloc(svm_node*,l);
    memcpy(tmpSV, model->SV, sizeof(svm_node*)*l);

    // Handling rho, omegaNorm, avgDistP, avgDistN, avgFP, avgFN, label, nSV and sv_coef
    for (int i=0; i<nrClass; ++i)
    {
      model->rho[classOrder[i]]=tmpRho[i];
      model->omegaNorm[classOrder[i]]=tmpOmegaNorm[i];
      model->avgDistP[classOrder[i]]=tmpAvgDistP[i];
      model->avgDistN[classOrder[i]]=tmpAvgDistN[i];
      model->avgFP[classOrder[i]]=tmpAvgFP[i];
      model->avgFN[classOrder[i]]=tmpAvgFN[i];
      model->label[classOrder[i]]=tmpLabel[i];
      model->nSV[classOrder[i]]=tmpNSV[i];
    }
    // Handling SV and sv_coef
    int *start=Malloc(int, nrClass);
    start[0]=0;
    for(int i=1; i<nrClass; ++i)
      start[i]=start[i-1]+model->nSV[i-1];
    int tmpStart=0;
    for(int i=0; i<nrClass; ++i)
    {
      for(int j=0; j<tmpNSV[i]; ++j)
      {
        // SV
        model->SV[start[classOrder[i]]+j]=tmpSV[tmpStart+j];
        // sv_coef
        for(int k=0; k<nrClass; ++k)
          model->sv_coef[k][start[classOrder[i]]+j]=tmpSvCoeff[k][tmpStart+j];
      }
      tmpStart+=tmpNSV[i];
    }
    // Handling sv_coef
    // Free temporary arrays
    free(tmpRho);
    free(tmpOmegaNorm);
    free(tmpAvgDistP);
    free(tmpAvgDistN);
    free(tmpAvgFP);
    free(tmpAvgFN);
    free(tmpLabel);
    free(tmpNSV);
    for(int i=0; i<nrClass; ++i)
      free(tmpSvCoeff[i]);
    free(tmpSvCoeff);
    free(tmpSV);
    free(start);
  }
  else if (model->param.svm_type==C_SVC)
  {

  }
  else
  {
    fprintf(stderr, "ERROR: Model of this type cannot be reorganized!");
    exit(-1);
  }
*/
