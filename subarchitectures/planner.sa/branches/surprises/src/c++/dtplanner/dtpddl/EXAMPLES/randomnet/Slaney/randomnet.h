#include <stdio.h>
#include <stdlib.h>


#define MAX_FEEDERS 20
#define MAX_KIDS 6
#define NAMELENGTH 20
#define RMAX 100
#define SDMAX 1000

/*
 * Here are a couple of useful universal quantifiers
 */

#define forall_trees(f,x) for (x=0; x<MAX_FEEDERS && f[x]; x++)
#define forall_kids(n,x) for (x=0; x<MAX_KIDS && n->kids[x]; x++)


typedef enum {
  false, true
} boolean;


typedef enum {
  cb, sd, line, object_types
} object;


typedef struct node_of_tree {
  object sort;                          /* CB or SD */
  int id;                               /* Unique identifier */
  int oldid;                            /* Copy of the id */
  char name[NAMELENGTH];                /* ID for printing */
  char oldname[NAMELENGTH];             /* Copy of name */
  int line_out;                         /* Downstream line id */
  boolean reversed;                     /* Line out from side 1 */
  boolean oldreversed;                  /* Copy of reversed */
  boolean terminal;                     /* Nothing downstream */
  boolean closed;                       /* State of the switch */
  struct node_of_tree *kids[MAX_KIDS];  /* Other ends of line out */
} NODE, *node;


typedef struct {
  node root;            /* Circuit breaker of the feeder tree */
  int component;        /* Which connected set of trees it is in */
} TREE, *tree;



/*
 * ------------- prototypes (including parameter names) -------------
 */

void get_options(int argc, char *argv[],
                 double *T, double *B, double *X, double *C,
                 int *R, int *N, int *S, int *Z, boolean *opens, double *F,
		 boolean *help, int *lmin, int *lmax);
tree generate_tree(double T, double B, double X, int next_id[]);
void extend(node n, double T, double B, double X, int next_id[]);
node new_node();
boolean joined_trees(tree feeder[], int R);
void set_old_id(node n);
boolean this_time_for_sure(tree feeder[]);
void join_components(tree feeder[], int ca, int cb);
void join_nodes(tree feeder[], int ca, int cb, node na, node nb);
node random_node(tree feeder[], int c);
node get_terminal_node(tree feeder[], int c, int r);
node get_terminal_in_tree(node n, int r);
boolean closed_component(tree feeder[], int c);
void reset_terminals(node n);
int terminal_nodes_in_component(tree feeder[], int c);
int terminal_nodes_in_tree(node n);
void no_loose_ends(tree feeder[], double C, int next_id[]);
void print_opens(node n, boolean printed_opens[]);
void rename_everything(tree feeder[], int next_id[]);
int *set_sd_index(int maxsd, tree feeder[]);
void set_sd_used(boolean used[], node n);
int *set_line_index(int maxline, tree feeder[]);
void set_line_used(boolean used[], node n);
void print_network(tree feeder[], int next_id[], boolean opens,
                   double F, int Z);
void change_names(node n, int sdindex[], int lineindex[]);
void print_one_line(node ni, boolean parent, int x, node no);
void print_all_lines(node n);
int count_lines(node n);
int max_sd_id(node n, int sofar);
void print_failure_message();
void print_the_job(double T, double B, double X, double C,
		   int R, int N, int S);
boolean probably_infinite(double T, double B, double X);
void find_faulty(int linecount, boolean faulty[], double F);
void print_faulty(boolean faults[], int linecount);
void list_node(node n, node thenode[]);
int f2d(double f);
void rabort(char * s);
void free_memory(tree f[]);
void free_all_nodes(node n);
void print_help();
