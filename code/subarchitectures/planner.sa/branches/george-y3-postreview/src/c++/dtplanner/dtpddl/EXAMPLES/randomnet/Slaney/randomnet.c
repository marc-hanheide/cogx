/*
 * Generate a random power distribution network by first generating
 * the specified number of feeders (trees downstream from the circuit
 * breakers) and then generating the connections between them.

 * Each tree starts with a root node (the circuit breaker) and each
 * nonterminal node is extended or terminated until the tree is
 * done. The extension is governed by three probabilities:

 *   T  the probability that the node is terminal
 *   B  the probability that the node is a branch point given that
 *      it is not terminal
 *   X  the prabability that it branches at least n+1 ways given that
 *      it branches at least n ways (for n > 1)

 * Suggested (default) values are  T = 0.45,  B = 0.4,  X = 0.15.

 * Each tree is a separate component of the overall graph, with one or
 * more terminal nodes. While there are separated components, randomly
 * choose two of them and join them by identifying a terminal node of
 * one with a terminal node of the other. The merged node is then no
 * longer terminal. If a connected graph results, consider each
 * terminal node in turn in random order and with probability C
 * identify it with another terminal node, subject to the constraint
 * that no two of the trees be joined twice. If no connected graph
 * results, go back and repeat the merging process from the start.
 * Repeat up to R times, then print the network, connected or not.

 * Suggested (default) values for these parameters are C = 0.5 R = 100.

 * A random number seed S (an integer) should also be given.

 * There is a maximum number MAX_FEEDERS of feeders, and a maximum
 * number MAX_KIDS of children of a node (ways it can branch). These
 * are set in randomnet.h where the data structures are also defined.

 * R also has a maximum value set in randomnet.h but this is less
 * critical and is only needed to keep things within reasonable limits.

 * Command line option -o causes the list of devices to be printed
 * after the configuration, noting which are open and which are closed.

 * Because we generally do not want feeders that are "too big" (nor ones
 * that are too small) the program outputs only the results that lie in
 * a window from lmin to lmax. By default this is 2 to 10, but it can be
 * any subrange of 1 to infinity. Note that lmin and lmax limit the
 * number of switching devices (minus the circuit breaker). Note also that
 * it is not generally a good idea to set the lower bound too close to the
 * upper bound, just so that the generator has a reasonable degree of
 * choice about the branching topology.

 * Another option is to choose a number of "faulty" lines, also at random.
 * Option "f <proportion>" where <proportion> is a real between 0 and 1
 * causes the given proportion of lines to be selected and labelled as
 * faulty. A second random number seed Z should be given if faulty lines
 * are desired.
 */


#include "randomnet.h"

node debugger;

main(int argc, char *argv[])
{
  int N=3;              /* Number of feeders */
  double T=0.45;        /* Prob that node is terminal */
  double B=0.4;         /* Prob that node branches given non-terminal */
  double X=0.15;        /* Prob that node branches again given branches */
  double C=0.5;         /* Prob that spare nodes join trees */
  int lmin=2;           /* Minimum number of lines per feeder */
  int lmax=10;          /* Maximum number of lines per feeder */
  int R=10;             /* Number of repeats before giving up */
  int S=1;              /* Random number seed for tree generation */
  int Z=1;              /* Random number seed for fault generation */
  boolean opens=false;  /* Print which devices are open/closed */
  double F=0.0;         /* Proportion of faulty lines */

  tree feeder[MAX_FEEDERS];      /* Pointers to the feeders */
  int next_id[object_types];     /* Just for naming purposes */
  int i;                         /* Local variable for iterations */
  int old_next_id[object_types]; /* Copy for iterations */
  boolean help = false;

  /*
   * Initialisation
   */
  for (i=0; i<MAX_FEEDERS; i++) feeder[i] = 0;
  for (i=0; i<object_types; i++) next_id[i] = 1;
  get_options(argc,argv,
	      &T,&B,&X,&C,&R,&N,&S,&Z,&opens,&F,&help,&lmin,&lmax);
  if (help) {
    print_help();
    exit(0);
  }

  srand48(S);

  /*
   * Check that the probability of termination (T) is sufficiently
   * large to make the trees (probably) finite, at least given that
   * no node has more than three children.
   */

  if (probably_infinite(T,B,X))
    rabort("Set T > B/(1+B-X), to avoid infinite trees.");

  /*
   * Generate the N feeders using the given probabilities of
   * termination, branching and extra branching. Repeat until
   * each tree is of an acceptable size.
   */
  for (i=0; i<N; i++) {
    old_next_id[cb] = next_id[cb];
    old_next_id[sd] = next_id[sd];
    old_next_id[line] = next_id[line];
    do {
      if (feeder[i]) {
	free_all_nodes(feeder[i]->root);
	free(feeder[i]);
	feeder[i] = 0;
      }
      next_id[cb] = old_next_id[cb];
      next_id[sd] = old_next_id[sd];
      next_id[line] = old_next_id[line];
      feeder[i] = generate_tree(T,B,X,next_id);
    }
    while (next_id[sd]-old_next_id[sd] < lmin ||
	   next_id[sd]-old_next_id[sd] > lmax);
  }

  /*
   * Try to make the trees into a connected graph by repeatedly
   * identifying a terminal node of one feeder with a terminal
   * node of another. Print a warning if no connected graph is
   * attained within R attempts. When the graph is connected,
   * join pairs of remaining terminal nodes (with probability C)
   * or earth them. Finally pretty-print the network.
   */
  if (!joined_trees(feeder,R)) print_failure_message();
  no_loose_ends(feeder,C,next_id);
  print_network(feeder,next_id,opens,F,Z);

  free_memory(feeder);
  exit(0);
}



/*
 * Read the command line arguments into the variables (see main)
 */


void get_options(int argc, char *argv[],
                 double *T, double *B, double *X, double *C,
                 int *R, int *N, int *S, int *Z, boolean *opens, double *F,
		 boolean *help, int *lmin, int *lmax)
{
  extern  char *optarg;
  int option;
  int x;

  while ((option = getopt(argc,argv,"b:c:f:hl:n:or:s:t:u:x:z:")) != -1)
    switch(option) {
    case 'b':
      sscanf(optarg,"%lf",B);
      if (*B < 0.0) *B = 0.0;
      if (*B > 1.0) *B = 1.0;
      break;
    case 'c':
      sscanf(optarg,"%lf",C);
      if (*C < 0.0) *C = 0.0;
      if (*C > 1.0) *C = 1.0;
      break;
    case 'f':
      sscanf(optarg,"%lf",F);
      if (*F < 0.0) *F = 0.0;
      if (*F > 1.0) *F = 1.0;
      break;
    case 'h':
      *help = true;
      break;
    case 'l':
      sscanf(optarg,"%d",lmin);
      if (*lmin < 1) *lmin = 1;
      if (*lmax > 0 && *lmax < *lmin) *lmax = *lmin;
      break;
    case 'n':
      sscanf(optarg,"%d",N);
      if (*N < 1) *N = 1;
      if (*N > MAX_FEEDERS) *N = MAX_FEEDERS;
      break;
    case 'o':
      *opens = true;
      break;
    case 'r':
      sscanf(optarg,"%d",R);
      if (*R < 1) *R = 1;
      if (*R > RMAX) *R = RMAX;
      break;
    case 's':
      sscanf(optarg,"%d",S);
      break;
    case 't':
      sscanf(optarg,"%lf",T);
      if (*T < 0.0) *T = 0.0;
      if (*T > 1.0) *T = 1.0;
      break;
    case 'u':
      sscanf(optarg,"%d",lmax);
      if (*lmax > 0 && *lmax < *lmin) *lmin = *lmax;
      break;
    case 'x':
      sscanf(optarg,"%lf",X);
      if (*X < 0.0) *X = 0.0;
      if (*X > 1.0) *X = 1.0;
      break;
    case 'z':
      sscanf(optarg,"%d",Z);
      break;
    }
}



/*
 * Set up a new tree, make its root node a circuit breaker which is
 * closed and feeds a line (i.e. we do not allow trees to consist of
 * just a circuit breaker) and call extend to build the non-root nodes.
 * Note that the line out of a circuit breaker always comes from side 2
 * so the randomisation of the "reversed" flag is overridden in the
 * case of the root node.
 */

tree generate_tree(double T, double B, double X, int next_id[])
{
  tree f;

  f = (tree) malloc(sizeof(TREE));
  if (!f) rabort("Could not allocate memory for feeder");

  f->root = new_node();
  f->root->sort = cb;
  f->root->id = next_id[cb]++;
  sprintf(f->root->name,"cb %d",f->root->id);
  f->root->reversed = false;
  f->root->terminal = false;
  f->root->closed = true;
  f->root->line_out = next_id[line]++;

  extend(f->root,T,B,X,next_id);
  return f;
}



/*
 * Recursively generate the children of each node until terminal ones
 * are reached. Abort the construction if too many nodes have been
 * generated: this almost certainly means that the probability of
 * branching (B) is set too high or the probability of terminating (T)
 * is set too low.
 *
 * Each child of node n is a new switching device. It is terminal (i.e.
 * a leaf of the feeder tree) with probability T. Non-terminal nodes
 * are closed (they allow current to flow to the lines downstream).
 * Terminal ones are open (they prevent current from passing). At this
 * stage, terminal nodes have no line out because there is nothing
 * downstream from them. Later they will be joined together to make a
 * grid, so there may be lines downstream if they get closed by some
 * switching actions when the network is in operation. Some terminal
 * nodes may later be earthed (have a line out which goes to earth
 * rather than to any further nodes) in which case they will become
 * closed and non-terminal, but at this stage they are still marked as
 * terminal because they form the boundary of the feeder.
 *
 */

void extend(node n, double T, double B, double X, int next_id[])
{
  int children = 1;
  int i;
  node nki;

  if (next_id[sd] > SDMAX) rabort("Too many switches");

  if (n->terminal)       /* No extension */
    return;

  if (drand48() > B)     /* Non-branching extension */
    children = 1;
  else                   /* Branching extension */
    for (children=2;
	 children < MAX_KIDS && drand48() < X;
	 children++);

  for (i=0; i<children; i++) {
    nki = new_node();
    nki->sort = sd;
    nki->id = next_id[sd]++;
    sprintf(nki->name,"sd %d",nki->id);
    nki->terminal = (drand48() < T);
    nki->closed = !(nki->terminal);
    if (nki->terminal) 
      nki->line_out = 0;
    else {
      nki->line_out = next_id[line]++;
      extend(nki,T,B,X,next_id);
    }
    n->kids[i] = nki;
  }
}


/*
 * This just grabs memory for a new node and initialises its kids.
 * The "reversed" flag means that the line out of this node will come
 * from side 1 rather than side 2. For no particularly good reason,
 * except to confuse planners, this is randomised.
 */

node new_node()
{
  node n;
  int i;

  n = (node) malloc(sizeof(NODE));
  if (!n) rabort("Could not allocate memory for device");

  for (i=0; i<MAX_KIDS; i++) n->kids[i] = 0;
  n->reversed = lrand48() % 2;
  return n;
}



/*
 * Attempt R times to join the trees by their terminal nodes, subject
 * to a limit of one connection between a pair of trees. Return
 * success or failure. Success means the network is a connected graph.
 *
 * The "old id" etc are needed here, so that the initial state of the
 * set of feeders can be recovered for repeat attempts at joining.
 */

boolean joined_trees(tree feeder[], int R)
{
  int i;

  forall_trees(feeder,i)
    set_old_id(feeder[i]->root);

  do {
    if (this_time_for_sure(feeder))
      return true;
  }
  while (--R);
  return false;
}



/*
 * Make a copy of the id, name and reversed flag, just for
 * book-keeping purposes.
 */

void set_old_id(node n)
{
  int i;

  n->oldid = n->id;
  strcpy(n->oldname,n->name);
  n->oldreversed = n->reversed;
  forall_kids(n,i)
    set_old_id(n->kids[i]);
}



/*
 * One iteration of attempting to join the feeder trees to make a
 * connected graph. While the graph is not connected, choose two of
 * its components and join them together by identifying a terminal
 * node of one with a terminal node of the other.
 *
 * The construction fails if at some point there are two components,
 * one of which has no terminal nodes, since that component cannot
 * be joined to the rest of the network.
 *
 * The construction succeeds if there is only one component.
 */

boolean this_time_for_sure(tree feeder[])
{
  int i;
  int number_of_components;
  int ca, cb;

  forall_trees(feeder,i) {
    feeder[i]->component = i;
    reset_terminals(feeder[i]->root);
  }

  number_of_components = i;

  while (number_of_components > 1) {
    ca = lrand48() % number_of_components;
    cb = lrand48() % (number_of_components-1);
    if (cb >= ca) cb++;
    if (ca > cb) {
      i = ca;
      ca = cb;
      cb = i;
    }
    join_components(feeder,ca,cb);
    number_of_components--;

    if (number_of_components > 1 &&
	terminal_nodes_in_component(feeder,ca) == 0)
      return false;
  }
  return true;
}


/*
 * For each iteration of attempting to join up the network, the forest
 * of feeders must be reset to its original state.
 */

void reset_terminals(node n)
{
  int i;

  if (!n) return;
  forall_kids(n,i)
    reset_terminals(n->kids[i]);
  n->id = n->oldid;
  strcpy(n->name,n->oldname);
  n->reversed = n->oldreversed;
  n->terminal = !(n->kids[0]);
}



/*
 * Count the terminal nodes in component c of the network.
 * This is obviously the sum of the numbers of terminal nodes in the feeder
 * trees making up the component.
 */

int terminal_nodes_in_component(tree feeder[], int c)
{
  int i;
  int k=0;

  forall_trees(feeder,i)
    if (feeder[i]->component == c)
      k += terminal_nodes_in_tree(feeder[i]->root);
  return k;
}


/*
 * The number of terminal nodes in a tree is the sum of the numbers for
 * its children, or 1 if the node is itself terminal. Why am I telling
 * you this? Pfffff!
 */

int terminal_nodes_in_tree(node n)
{
  int i,k=0;

  if (n->terminal) return 1;
  forall_kids(n,i)
    k += terminal_nodes_in_tree(n->kids[i]);
  return k;
}



/*
 * Two components are joined by choosing a random terminal node in
 * each and then identifying the two nodes. Note that at this stage
 * the two components are guaranteed to contain terminal nodes, so
 * the operation of choosing a node cannot fail.
 */

void join_components(tree feeder[], int ca, int cb)
{
  node na, nb;
  int i;

  na = random_node(feeder,ca);
  nb = random_node(feeder,cb);
  join_nodes(feeder,ca,cb,na,nb);
}



/*
 * Let na and nb be terminal nodes in components ca and cb respectively.
 * To join the components, join the nodes. Do this by giving them the
 * same name, copying the details of na into nb.
 *
 * From the nb perspective, the line into na is a line out, and vice
 * versa. Therefore the "reversed" status of the two nodes will be
 * opposite.
 *
 * Once the nodes have been joined, they no longer count as terminal,
 * although of course they are still open.
 *
 * When the nodes have been joined, the component cb must be re-labelled
 * as ca. It is given that ca is a smaller integer than cb. All the
 * component names above cb must then be decremented so that their
 * numbers still run consecutively.
 */

void join_nodes(tree feeder[], int ca, int cb, node na, node nb)
{
  int i;

  nb->id = na->id;
  strcpy(nb->name,na->name);
  nb->reversed = !(na->reversed);
  na->terminal = nb->terminal = false;

  if (ca != cb) {
    forall_trees(feeder,i) {
      if (feeder[i]->component == cb)
	feeder[i]->component = ca;
      else if (feeder[i]->component > cb)
	feeder[i]->component--;
    }
  }
}


/*
 * Generate a random number x and return the x-th terminal node in
 * component number c.
 */

node random_node(tree feeder[], int c)
{
  return get_terminal_node(feeder,c,
      (lrand48() % terminal_nodes_in_component(feeder,c))+1);
}



/*
 * Given that there are at least r terminal nodes in component c,
 * find and return the r-th one.
 *
 * The abort condition is not supposed to happen: it is there just as
 * a sanity check and for debugging purposes.
 */

node get_terminal_node(tree feeder[], int c, int r)
{
  int i;
  int onit;

  forall_trees(feeder,i)
    if (feeder[i]->component == c) {
      onit = terminal_nodes_in_tree(feeder[i]->root);
      if (onit < r) r -= onit;
      else return get_terminal_in_tree(feeder[i]->root,r);
    }
  rabort("Bad terminal node count");
  return 0;
}



/*
 * Given that there are at least r terminal nodes in the tree, find
 * and return the r-th one.
 */

node get_terminal_in_tree(node n, int r)
{
  node rn;
  int i;
  int onit;

  if (n->terminal) {
    if (r == 1) return n;
    else return 0;
  }
  forall_kids(n,i) {
    rn = get_terminal_in_tree(n->kids[i],r);
    if (rn)
      return rn;
    r -= terminal_nodes_in_tree(n->kids[i]);
  }
  return 0;
}



/*
 * After the feeders have ben joined into one connected network, there
 * may still be terminal nodes. Each of these may be treated in either of
 * two ways: it may be earthed by giving it a line out going to earth
 * (represented by side 2 of the dummy circuit breaker cb 0) or it may be
 * identified with another terminal node, which may be in the same feeder
 * or a different one. If it is identified with another node, this creates
 * a loop in the network.
 *
 * The parameter C is the probability of connection rather than earthing.
 *
 * Note that at this point there is only one component of the graph.
 */

void no_loose_ends(tree feeder[], double C, int next_id[])
{
  int tnic;
  node na, nb;

  tnic = terminal_nodes_in_component(feeder,0);
  while (tnic) {
    na = random_node(feeder,0);
    na->terminal = false;
    tnic--;
    if (tnic && drand48() < C) {
      nb = random_node(feeder,0);
      tnic--;
      if (na->id < nb->id) join_nodes(feeder,0,0,na,nb);
      else join_nodes(feeder,0,0,nb,na);
    }
    else {
      na->line_out = next_id[line]++;
      na->closed = true;
    }
  }
}


/*
 * ---------------------------------------------------------------------
 * The remaining functions are related to printing out the network
 * after it has been generated. This is not completely trivial because
 * devices and lines may have to be renamed in order to get the numbers
 * to run consecutively from 1.
 * ---------------------------------------------------------------------
 */

void print_network(tree feeder[], int next_id[], boolean opens,
                   double F, int Z)
{
  int i;
  int sdcount=0;
  int linecount=0;
  boolean *printed_opens;
  boolean faults[SDMAX];

  for (i=0; i<SDMAX; i++) faults[i] = false;
  rename_everything(feeder,next_id);

  forall_trees(feeder,i) {
    sdcount = max_sd_id(feeder[i]->root,sdcount);
    linecount += count_lines(feeder[i]->root);
  }

  printf("CB %d 0 L %d SD %d 0\n", i, linecount, sdcount);

  forall_trees(feeder,i)
    print_all_lines(feeder[i]->root);

  if (opens) {
    printed_opens = (boolean*) malloc(sizeof(boolean) * next_id[sd]);
    if (!printed_opens) rabort("Could not allocate memory");
    for (i=0; i<next_id[sd]; i++) printed_opens[i] = false;
    forall_trees(feeder,i)
      print_opens(feeder[i]->root,printed_opens);
    if (printed_opens) free(printed_opens);
  }

  if (F) {
    srand48(Z);
    find_faulty(linecount,faults,F);
    print_faulty(faults,linecount);
  }
}



/*
 * Go through all the nodes recursively. For each one, if it has not
 * been visited before, print a line saying whether it is open or
 * closed. The array "printed_opens" is just book-keeping, so that we
 * don't print the details for the same node twice.
 *
 * It is assumed that every node is either a circuit breaker (cd) or
 * a switching device (sd). If the code is changed later to allow
 * other sorts of device, this function will have to be rewritten.
 */

void print_opens(node n, boolean printed_opens[])
{
  int i;

  if (!n) return;
  if (n->sort==cb || !printed_opens[n->id]) {
    if (n->closed) printf("%s closed\n",n->name);
    else printf("%s open\n",n->name);
    if (n->sort==sd) printed_opens[n->id] = true;
  }
  forall_kids(n,i)
    print_opens(n->kids[i],printed_opens);
}



/*
 * When the feeder trees were created, the switching devices and lines
 * were numbered from 1 to some integers s and l respectively. During
 * the process of looping the loose ends, however, some of these
 * numbers may have been deleted from the list whenever two switching
 * devices were identified. The following functions renumber devices
 * and lines so that the numbers are again consecutive.
 */

void rename_everything(tree feeder[], int next_id[])
{
  int *sdindex;
  int *lineindex;
  int i;

  sdindex = set_sd_index(next_id[sd],feeder);
  lineindex = set_line_index(next_id[line],feeder);

  forall_trees(feeder,i)
    change_names(feeder[i]->root,sdindex,lineindex);

  if (sdindex) free(sdindex);
  if (lineindex) free(lineindex);
}



/*
 * Allocate an array of the appropriate length to form an index between
 * the old names of switching devices and their new names. Record
 * which numbers are now used as IDs of switching devices, and write
 * the index accordingly:
 *   sdindex[old_number_of_device] = new_number_of_device.
 * Returns a pointer to the index. Called from rename_everything.
 */

int *set_sd_index(int maxsd, tree feeder[])
{
  boolean *used;
  int i,j;
  int *sdindex;

  sdindex = (int*) malloc(sizeof(int)*maxsd);
  if (!sdindex) rabort("Could not allocate memory");
  for (i=0; i<maxsd; i++) sdindex[i] = 0;

  used = (boolean*) malloc(sizeof(boolean) * maxsd);
  if (!used) rabort("Could not allocate memory");
  for (i=0; i<maxsd; i++) used[i] = false;

  forall_trees(feeder,i)
    set_sd_used(used,feeder[i]->root);
  for (i=0,j=0; i<maxsd; i++)
    if (used[i])
      sdindex[i] = ++j;

  if (used) free(used);

  return sdindex;
}



/*
 * Recursively traverse the nodes recording which ID numbers are used.
 */

void set_sd_used(boolean used[], node n)
{
  int i;

  if (!n) return;
  if (n->sort == sd) used[n->id] = true;
  forall_kids(n,i)
    set_sd_used(used,n->kids[i]);
}



/*
 * Much the same as set_sd_index but for line IDs instead of device IDs.
 */

int *set_line_index(int maxline, tree feeder[])
{
  boolean *used;
  int i,j;
  int *lineindex;

  lineindex = (int*) malloc(sizeof(int)*maxline);
  if (!lineindex) rabort("Could not allocate memory for line index");
  for (i=0; i<maxline; i++) lineindex[i] = 0;

  used = (boolean*) malloc(sizeof(boolean) * maxline);
  if (!used) rabort("Could not allocate memory for line index");
  for (i=0; i<maxline; i++) used[i] = false;

  forall_trees(feeder,i)
    set_line_used(used,feeder[i]->root);
  for (i=1,j=0; i<maxline; i++)
    if (used[i])
      lineindex[i] = ++j;

  if (used) free(used);

  return lineindex;
}



/*
 * Again, the same as set_sd_used but for lines.
 */

void set_line_used(boolean used[], node n)
{
  int i;

  if (!n) return;
  used[n->line_out] = true;
  forall_kids(n,i)
    set_line_used(used,n->kids[i]);
}



/*
 * Use the indexes to change the names of switching devices and lines
 * so as to make them run sequentially from 1. This needs to visit
 * every node, so it is recursive.
 */

void change_names(node n, int sdindex[], int lineindex[])
{
  int i;

  if (!n) return;

  if (n->sort == sd && n->id != sdindex[n->id]) {
    n->id = sdindex[n->id];
    sprintf(n->name, "sd %d", n->id);
  }

  if (n->line_out != lineindex[n->line_out])
    n->line_out = lineindex[n->line_out];

  forall_kids(n,i)
    change_names(n->kids[i],sdindex,lineindex);
}




/*
 * This is the main part of the output routine. For each line, for each
 * pair of device sides incident on that line, print a record of the
 * connection.
 *
 * There are three cases:
 *   n is joined to each of its kids
 *   each pair of its kids are joined together
 *   if n is earthed, it is joined to side 2 of the dummy cb 0.
 */

void print_all_lines(node n)
{
  int i,j;

  if (!n || !(n->line_out)) return;

  forall_kids(n,i) {
    print_one_line(n,true,n->line_out,n->kids[i]);
    forall_kids(n,j) if (j > i)
      print_one_line(n->kids[i],false,n->line_out,n->kids[j]);
  }

  forall_kids(n,i)
    print_all_lines(n->kids[i]);

  if (!i)
    printf("%s %d l %d cb 0 2\n",
	   n->name, (n->reversed? 1: 2), n->line_out);
}



/*
 * Nodes ni (node in) and no (node out) are incident on line x. Print a
 * record of that fact.
 *
 * If ni is upstream from no (i.e. ni is the parent) and neither is
 * reversed, the line joins side 2 of ni to side 1 of no. On the other
 * hand, if the two nodes are siblings and not reversed, the line joins
 * their two side 1s. If either is reversed, the side incident on the
 * line is opposite from the "natural" one.
 *
 * For no especially good reason, only side 2 of a circuit breaker is
 * allowed to have a line coming out of it.
 */

void print_one_line(node ni, boolean parent, int x, node no)
{
  printf("%s %d l %d %s %d\n",
	 ni->name,
	 (ni->sort==cb? 2: (ni->reversed==parent? 1: 2)),
	 x,
	 no->name,
	 (no->sort==cb? 2: (no->reversed? 2: 1)));
}



/*
 * Find the maximum ID of any node in the (sub)tree starting at n, or
 * sofar (the maximum found so far) if sofar is bigger than the local
 * maximum. This is used as a way of calculating how many switching
 * devices there are.
 */

int max_sd_id(node n, int sofar)
{
  int i;

  if (n) {
    if (n->id > sofar) sofar = n->id;
    forall_kids(n,i)
      sofar = max_sd_id(n->kids[i],sofar);
  }
  return sofar;
}



/*
 * This is a more straightforward recursive function to count the lines
 * downstream from node n. These are the line out from n (if any) plus
 * the lines downstream from its children (if any).
 */

int count_lines(node n)
{
  int i, k;

  if (!n || !(n->line_out)) return 0;

  k = 0;
  forall_kids(n,i)
    k += count_lines(n->kids[i]);

  return k+1;
}




/*
 * No comment
 */

void print_failure_message()
{
  printf("Failure: could not join the feeders.\n\n");
}




/*
 * Don't proceed with the computation if the trees are likely to go
 * infinite. At depth k of the tree, each node has 0 children with
 * probability T, 1 child with probability (1-T)(1-B), 2 children
 * with probability (1-T)B(1-X), 3 with probability (1-T)BX(1-X),
 * 4 with probability (1-T)B[X^2](1-X) and so forth. In general, the
 * average number of children is
 *   1+B(Sigma_{i>=0} [X^i]) - T(1+B(Sigma_{i>=0} [X^i]))
 * Recalling that the sum of non-negative powers of X (for X between
 * 0 and 1) is equal to 1/(1-X), this expression simplifies to
 *   1 + B(1/(1-X)) - T(1 + B(1/(1-X)))
 * Evidently, if this is greater than 1, the number of nodes is
 * increasing with k and will go to infinity in the limit. If it is
 * less than 1 the number of nodes at depth k is decreasing with k
 * and goes to zero in the limit.
 * At any rate, for the trees to be (probably) finite, we require
 * B(1/(1-X)) - T(1 + B(1/(1-X))) < 0 which is to say
 *
 *           B
 *   T > ---------
 *       1 - X + B
 *
 * This is the sanity condition on T, B and X which the generator
 * refuses to violate.
 */

boolean probably_infinite(double T, double B, double X)
{
  if (X == 1.0 && B > 0.0)
    rabort("X must be less than 1, or the trees will be infinite.");

  if (T <= B / (1.0 - X + B)) {
    printf("\nB = %0.2f, X = %0.2f\n",B,X);
    printf("B/(1-X+B) = %0.4f\n",
	   (B / (1.0 - X + B)));
    printf("T = %0.2f\n\n",T);
  }
  return (T <= B / (1.0 - X + B));
}



void find_faulty(int linecount, boolean faulty[], double F)
{
  int x,y,z;
  int r;
  int number_faults;

  number_faults = f2d(linecount * F);
  if (!number_faults) return;

  for (z = 0; z<number_faults; z++) {
    r = lrand48() % (linecount-z);
    for (x=1,y=0; x<=linecount; x++) {
      if (!faulty[x]) {
        if (y==r) {
          faulty[x] = true;
          break;
        }
        else y++;
      }
    }
  }
}





void print_faulty(boolean faults[], int linecount)
{
  int x;

  for (x=1; x<=linecount; x++)
    if (faults[x])
      printf("l %d faulty\n",x);
}





/*
* This just puts the nodes at or below n into an array in id order
*/

void list_node(node n, node thenode[])
{
  int x;

  thenode[n->id] = n;
  forall_kids(n,x)
    list_node(n->kids[x],thenode);
}



/*
* Fuss-free conversion of double to int with rounding
*/

int f2d(double f)
{
  char s[20];
  int x;

  sprintf(s,"%0.0f",f);
  sscanf(s,"%d",&x);
  return x;
}



/*
 * Exit on detection of an error condition.
 */

void rabort(char * s)
{
  printf("Panic!!\n%s\n",s);
  exit(1);
}




/*
 * At the end of the computation, gracefully free all the memory used
 * for trees and their nodes.
 */

void free_memory(tree f[])
{
  int i;

  for (i=0; i<MAX_FEEDERS; i++)
    if (f[i]) {
      free_all_nodes(f[i]->root);
      free(f[i]);
    }
}



/*
 * Free the memory used for node n and all nodes downstream from it.
 */

void free_all_nodes(node n)
{
  int i;

  if (n) {
    for (i=0; i<MAX_KIDS; i++)
      free_all_nodes(n->kids[i]);
    free(n);
  }
}


void print_help()
{
  printf("SYNOPSIS\n");
  printf("  randomnet [-b -c -l -n -o -r -s -t -u -x]\n");
  printf("\n");
  printf("DESCRIPTION\n");
  printf("  Generate randomised power distribution networks consisting of\n");
  printf("  circuit breakers (cb) and switching devices (sd) connected by\n");
  printf("  lines. Output format is\n");
  printf("\n");
  printf("    CB <c> 0 L <l> SD <s> 0\n");
  printf("    <line definition>\n");
  printf("    <line definition>\n");
  printf("    ...\n");
  printf("\n");
  printf("  where c is the number of circuit breakers, l the number of\n");
  printf("  lines and s the number of switching devices. Each line\n");
  printf("  definition is of the form\n");
  printf("\n");
  printf("    <device type> <did> <s> l <lid> <device type> <did> <s>\n");
  printf("\n");
  printf("  where each device type is either \"cb\" (for circuit breaker) or\n");
  printf("  \"sd\" (for switching device), <did> is its (integer) id number,\n");
  printf("  <s> is a side (1 or 2) and <lid> is the id of the line. The\n");
  printf("  reading is that line number <lid> connects the specified sides\n");
  printf("  of the two devices. For example,\n");
  printf("\n");
  printf("    cb 3 2 l 8 sd 12 2\n");
  printf("\n");
  printf("  means that side 2 of circuit breaker #3 and side 2 of switching\n");
  printf("  device #12 are connected by line #8.\n");
  printf("\n");
  printf("OPTIONS\n");
  printf("  There are numerous options, mostly to control the average size\n");
  printf("  and topology of the generated networks.\n");
  printf("\n");
  printf("    -b <float>  The probability that a line branches. Default 0.4.\n");
  printf("\n");
  printf("    -c <float>  The probability that an open switch not otherwise\n");
  printf("                used to connect feeders together is connected to\n");
  printf("                other devices on both sides, thus forming a loop\n");
  printf("                in the network rather than a line to earth. Set to\n");
  printf("                0.0 for a loop-free network, 1.0 for as many loops\n");
  printf("                as possible. Default 0.5.\n");
  printf("\n");
  printf("    -f <float>  The proportion of lines (selected at random) to be\n");
  printf("                labelled as \"faulty\". If this option is given, a\n");
  printf("                second random number seed may be specified with\n");
  printf("                the -z option.\n");
  printf("\n");
  printf("    -h          Help. Prints this text.\n");
  printf("\n");
  printf("    -l <int>    Lower bound on the number of switching devices\n");
  printf("                in each feeder (excluding the circuit breaker at\n");
  printf("                the root). Used to eliminate trivial feeders such\n");
  printf("                as those consisting only of a circuit breaker, or\n");
  printf("                only a circuit breaker and one downstream switch.\n");
  printf("                Default 2.\n");
  printf("\n");
  printf("    -n <int>    The number of circuit breakers. Default 3.\n");
  printf("\n");
  printf("    -o          Print a list of the devices saying which ones are\n");
  printf("                open and which are closed in the initial state of\n");
  printf("                the network. If this option is not included, only\n");
  printf("                the configuration of lines is printed.\n");
  printf("\n");
  printf("    -r <int>    Make <int> attempts to connect the feeders into a\n");
  printf("                single network (a connected graph). Print a\n");
  printf("                warning if no such graph is produced in the\n");
  printf("                stated number of iterations. Default 100.\n");
  printf("\n");
  printf("    -s <int>    Seed for the random number generator. Default 1.\n");
  printf("\n");
  printf("    -t <float>  Probability that a switching device is terminal in\n");
  printf("                its feeder. It is then open, unless it has a line\n");
  printf("                out to earth. All non-terminal devices are closed.\n");
  printf("                Default 0.45.\n");
  printf("\n");
  printf("    -u <int>    Upper bound on the number of switching devices on\n");
  printf("                a feeder. The random network generator can produce\n");
  printf("                very large trees, and though this is rather rare,\n");
  printf("                it can be annoying. If the \"upper bound\" is set to\n");
  printf("                0, there is no limit and trees of any size may be\n");
  printf("                generated. If it is positive, only feeders with\n");
  printf("                numbers of switching devices in the range\n");
  printf("                (lower-bound ... upper-bound) will be generated.\n");
  printf("                Default 10.\n");
  printf("\n");
  printf("    -x <float>  Probability that a line branches at least B+1 ways\n");
  printf("                given that it branches at least B ways (for B > 1).\n");
  printf("                Branching more than two ways is quite rare in real\n");
  printf("                networks, so this should be a lower value than that\n");
  printf("                given by the -b option. Default 0.15.\n");
  printf("\n");
  printf("    -z <int>    Another random number seed. This one is for\n");
  printf("                selecting which lines are faulty. If option -f is\n");
  printf("                given, the default seed is 1. If -f is not given,\n");
  printf("                -z makes no sense and will be silently ignored.\n");
  printf("\n");
  printf("NOTES:\n");
  printf("  The generator will not accept values for T, B and X which would\n");
  printf("  lead to the expectation that the feeder trees would be infinite.\n");
  printf("  The threshold for this occurs where the expected number of nodes\n");
  printf("  at a given depth in the tree neither increases nor decreases\n");
  printf("  with the depth. Higher values for T lead to finite trees with\n");
  printf("  probability 1; lower values for T may lead to infinite ones. The\n");
  printf("  required inequation for an expectation of finite trees is\n");
  printf("\n");
  printf("           B\n");
  printf("   T > ---------\n");
  printf("       1 - X + B\n");
  printf("\n");
}
