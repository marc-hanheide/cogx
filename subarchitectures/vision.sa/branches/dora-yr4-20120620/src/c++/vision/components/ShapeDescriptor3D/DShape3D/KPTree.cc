/**
 * $id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#include "KPTree.hh"

namespace P 
{ 

#define DP_MIN_CLUSTER_SIZE 10

/********************************* KPList ************************************/
//  Purpose:
//
//    KPList::~KPList is the annihilator for class KPList.
//
//  Author:
//
//    Olivier Devillers
//
KPList::~KPList ( void )
{ 
  KPList *p,*q;

  if (!next) return;

  for ( p = this, q = this->next; q ; ) 
  {
    p->next = NULL;
    delete p;
    p = q;
    q = q->next ;
  }
}







/********************************** KPNode *************************************/
//  Purpose:
//
//    KPNode::conflict is true if the point P is inside the closed circumdisk.
//
//  Author:
//
//    Olivier Devillers
//
//  Parameters:
//
//    Input, point *P, the point to be checked.
//
//    Output, ind KPNode::conflict, is true if the point is inside the closed circumdisk.
//
ind KPNode::conflict ( KDVertex *p )
{
  switch ( flag.is_infinite() )
  {
  case 4:
    return 0;
  case 3:
    return 1;
  case 2:
    return (Dot((p->p - vertices[0]->p), (vertices[1]->p + vertices[2]->p)) >= 0 );
  case 1:
    return ( ( flag.is_last_finite() )
      ?( (Cross((p->p - vertices[2]->p) , (vertices[2]->p - vertices[0]->p))) >= 0)
      :( (Cross((p->p - vertices[0]->p) , (vertices[0]->p - vertices[1]->p))) >= 0 ));

  case 0:
// 
//  compute the det 4*4 column: x,y,x**2+y**2,1 for p and vertices [0,1,2]
//
    double x,y;
    double x0,y0;
      double x1,y1;
      double x2,y2;
        double z1,z2;
        double alpha,beta,gamma;
        x  = p->X();
        y  = p->Y();
        x0 = vertices[0]->X();
        y0 = vertices[0]->Y();
        x1 = vertices[1]->X();
        y1 = vertices[1]->Y();
        x2 = vertices[2]->X();
        y2 = vertices[2]->Y();
        x1-=x0;
          y1-=y0;
          x2-=x0;
            y2-=y0;
            x-=x0;
              y-=y0;
        z1=(x1*x1)+(y1*y1);
          z2=(x2*x2)+(y2*y2);
        alpha=(y1*z2)-(z1*y2);
          beta=(x2*z1)-(x1*z2);
          gamma=(x1*y2)-(y1*x2);
        return ((alpha*x)+(beta*y)+gamma*((x*x)+(y*y))  <= 0 );
  }
  return 0;
}

//  Purpose:
//
//    KPNode::KPNode ( void ) is a creation routine for class KPNode.
//
//  Author:
//
//    Olivier Devillers
//
KPNode::KPNode ( void )
{
  P::Vector2 mot = P::Vector2(0.,0.);
  vertices[0] = new KDVertex (1.0, 0.0);
  vertices[1] = new KDVertex (-0.5, 0.8660254 );
  vertices[2] = new KDVertex (-0.5, -0.8660254);

  flag.infinite ( 3 );
  nb = 0;
  sons = NULL;
}

//  Purpose:
//
//    KPNode::KPNode ( KPNode* root, ind i ) is a creation routine for class KPNode.
//
//  Author:
//
//    Olivier Devillers
//
KPNode::KPNode ( KPNode *root, ind i )
{
  vertices[0] = root->vertices[0] ;
  vertices[1] = root->vertices[1] ;
  vertices[2] = root->vertices[2] ;

  flag.infinite ( 4 );
  nb = 0;
  ncl=0;
  sons = NULL;
  neighbors[i] = root ;
  root->neighbors[i] = this ;
}

//  Purpose:
//
//    KPNode::KPNode ( KPNode* f, point *c, ind i ) is a creation routine for class KPNode.
//
//  Discussion:
//
//    The triangle is created in ccw order.
//    The circumdisk and flatness are not computed.
//
//  Author:
//
//    Olivier Devillers
//
KPNode::KPNode ( KPNode *f, KDVertex *c, ind i )
{
  switch ( f->flag.is_infinite() )
  {
  case 0:  flag.infinite ( 0 );
    break;
  case 1: if (f->flag.is_last_finite() ) flag.infinite ( (i==1) ? 0 : 1 );
      else                        flag.infinite ( (i==2) ? 0 : 1 );
      if ( flag.is_infinite() )
      {
        if ( f->flag.is_last_finite()  )
          { if ( i==0 ) flag.last_finite(); }
        else
          { if ( i==1 ) flag.last_finite(); }
      }
    break;
  case 2: flag.infinite ( (i==0) ? 2 : 1 );
        if (i==1) flag.last_finite();
    break;
  case 3:  flag.infinite ( 2 );
    break;
  }
  nb = 0;
  sons = NULL;
  f->sons = new KPList(f->sons, this);
  f->neighbors[i]->sons = new KPList(f->neighbors[i]->sons, this);
  f->neighbors[i]->neighbors[ f->neighbors[i]->neighbor_ind(f) ] = this ;
  vertices[0] = c;
  neighbors[0] = f->neighbors[i];
  switch (i) {
  case 0:
    vertices[1] = f->vertices[1];
    vertices[2] = f->vertices[2];
    break;
  case 1:
    vertices[1] = f->vertices[2];
    vertices[2] = f->vertices[0];
    break;
  case 2:
    vertices[1] = f->vertices[0];
    vertices[2] = f->vertices[1];
    break;
  }
}

//  Purpose:
//
//    KPNode::find_conflict returns an alive node in conflict.
//
//  Author:
//
//    Olivier Devillers
//
KPNode* KPNode::find_conflict ( KDVertex *p )
{
  KPList* l;
  KPNode* n;

  if ( !conflict ( p ) )
  {
    return NULL;
  }

  if ( ! flag.is_dead() ) 
  {
    return this;
  }

  for ( l = sons; l; l = l->next )
  {
    if ( l->key->nb != nb )
    {
      l->key->nb = nb;
      n=l->key->find_conflict(p);
      if ( n ) 
      {
        return n;
      }
    }
  }

  return NULL;
}

/**
 * Get next unused node
 */
void KPNode::GetNextNode(KPNode* &node, int ncl)
{
  if (!flag.is_infinite())
  {
    if (vertices[0]->kp!=0 && vertices[1]->kp!=0 && vertices[2]->kp!=0)
    {
      if (TestNB(vertices[0],ncl) && TestNB(vertices[1],ncl) && TestNB(vertices[2],ncl))
      {
        node = this;
      }
    }
  }

  if (neighbors[0]->nb!=nb && node==0)
  {
    neighbors[0]->nb = nb;
    neighbors[0]->GetNextNode(node, ncl);
  }
  if (neighbors[1]->nb!=nb && node==0)
  {
    neighbors[1]->nb = nb;
    neighbors[1]->GetNextNode(node, ncl);
  }
  if (neighbors[2]->nb!=nb && node==0)
  {
    neighbors[2]->nb = nb;
    neighbors[2]->GetNextNode(node, ncl);
  }
}



/**
 * @brief Cluster on connected nodes
 */
void KPNode::Cluster(P::Array<KDVertex*> &out, int ncl)
{
  if (!flag.is_infinite())
  {
    if (TestNB(vertices[0],ncl) && TestNB(vertices[1],ncl) && TestNB(vertices[2],ncl))
    {
      if (vertices[0]->nb!=nb)
      {
        vertices[0]->nb=nb;
        out.PushBack(vertices[0]);
      }
      if (vertices[1]->nb!=nb)
      {
        vertices[1]->nb=nb;
        out.PushBack(vertices[1]);
      }
      if (vertices[2]->nb!=nb)
      {
        vertices[2]->nb=nb;
        out.PushBack(vertices[2]);
      }

      if (neighbors[0]->nb!=nb)
      {
        neighbors[0]->nb=nb;
        neighbors[0]->Cluster(out, ncl);
      }
      if (neighbors[1]->nb!=nb)
      {
        neighbors[1]->nb=nb;
        neighbors[1]->Cluster(out, ncl);
      }
      if (neighbors[2]->nb!=nb)
      {
        neighbors[2]->nb=nb;
        neighbors[2]->Cluster(out, ncl);
      }
    }    
  }
}

/**
 * set links to the other vertices
 */
void KPNode::SetVertexLinks()
{
  KPList* l;

  if ( flag.is_dead() )
  {
    for ( l=sons; l; l = l->next )
      if ( l->key->nb != nb )
      {
        l->key->nb = nb;
        l->key->SetVertexLinks();
      }
    return;
  }

  if (neighbors[0]->nb != nb)
        if ( ! flag.is_infinite() ) 
        {
          vertices[1]->links.PushBack(vertices[2]);
          vertices[2]->links.PushBack(vertices[1]);
        }

  if (neighbors[1]->nb != nb)
  {
        if ( ! flag.is_infinite() )  
        {
          vertices[2]->links.PushBack(vertices[0]);
        }
        else if ( ( flag.is_infinite() == 1) &&( flag.is_last_finite()))
        {
          vertices[2]->links.PushBack(vertices[0]);
          vertices[0]->links.PushBack(vertices[2]);
        }
  }

  if (neighbors[2]->nb != nb)
  {
        if ( ! flag.is_infinite() )
        {
          vertices[0]->links.PushBack(vertices[1]);
        }
        else if ( ( flag.is_infinite() == 1) &&(!flag.is_last_finite()))
        {
          vertices[0]->links.PushBack(vertices[1]);
          vertices[1]->links.PushBack(vertices[0]);
        }
  }
}







/**************************** KPTree *********************************/

/**
 * @brief Split a cluster if the triangles are disconnected
 * @param in Clusters of keypoints with similar motion
 * @param out Clusters of keypoints with similar motion which are connected by delaunay..
 */
void KPTree::GetGroups(Array<Array<KeypointDescriptor*> > &mpsGroups)
{
  KPNode *node;
  P::Array<KDVertex*> kdvs;
  int ncl=KDVertex::CL_DP;

  do{
    ncl++;
    node=0;
    root->nb++;
    root->GetNextNode(node, KDVertex::CL_NOT_SET);

    if (node!=0)
    {
      kdvs.Clear();
      root->nb++;
      node->nb=root->nb;
      node->Cluster(kdvs, KDVertex::CL_NOT_SET);             //connection triangle

      if(kdvs.Size() >= DP_MIN_CLUSTER_SIZE) 
      {
        mpsGroups.PushBack(Array<KeypointDescriptor*>());

        for (unsigned j=0; j<kdvs.Size(); j++)
          mpsGroups.Last().PushBack(kdvs[j]->kp);
      }

      for (unsigned j=0; j<kdvs.Size(); j++)
        SetNB(kdvs[j],ncl);
    }

  }while (node != 0);

}

/**
 * Set links to other vertices
 */
void KPTree::SetVertexLinks()
{
  root->nb = ++nb;
  root->SetVertexLinks();
}


//  Purpose:
//
//    KPTree::KPTree ( void ) initializes the Delaunay tree.
//
//  Author:
//
//    Olivier Devillers
//
KPTree::KPTree ( void )
{
  nb = 0;
  root = new KPNode();
  new KPNode(root, 0);
  new KPNode(root, 1);
  new KPNode(root, 2);
  root->neighbors[0]->neighbors[1] =  root->neighbors[1];
  root->neighbors[0]->neighbors[2] =  root->neighbors[2];
  root->neighbors[1]->neighbors[0] =  root->neighbors[0];
  root->neighbors[1]->neighbors[2] =  root->neighbors[2];
  root->neighbors[2]->neighbors[0] =  root->neighbors[0];
  root->neighbors[2]->neighbors[1] =  root->neighbors[1];
}

//  Purpose:
//
//    KPTree::~KPTree is the destructor for class KPTree.
//
//  Author:
//
//    Olivier Devillers
//
KPTree::~KPTree ( void )
{
  nb++;
}


//  Purpose:
//
//    KPTree::operator=+ is the insertion operator for class KPTree.
//
//  Author:
//
//    Olivier Devillers
//
KPTree &KPTree::operator += (KDVertex *p )
{
  KPNode* n;
  KPNode* created;
  KPNode* last;
  KPNode* first;
  KDVertex* q;
  KDVertex* r;
  ind i;

  root->nb = ++nb;

  if ( ! ( n = root->find_conflict(p) ) ) 
  {
    return *this ;
  }
  //
  // test if p is already inserted
  //
    for ( i=0; (int) i < 3- (int) n->flag.is_infinite(); i++ )
    if ( ( p->X()==n->vertices[i]->X() )&&( p->Y()==n->vertices[i]->Y() ) )
                                return *this;
  n->flag.kill();
    // we will turn cw around first vertex of n, till next triangle
    // is not in conflict
    q = n->vertices[0];
    while( n->neighbors[ i=n->cw_neighbor_ind(q) ]->conflict(p) )
                {n = n->neighbors[i];n->flag.kill();}

  first = last = new KPNode(n,p,i);
    // we will turn cw around r, till next triangle is not in conflict
    r = n->vertices[(((int) i)+2)%3];
    while( 1){
        i = n->cw_neighbor_ind(r);
        if (n->neighbors[i]->flag.is_dead()){
              n = n->neighbors[i]; continue;
        }
        if (n->neighbors[i]->conflict(p) ){
            n = n->neighbors[i];
            n->flag.kill();
            continue;
        }
        break;
    }
  // n is killed by p
  // n->neighbors[i] is not in conflict with p
  // r is vertex i+1 of n
  while ( 1 ) {

    created = new KPNode(n,p,i);
    created->neighbors[2]=last;
    last->neighbors[1]=created;
    last=created;
    r = n->vertices[(((int) i)+2)%3];   // r turn in n ccw
    if (r==q) break;
    // we will turn cw around r, till next triangle is not in conflict
    while( 1){
        i = n->cw_neighbor_ind(r);
        if (n->neighbors[i]->flag.is_dead()){
              n = n->neighbors[i]; continue;
        }
        if (n->neighbors[i]->conflict(p) ){
            n = n->neighbors[i];
            n->flag.kill();
            continue;
        }
        break;
    }
  }
  first->neighbors[2]=last;
  last->neighbors[1]=first;
  return *this;
}



}
