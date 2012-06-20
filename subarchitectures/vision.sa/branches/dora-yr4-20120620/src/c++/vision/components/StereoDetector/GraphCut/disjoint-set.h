/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#ifndef DISJOINT_SET
#define DISJOINT_SET

// disjoint-set forests using union-by-rank and path compression (sort of).

namespace Z
{
  
typedef struct {
  int rank;         // rank of element (starts with 0)
  int p;            // ??? TODO Parent element???
  int size;         // ??? TODO How many elements are in a set
} uni_elt;


class universe {
public:
  universe(int elements);
  ~universe();
  int find(int x);  
  void join(int x, int y);
  int size(int x) const { return elts[x].size; }
  int num_sets() const { return num; }
  void printAll();

private:
  uni_elt *elts;
  int num;
};


universe::universe(int elements) 
{
  elts = new uni_elt[elements];
  num = elements;
  for (int i = 0; i < elements; i++) {
    elts[i].rank = 0;
    elts[i].size = 1;
    elts[i].p = i;
  }
}
  
universe::~universe() {
  delete [] elts;
}

int universe::find(int x) {
  int y = x;
  while (y != elts[y].p)
  {
// printf("  y: %u - elts[y].p: %u\n", y, elts[y].p);
    y = elts[y].p;
  }
  elts[x].p = y;
  return y;
}

void universe::join(int x, int y) 
{
// printf("  ==> universe join: %u-%u\n", x, y);  
  if (elts[x].rank > elts[y].rank) {
    elts[y].p = x;
    elts[x].size += elts[y].size;
  } else {
    elts[x].p = y;
    elts[y].size += elts[x].size;
    if (elts[x].rank == elts[y].rank)
      elts[y].rank++;
  }
  num--;
}

void universe::printAll() 
{
  for(int i=0; i<num; i++)
  {
    printf("  %u: elts: rank: %u - p: %u - size: %u\n", i, elts[i].rank, elts[i].p, elts[i].size);
  }
}

}

#endif
