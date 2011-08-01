/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
 *
 * Authorship of this source code was supported by EC FP7-IST grant
 * 215181-CogX.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dear CogX team member :: Please email (charles.gretton@gmail.com)
 * if you make a change to this and commit that change to SVN. In that
 * email, can you please attach the source files you changed as they
 * appeared before you changed them (i.e., fresh out of SVN), and the
 * diff files (*). Alternatively, you could not commit your changes,
 * but rather post me a patch (**) which I will test and then commit
 * if it does not cause any problems under testing.
 *
 * (*) see http://www.gnu.org/software/diffutils/diffutils.html --
 * GNU-09/2009
 *
 * (**) see http://savannah.gnu.org/projects/patch -- GNU-09/2009
 * 
 */

#ifndef BOOLEAN_STATE_HH
#define BOOLEAN_STATE_HH

#include "state_basics.hh"

namespace Planning
{
    class Boolean_State
    {
    public:
        friend std::ostream& std::operator<<(std::ostream&, const Boolean_State&);
        
	/* Initially everything is FALSE (all bits in \member{data}
	 * are FALSE). \argument{size} is the number of propositions
	 * that this state is required to keep the truth value of.*/
	Boolean_State(uint number_of_atoms = 0);

        uint get__number_of_atoms() const;
        
	/* Copy construction clones \member{data}.*/
	Boolean_State(const Boolean_State&);
        
// 	/* Copy construction makes state with \member{data} cloning
//          * \argument{vector}.*/
// 	explicit Boolean_State(const std::vector<ELEM_TYPE>&);
        
	/* This clones the same elements as the copy constructor.*/
	Boolean_State& operator=(const Boolean_State&);
	
	/* Comparison is based on the size of \member{data} first, and
	 * then on the actual elements in \memeber{data}.*/
	bool operator==(const Boolean_State& state) const;
	bool operator<(const Boolean_State& state) const;
        
	/* Change individual (at index \argument{uint}) bits of the
	 * state representation (\member{data}).*/
	void flip_on(uint);/*to \true*/
	void flip_off(uint);/*to \false*/
	void flip(uint);/*to \opposite*/
	void flip_on(uint index, std::vector<ELEM_TYPE>& tmpData) const;/*to \true*/
	void flip_off(uint index, std::vector<ELEM_TYPE>& tmpData) const;/*to \false*/
        
	/* Is proposition \argument{uint} \true?*/
	bool is_true(uint) const;
	
	/* Is proposition \argument{uint} \false?*/
	bool is_false(uint in) const;

	/*Hashing is either via \library{boost} or \library{TR1}. The
	 * hash is generated from \member{data}.*/
	std::size_t hash_value() const;

	/* Same as \member{hash_value()} but allows to pass a reference
	 * to an external std::vector of data elements.*/
	std::size_t hash_value(const std::vector<ELEM_TYPE>&);

        /*(see \member{data.size()})*/
        uint size() const;

    protected:
	/* \macro{SIZE_ELEM} bits per element in this std::vector, each bit
	 * represents the truth value of a proposition.*/
	std::vector<ELEM_TYPE> data;
    private:
	/* All bits are 1.*/
	static const ELEM_TYPE big = -1;

        /*Number of atoms whose truth values are stored in this state.*/
        uint number_of_atoms;
    };
}


#endif

/* Capable generous men do not create victims, the nurture victims. 
 *
 * -- Julian Assange, being interviewed by Chris Anderson in a TED
 * session titled "Why the world needs WikiLeaks", July 26 2010.
 */

