
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

#ifndef INTEGER_STATE_HH
#define INTEGER_STATE_HH

#include "state_basics.hh"

namespace Planning
{
    class Integer_State
    {
    public:
        friend std::ostream& std::operator<<(std::ostream&, const Integer_State&);

	/* Initially everything is true (all bits in \member{data} are
	 * true). \argument{size} is the number of propositions that
	 * this state is required to keep the truth value of. WARNING
	 * \argument{size} is not necessarily equal to
	 * \member{data.size()}.*/
	Integer_State(uint size = 0);
        
	/* Copy construction clones \member{data}.*/
	Integer_State(const Integer_State&);
        
	/* Copy construction makes state with \member{data} cloning
         * \argument{vector}.*/
	explicit Integer_State(const std::vector<int>&);

	/* This clones the same elements as the copy constructor.*/
	Integer_State& operator=(const Integer_State&);
	
	/* Comparison is based on the size of \member{data} first, and
	 * then on the actual elements in \memeber{data}.*/
	bool operator==(const Integer_State& state) const;
	bool operator<(const Integer_State& state) const;
        
        void write(uint, int);
        void increment(uint);
        void decrement(uint);

        /*What is the entry at index \argument{uint}.*/
        int read(uint) const;

        /*Is the entry at index \argument{uint} equal to \argument{int}.*/
        bool is_equal(uint, int) const;
        
	/* Hashing is either via \library{boost} or \library{TR1}. The
	 * hash is generated from \member{data}.*/
	std::size_t hash_value() const;

	/* Same as \member{hash_value()} but allows to pass a reference
	 * to an external std::vector of data elements.*/
	std::size_t hash_value(const std::vector<int>&);

        /*(see \member{data.size()})*/
        uint size() const;
    protected:
	/* \macro{SIZE_ELEM} bits per element in this std::vector, each bit
	 * represents the truth value of a proposition.*/
	std::vector<int> data;
    };
}

#endif

/* Precious bodily fluids.
 *
 * -- Major General Albert Stubblebine III, infamous proponent of
 * psychic warfare.
 *
 */
