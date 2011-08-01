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

#include "boolean_state.hh"

using namespace Planning;

uint Boolean_State::size() const {return data.size();}

bool Boolean_State::is_false(uint in) const {return !is_true(in);}

/*Change individual (at index \argument{uint}) bits of the
 * state representation (\member{data}).*/
void Boolean_State::flip_on(uint index)
{
    uint remainder = REMAINDER(index);
    uint bitVecNum = FLOOR(index);

    ELEM_TYPE tmp = 1;
    tmp = tmp<<remainder;
    data[bitVecNum] |= tmp;
    
    /*Now it's true, some actions may be executable.*/
}

    
void Boolean_State::flip_on(uint index, std::vector<ELEM_TYPE>& tmpData) const
{
    uint remainder = REMAINDER(index);
    uint bitVecNum = FLOOR(index);

    ELEM_TYPE tmp = 1;
    tmp = tmp<<remainder;
    tmpData[bitVecNum] |= tmp;
    
    /*Now it's true, some actions may be executable.*/
}

    
void Boolean_State::flip_off(uint index)
{
    uint remainder = REMAINDER(index);
    uint bitVecNum = FLOOR(index);

    ELEM_TYPE tmp = 1;
    tmp = tmp<<remainder;
    tmp = tmp^big;
    data[bitVecNum] &= tmp;
}

    
void Boolean_State::flip_off(uint index, std::vector<ELEM_TYPE>& tmpData) const 
{
    uint remainder = REMAINDER(index);
    uint bitVecNum = FLOOR(index);

    ELEM_TYPE tmp = 1;
    tmp = tmp<<remainder;
    tmp = tmp^big;
    tmpData[bitVecNum] &= tmp;
}

    
void Boolean_State::flip(uint index)
{
    uint remainder = REMAINDER(index);
    uint bitVecNum = FLOOR(index);

    ELEM_TYPE tmp = 1;
    tmp = tmp<<remainder;
    
    if(data[bitVecNum] & tmp){
        tmp = tmp^big;
        data[bitVecNum] &= tmp;
    } else {
        data[bitVecNum] |= tmp;

        /*Now it's true, some actions may be executable.*/
    }
}
    
bool Boolean_State::is_true(uint index) const
{
    uint remainder = REMAINDER(index);
    uint bitVecNum = FLOOR(index);

    ELEM_TYPE tmp = 1;
    tmp = tmp<<remainder;
    
    return (data[bitVecNum] & tmp)?true:false;
}



uint Boolean_State::get__number_of_atoms() const
{
    return number_of_atoms;
}


Boolean_State::Boolean_State(uint number_of_atoms)
    :number_of_atoms(number_of_atoms)
{
    
    INTERACTIVE_VERBOSER(true, 7000, "Made an Boolean state with  :: "
                         <<this->number_of_atoms
                         <<" propositions.");
    
    data = std::vector<ELEM_TYPE>(CEIL(number_of_atoms));
    
    /*All propositions are initially false.*/
    for(uint i = 0; i < data.size(); i++){
        data[i] = 0;
    }
    
    for(uint i = number_of_atoms + 1; i < SIZE_ELEM * data.size(); i++){
        flip_on(i);
    }    

#ifndef NDEBUG
    for(uint i = 0 ; i < number_of_atoms; i++){
        assert(is_false(i));
    }
#endif
}
    
Boolean_State::Boolean_State(const Boolean_State& state)
    :data(state.data),
     number_of_atoms(state.number_of_atoms)
     
{
}

    
// Boolean_State::Boolean_State(const std::vector<ELEM_TYPE>& tmpData)
//     :data(tmpData),
//      number_of_atoms(state.number_of_atoms)
// {
// }
    
size_t Boolean_State::hash_value() const
{
    return boost::hash_range(data.begin(), data.end());
}

    
size_t Boolean_State::hash_value(const std::vector<ELEM_TYPE>& dataRef)
{	
    return boost::hash_range(dataRef.begin(), dataRef.end());
}
    
Boolean_State& Boolean_State::operator=(const Boolean_State& state)
{
    this->data = state.data;
    this->number_of_atoms = state.number_of_atoms;

    return *this;
}

    
bool Boolean_State::operator<(const Boolean_State& state) const
{
    return (state.data.size() < this->data.size())?true:
        ((state.data.size() == this->data.size())?(state.data < this->data):false);
}

bool Boolean_State::operator==(const Boolean_State& state) const
{
    return state.data == this->data;
}
 

    


/*Function for STL and boost to access \member{hash_value} of
 * \argument{GroundAction}.*/
    
std::size_t std::hash_value(const Boolean_State& state)
{
    return state.hash_value();
}
    
std::ostream& std::operator<<(std::ostream& o, const Boolean_State& state)
{
    /*Print out the fluent propositions first.*/
    for(uint i = 0; i < state.get__number_of_atoms(); i++){
        if(state.is_true(i)){
            o<<"T";
        } else {
            o<<"F";
        }
    }

    return o;
}
