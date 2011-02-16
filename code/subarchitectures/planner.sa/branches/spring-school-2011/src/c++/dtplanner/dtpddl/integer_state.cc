


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

#include "integer_state.hh"

using namespace Planning;



Integer_State::Integer_State(uint size)
    :data(size)
{
    for(auto i =0; i < size; i++){
        assert(i < data.size());
        data[i] = 0;
    }
    
}

        
Integer_State::Integer_State(const Integer_State& integer_State)
    :data(integer_State.data)
{
}

Integer_State::Integer_State(const std::vector<int>& input_data)
    :data(input_data)
{
}

        
Integer_State& Integer_State::operator=(const Integer_State& integer_State)
{
    this->data = integer_State.data;

    return *this;
}

	
bool Integer_State::operator==(const Integer_State& state) const
{
    return this->data == state.data;
}

bool Integer_State::operator<(const Integer_State& state) const
{
    if(this->data.size() < state.data.size()){
        return true;
    } else if (this->data.size() == state.data.size()) {
        if(this->data < state.data){
            return true;
        }
    }
    
    
    return false;
}
        
void Integer_State::write(uint index, int value)
{
    assert(index < data.size());
    data[index] = value;
}
       
void Integer_State::increment(uint index)
{
    assert(index < data.size());
    data[index]++;
}

void Integer_State::decrement(uint index)
{
    assert(index < data.size());
//     assert(data[index] > 0);
    data[index]--;
}

int Integer_State::read(uint index) const
{
    assert(index < data.size());

    return data[index];
}

bool Integer_State::is_equal(uint index, int value) const
{
    assert(index < data.size());

    return (value == data[index]);
}

        
std::size_t Integer_State::hash_value() const
{
    return boost::hash_range(data.begin(), data.end());
}


std::size_t Integer_State::hash_value(const std::vector<int>& input)
{
    return boost::hash_range(input.begin(), input.end());
}

uint Integer_State::size() const
{
    return data.size();
}

std::size_t std::hash_value(const Integer_State& state)
{
    return state.hash_value();
}
    
std::ostream& std::operator<<(std::ostream& o, const Integer_State& state)
{
    /*Print out the fluent propositions first.*/
    for(int i = 0; i < state.data.size(); i++){
        o<<i<<" := "<<state.data[i]<<"; ";
    }
    
    return o;
}
