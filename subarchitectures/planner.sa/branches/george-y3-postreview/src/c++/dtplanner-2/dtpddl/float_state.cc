
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


#include "float_state.hh"

using namespace Planning;


Float_State::Float_State(uint size)
    :data(size)
{
    for(uint i =0; i < size; i++){
        
        QUERY_UNRECOVERABLE_ERROR(i >= data.size(),
                                  "Failed trying to read float at index :: "
                                  <<i<<std::endl
                                  <<" as that entry is out of bounds.");
        
        assert(i < data.size());
        data[i] = 0.0;
    }
}

        
Float_State::Float_State(const Float_State& float_State)
    :data(float_State.data)
{
}

Float_State::Float_State(const std::vector<double>& input_data)
    :data(input_data)
{
}

        
Float_State& Float_State::operator=(const Float_State& float_State)
{
    this->data = float_State.data;

    return *this;
}

	
bool Float_State::operator==(const Float_State& state) const
{
    return this->data == state.data;
}

bool Float_State::operator<(const Float_State& state) const
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
        
void Float_State::write(uint index, double value)
{
    QUERY_UNRECOVERABLE_ERROR(index >= data.size(),
                              "Failed trying to read float at index :: "
                              <<index<<std::endl
                              <<" as that entry is out of bounds.");
    
    data[index] = value;
}
       
// void Float_State::increment(uint index)
// {
//     data[index]++;
// }

// void Float_State::decrement(uint index)
// {
//     assert(data[index] > 0);
//     data[index]--;
// }

double Float_State::read(uint index) const
{
    QUERY_UNRECOVERABLE_ERROR(index >= data.size(),
                              "Failed trying to read float at index :: "
                              <<index<<std::endl
                              <<" as that entry is out of bounds.");
    
    assert(index < data.size());

    return data[index];
}

bool Float_State::is_equal(uint index, double value) const
{
    QUERY_UNRECOVERABLE_ERROR(index >= data.size(),
                              "Failed trying to read float at index :: "
                              <<index<<std::endl
                              <<" as that entry is out of bounds.");
    
    assert(index < data.size());

    return (value == data[index]);
}

        
std::size_t Float_State::hash_value() const
{
    return boost::hash_range(data.begin(), data.end());
}


std::size_t Float_State::hash_value(const std::vector<double>& input)
{
    return boost::hash_range(input.begin(), input.end());
}

uint Float_State::size() const
{
    return data.size();
}

std::size_t std::hash_value(const Float_State& state)
{
    return state.hash_value();
}
    
std::ostream& std::operator<<(std::ostream& o, const Float_State& state)
{
    /*Print out the fluent propositions first.*/
    for(uint i = 0; i < state.data.size(); i++){
        o<<i<<" := "<<state.data[i]<<"; ";
    }
    
    return o;
}
