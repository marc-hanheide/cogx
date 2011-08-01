#ifndef PARSER_HH
#define PARSER_HH
  

string string_constant__CB;
string string_constant__0;
string string_constant__L;
string string_constant__SD;


void print()
{
    cout<<string_constant__CB<<" "
        <<number_of_circuit_breakers<<" "
        <<string_constant__0<<" "
        <<string_constant__L<<" "
        <<number_of_lines<<" "
        <<string_constant__SD<<" "
        <<number_of_switches<<" "
        <<string_constant__0<<" "
        <<std::endl;
    
    for(auto  line__to__device =  line__to__devices.begin()
            ; line__to__device != line__to__devices.end()
            ; line__to__device++){
        auto line_Id = line__to__device->first;
        auto& devices = line__to__device->second;

        
        for(auto device1 = devices.begin()
                ; device1 != devices.end()
                ; device1++){
            for(auto device2 = device1
                    ; device2 != devices.end()
                    ; device2++){
                if(device2 == device1) device2++;
                if(device2 == devices.end()) break;
                
                
                assert(device__to__line__to__side.find(*device1) != device__to__line__to__side.end());
                assert(device__to__line__to__side.find(*device2) != device__to__line__to__side.end());
                
                assert(device__to__line__to__side[*device1].find(line_Id)
                       != device__to__line__to__side[*device1].end());
                assert(device__to__line__to__side[*device2].find(line_Id)
                       != device__to__line__to__side[*device2].end());
                
                auto s1 = device__to__line__to__side[*device1][line_Id];
                auto s2 = device__to__line__to__side[*device2][line_Id];
                auto device1_typestring = type_to_string(device1->second);
                auto device2_typestring = type_to_string(device2->second);
                auto device1_side = side_to_int(s1);
                auto device2_side = side_to_int(s2);
                auto device1_id = device1->first;
                auto device2_id = device2->first;
                
                cout<<device1_typestring<<" "
                    <<device1_id<<" "
                    <<device1_side<<" "
                    <<" l "
                    <<line_Id<<" "
                    <<device2_typestring<<" "
                    <<device2_id<<" "
                    <<device2_side<<"\n";
            }
        }
    }

    
    for(auto faulty_line = faulty_lines.begin()
            ; faulty_line != faulty_lines.end()
            ; faulty_line++){
        cout<<"l "<<*faulty_line<<" faulty"<<std::endl;
        
    }

    for(auto closed_device = closed_devices.begin()
            ; closed_device != closed_devices.end()
            ; closed_device++){
        auto device_typestring = type_to_string(closed_device->second);
        cout<<device_typestring<<" "<<closed_device->first<<" closed"<<std::endl;
        
    }
}


void parse(const string& file_name)
{
    FILE * file = 0;
    char ch;
    file = fopen(file_name.c_str(), "r");

    {if(!file) UNRECOVERABLE_ERROR("Cannot open file :: "<<file_name);}
    
    {if(ferror(file)) UNRECOVERABLE_ERROR("Cannot open file :: "<<file_name);}

    // CB <c> 0 L <l> SD <s> 0
    string first_line = "";
    while((ch = getc(file)) != '\n' &&
          ch != EOF){
        first_line += ch;
    }
    
    istringstream iss(first_line);
    
    
    iss>>string_constant__CB
       >>number_of_circuit_breakers
       >>string_constant__0
       >>string_constant__L
       >>number_of_lines
       >>string_constant__SD
       >>number_of_switches
       >>string_constant__0;

    do{
        
        string line = "";
        while((ch = getc(file)) != '\n' &&
              ch != EOF){
            line += ch;
        }

        if(ch == EOF) continue;
        
        cerr<<line<<std::endl<<std::endl;
        
        // [cb|sw] INT__DEVICE_ID [1|2] 'l' INT__LINE_ID [cb|sw] INT__DEVICE_ID [1|2]
        // [cb|sw] INT__DEVICE_ID closed
        // 'l' INT__LINE_ID faulty
        istringstream line_iss(line);
        string line_or_device = "";
        
        line_iss>>line_or_device;

        if(line_or_device == "cb" || line_or_device == "sd"){

            
            Device_Id starting_device_Id;
            uint id;
            line_iss>>id;
            if(line_or_device == "cb"){
                starting_device_Id = Device_Id(id, circuit_breaker);
            } else if (line_or_device == "sd") {
                starting_device_Id = Device_Id(id, remote_switch);
                
            }
            
            
            string str;
            line_iss>>str;

            if(str == "open"){
                continue;
            } else if (str == "closed") {
                closed_devices.insert(starting_device_Id);
                continue;
            } else if (str == "1" || str == "2") {

                Side starting_side = Side::side1;
                if (str == "1" ) {
                    starting_side = Side::side1;
                } else if (str == "2") {
                    starting_side = Side::side2;
                }

                line_iss>>str;
                assert(str == "l");

                
                Line_Id line_Id;
                line_iss>>line_Id;

                Device_Id ending_device_Id;
                
                line_iss>>line_or_device;
                line_iss>>id;
                cerr<<line_or_device<<" "<<id<<std::endl;
                if(line_or_device == "cb"){
                    ending_device_Id = Device_Id(id, circuit_breaker);
                } else if (line_or_device == "sd") {
                    ending_device_Id = Device_Id(id, remote_switch);       
                } else {
                    UNRECOVERABLE_ERROR("unparsable device.");
                }
                





                
                line_iss>>str;
                
                Side ending_side = Side::side1;
                if (str == "1" ) {
                    ending_side = Side::side1;
                } else if (str == "2") {
                    ending_side = Side::side2;
                }




                

                if(device__to__side__to__line.find(starting_device_Id) == device__to__side__to__line.end()){
                    device__to__side__to__line[starting_device_Id] = map<Side, Line_Id>();
                }
                
                if(device__to__line__to__side.find(starting_device_Id) == device__to__line__to__side.end()){
                    device__to__line__to__side[starting_device_Id] = map<Line_Id, Side>();
                }
                

                if(device__to__side__to__line.find(ending_device_Id) == device__to__side__to__line.end()){
                    device__to__side__to__line[ending_device_Id] = map<Side, Line_Id>();
                }
                
                if(device__to__line__to__side.find(ending_device_Id) == device__to__line__to__side.end()){
                    device__to__line__to__side[ending_device_Id] = map<Line_Id, Side>();
                }                

                if( (device__to__side__to__line[starting_device_Id].find(starting_side) !=
                     device__to__side__to__line[starting_device_Id].end()) &&
                    (device__to__side__to__line[ending_device_Id].find(ending_side) !=
                     device__to__side__to__line[ending_device_Id].end())){
                    if(device__to__side__to__line[starting_device_Id][starting_side] !=
                       device__to__side__to__line[ending_device_Id][ending_side]){
                        UNRECOVERABLE_ERROR("Too many lines assigned to sides.");
                    }
                }
                
                
                if(device__to__side__to__line[starting_device_Id].find(starting_side) !=
                   device__to__side__to__line[starting_device_Id].end()){
                    if(device__to__side__to__line[starting_device_Id][starting_side] != line_Id){
                        cerr<<"Warning, one side asked to take multiple lines.";
                        line_Id = device__to__side__to__line[starting_device_Id][starting_side];   
                    }
                } else if (device__to__side__to__line[ending_device_Id].find(ending_side) !=
                           device__to__side__to__line[ending_device_Id].end()) {

                    if(device__to__side__to__line[ending_device_Id][ending_side] != line_Id){
                        cerr<<"Warning, one side asked to take multiple lines.";
                        line_Id = device__to__side__to__line[ending_device_Id][ending_side];
                    }
                }

                devices.insert(starting_device_Id);
                devices.insert(ending_device_Id);
                lines.insert(line_Id);

                
                if(line__to__devices.find(line_Id) == line__to__devices.end()){
                    line__to__devices[line_Id] = set<Device_Id>();
                }
                
                line__to__devices[line_Id].insert(starting_device_Id);
                line__to__devices[line_Id].insert(ending_device_Id);
                
                device__to__side__to__line[starting_device_Id][starting_side] = line_Id;
                device__to__line__to__side[starting_device_Id][line_Id] = starting_side;
                
                device__to__side__to__line[ending_device_Id][ending_side] = line_Id;
                device__to__line__to__side[ending_device_Id][line_Id] = ending_side;
                
                cerr<<"device line :: "<<type_to_string(starting_device_Id.second)<<" "
                    <<starting_device_Id.first<<" "
                    <<side_to_int(starting_side)<<" "
                    <<" l "<<line_Id<<" "
                    <<type_to_string(ending_device_Id.second)<<" "
                    <<ending_device_Id.first<<" "
                    <<side_to_int(ending_side)<<" "<<std::endl;
                
                
            }  else {
                assert(0);
            }
            
        } else if (line_or_device == "l") {
            Line_Id line_id;
            line_iss>>line_id;
            string status_string;
            line_iss>>status_string;
            if(status_string == "faulty"){
                faulty_lines.insert(line_id);
            }
        } else {
            UNRECOVERABLE_ERROR(line);
        }
        
        
        
    } while(ch != EOF);
    
    print();
}


#endif
