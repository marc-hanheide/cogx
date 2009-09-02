#ifndef DATABASE_HH
#define DATABASE_HH

#include "global.hh"

#include <sqlite3.h>

#define SQL_ADDRESS_TYPE ((sizeof(void*) == 8)?"bigint":"int")

/*Database for caching state information.*/
class Database
{
public:
    Database(uint state_size);
    
    class SQL_COMMANDS
    {
    private:
        const string table_name;
        const string id_column;
        const string data_column;
        const string data_column_type;

        uint state_size;
    public:
        SQL_COMMANDS(uint state_size):
            state_size(state_size),
            table_name("states"),
            id_column("State_Address"),
            data_column("State_Vector"),
            data_column_type("blob")
        {  
        };
    
        string GET__CREATE_TABLE_COMMAND()
        {
            ostringstream oss; 
            oss<<"CREATE TABLE "<<table_name<<" ( "
               <<id_column<<" "<<SQL_ADDRESS_TYPE<<", "
               <<data_column<<" "<<data_column_type<<", "//<<" char("<<state_size<<"), "
               <<"PRIMARY KEY ("<<id_column<<"))";
        
            return oss.str();
        }

        string GET__INSERT_STATE_COMMAND(void* id, const vector<unsigned int> data)
        {
            ostringstream oss;
            
            assert(sizeof(int) == sizeof(void*));
        
            oss<<"INSERT INTO "<<table_name<<" ("<<id_column<<", "<<data_column<<")"
               <<" VALUES ("<<reinterpret_cast<int>(id)<<", ";

            for(int i =0; i < data_size; i++){
                character_data<<data[i]<<((i = data_size - 1)?")":", ");
            }

            return oss.str();
        }

        string GET__GET_STATE_COMMAND(void* id)
        {
            ostringstream oss;

            assert(sizeof(int) == sizeof(void*));
        
            oss<<"SELECT "<<data_column
               <<" FROM "<<table_name
               <<" WHERE "<<id_column<<" = "<<reinterpret_cast<int>(id);
        
            return oss.str();
        }
    };
private:
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;
    SQL_COMMANDS sql_commands;
};


#endif
