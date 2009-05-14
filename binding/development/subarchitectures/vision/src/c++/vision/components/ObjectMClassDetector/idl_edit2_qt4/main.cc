#include <stdlib.h>
#include <qapplication.h>
#include "idl_edit.hh"

int main( int argc, char **argv )
{
 
    QApplication a( argc, argv );

    IDL_Edit w;
    a.setMainWidget( &w );
    w.show();
    return a.exec();
}
