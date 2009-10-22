#include <tools/data_plot_test.h>

int main(int argc, char **argv)
{
    QApplication a(argc, argv);

    vector<double> data;
    int max_size = 40;
    MainWindow mainWindow (max_size, data);

    mainWindow.resize(600,400);
    mainWindow.start();
    mainWindow.show();

    return a.exec();
}
