#include <QApplication>
#include "myProject.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    myProject myproject;
    myproject.show();
    return app.exec();
}
