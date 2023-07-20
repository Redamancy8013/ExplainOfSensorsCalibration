#include "mainwindow.h"
#include "my_log.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    string filepath = __FILE__;
    boost::char_separator<char> sep { "/" };
    tokenizer token(filepath, sep);
    vector<string> path(token.begin(), token.end());
    string log_path;
    for (int i = 0; i < path.size() - 1; i++) {
        log_path = log_path + "/" + path[i];
    }
    string foldpath = log_path + "/log";
    mkdir(foldpath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    log_path = foldpath + "/" + "log.txt";
    cout <<"log file will save to:"<< log_path << endl;
    MyLogInit(log_path);
    w.show();
    return a.exec();
}
