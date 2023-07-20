// #######################################################################################
// #                                Dbag tool                                            #
// #                    contact: wangxubit@foxmail.com                                   #
// #                            可视化界面用于回放数据                                       #
// #######################################################################################

#include "mainwindow.h"
#include <QApplication>
#include <sys/stat.h>
#include <sys/types.h>

int main(int argc, char * argv[]){
	QApplication mainapp(argc, argv);
	MainWindow window;
	window.show();
	return mainapp.exec();
}