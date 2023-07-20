#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

//QT
#include <QMainWindow>
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include <QPushButton>
#include <QApplication>
#include <QAction>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QButtonGroup>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QStyle>
#include <QtCore>
#include <QToolButton>
#include <qstring.h>
#include <QComboBox>
#include <QSpinBox>
#include <QListWidgetItem>
#include <QListView>
#include <QKeyEvent>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QSplashScreen>
#include <QElapsedTimer>
#include <QProgressBar>
#include <QCheckBox>

//C++
#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <sstream>
#include <math.h>

#include "offline_playback.h"

class QVTKWidget;

class MainWindow: public QMainWindow {
	Q_OBJECT
public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();
	void ReadIcons();
	void DesignAppearance();
	void OpenFile();
	void StartProcess();
	void EndProcess();
	void NextProcess();
	void BackProcess();
	void UpdateProgress();
	void FindStartSeq();
private:
	QWidget * slide_widget_;
	QSlider* data_slider_;

    QComboBox *date_box_;

	QPushButton *file_button_;
    QPushButton *start_button_;
    QPushButton *end_button_;
    QPushButton *pre_button_;
    QPushButton *next_button_;

	QLabel* date_label_;
	QLabel* time_label_;

	QSpinBox *frame_spinbox_;

	QCheckBox *lidar_checkbox_;
	QCheckBox *radar_checkbox_;
	QCheckBox *imu_checkbox_;

	OfflinePub pub_module_;
	int slider_l = 320;
	int slider_w = 40;

	int stop_ = true;
	int end_  = true;
	std::mutex seq_mutex;
	int maxframe_;
	int seq_;
	int pre_seq_;
	int pretime = 0;
	bool open_file_ = false;


	std::vector<QPixmap> imgs_;

public Q_SLOTS:
	void SlotLidarState(int state);
	void SlotRadarState(int state);
	void SlotImuState(int state);
	void SlotSeqValue(int value);
};
#endif
