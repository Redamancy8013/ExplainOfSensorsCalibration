#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>

#include <string>
#include <time.h>
#include <ctime>
#include <iostream>
#include <thread>
#include <mutex>

#include <progress.h>
#include <offline_playback.h>
#include "my_log.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_actionFileOpen_triggered();

    void on_actionStop_triggered();

    void on_actionEnd_triggered();

    void on_actionpanel_triggered();

    void on_dockWidget_visibilityChanged(bool visible);

    void on_actionNextFrame_triggered();

    void on_actionPreFrame_triggered();

    void on_actionInfo_triggered();

    void on_actiontool_bar_triggered();

    void SlotLidarState(int state);

    void SlotRadarState(int state);

    void SlotImuState(int state);

    void SlotSeqValue(int value);

    void SlotTimelabel(QString value);

    void SlotSliderUpdate(int value);

signals:
    void timeUpdate(QString);

    void progressUpdate(int);

private:
    void ReadIcons();

    void UpdateProgress();

    OfflinePub pub_module_;
    bool open_file_ = false;
    bool panel_flag_ = false;
    bool stop_ = true;
    bool end_  = true;
    int pretime = 0;

    std::mutex seq_mutex;

    Ui::MainWindow *ui;
    QLabel *label;
    Progress* pslider;
    std::vector<QPixmap> imgs_;
};
#endif // MAINWINDOW_H
