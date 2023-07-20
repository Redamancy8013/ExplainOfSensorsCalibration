#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ReadIcons();

    ui->setupUi(this);

    label = new QLabel();

    label->setStyleSheet("\
        color: rgb(0,0,0);\
        background:rgb(200,200,200);");
    label->setFixedSize(250 ,20);
    statusBar()->addWidget(label);

    ui->ProgressSlider->SetMaxValue(100);
    ui->ProgressSlider->SetMinValue(0);
    ui->actionFileOpen->setIcon(QIcon(imgs_[0]));
    ui->actionStop->setIcon(QIcon(imgs_[4]));
    ui->actionPreFrame->setIcon(QIcon(imgs_[2]));
    ui->actionNextFrame->setIcon(QIcon(imgs_[1]));
    ui->actionEnd->setIcon(QIcon(imgs_[7]));
    ui->actionpanel->setIcon(QIcon(imgs_[8]));

    ui->LidarcheckBox->setChecked(Qt::Checked);
    ui->RadarcheckBox->setChecked(Qt::Checked);
    ui->InscheckBox->setChecked(Qt::Checked);

    connect(ui->LidarcheckBox, SIGNAL(stateChanged(int)), this, SLOT(SlotLidarState(int)));
    connect(ui->RadarcheckBox, SIGNAL(stateChanged(int)), this, SLOT(SlotRadarState(int)));
    connect(ui->InscheckBox, SIGNAL(stateChanged(int)), this, SLOT(SlotImuState(int)));
    connect(ui->ProgressSlider, SIGNAL(valueChanged(int)), this, SLOT(SlotSeqValue(int)));
    connect(this, SIGNAL(timeUpdate(QString)), this, SLOT(SlotTimelabel(QString)));
    connect(this, SIGNAL(progressUpdate(int)), this, SLOT(SlotSliderUpdate(int)));

    ui->dockWidget->hide();

//    std::thread progress_t(&MainWindow::UpdateProgress, this);
    std::thread progress_t(std::bind(&MainWindow::UpdateProgress, this));
    progress_t.detach();
}

MainWindow::~MainWindow()
{
    delete label;
    delete ui;
}

void MainWindow::ReadIcons(){
    string projectpath = __FILE__;
    boost::char_separator<char> sep { "/" };
    tokenizer token(projectpath, sep);
    vector<string> path(token.begin(), token.end());
    string icon_path;
    for (int i = 0; i < path.size() - 1; i++) {
        icon_path = icon_path + "/" + path[i];
    }
	
	icon_path+="/icons/";
	std::cout<<icon_path<<std::endl;
    QPixmap file_pixmap(QString::fromStdString(icon_path + "file.png"));
    imgs_.push_back(file_pixmap);

    QPixmap arrow_r_pixmap(QString::fromStdString(icon_path + "next.png"));
    imgs_.push_back(arrow_r_pixmap);

    QPixmap arrow_l_pixmap(QString::fromStdString(icon_path + "pre.png"));
    imgs_.push_back(arrow_l_pixmap);

    QPixmap start_pixmap(QString::fromStdString(icon_path + "start.png"));
    imgs_.push_back(start_pixmap);

    QPixmap stop_pixmap(QString::fromStdString(icon_path + "stop.png"));
    imgs_.push_back(stop_pixmap);

    QPixmap end_pixmap(QString::fromStdString(icon_path + "end.png"));
    imgs_.push_back(end_pixmap);

    QPixmap openfile_pixmap(QString::fromStdString(icon_path + "openfile.png"));
    imgs_.push_back(openfile_pixmap);

    QPixmap play_pixmap(QString::fromStdString(icon_path + "playing.png"));
    imgs_.push_back(play_pixmap);

    QPixmap panel_pixmap(QString::fromStdString(icon_path + "panel.png"));
    imgs_.push_back(panel_pixmap);
}

void MainWindow::on_actionFileOpen_triggered()
{
    ui->actionFileOpen->setIcon(QIcon(imgs_[6]));
    QFileDialog *fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(QObject::tr("Open annotation file path"));
    fileDialog->setDirectory(".");
    fileDialog->setFileMode(QFileDialog::DirectoryOnly);
    fileDialog->setViewMode(QFileDialog::Detail);
    fileDialog->setOption(QFileDialog::ShowDirsOnly);
    QStringList fileNames;
    QString str;
    QStringList fileNames2;
    if (fileDialog->exec()) {
        QStringList str_dir = fileDialog->selectedFiles();
        std::cout<<str_dir[0].toStdString()<<std::endl;
        std::string file_path = str_dir[0].toStdString();
        int cut_start = file_path.size()-1;
        for(; cut_start>=0; --cut_start){
            if(file_path[cut_start]=='/')
                break;
        }
        std::string folder_name;
        if(file_path.size()>0)
            folder_name = " " + file_path.substr(cut_start+1, file_path.size()-cut_start+1) + " ";
        else
            folder_name = " " + file_path + " ";
        QString date = QString::fromStdString(folder_name);
        pub_module_.InputFilePath(file_path);
        unsigned char status = pub_module_.PreProcess();
        if(status == 0x0f){
            QMessageBox::StandardButton result = QMessageBox::critical(NULL,
                    "Error", "The data path is wrong!");
            switch (result) {
            default:
                break;
            }
            ui->actionFileOpen->setIcon(QIcon(imgs_[0]));
            return;
        }else if(status != 0x00){
            std::string stat = "Lost ";
            bool show_flag = false;
            if(status & 1 == 1){
                stat += "Lidar ";
                show_flag = true;
            }
            if((status>>1) & 1 == 1){
                stat += "Radar ";
                show_flag = true;
            }
            if((status>>2) & 1 == 1){
                stat += "Ins ";
                show_flag = true;
            }
            if((status>>3) & 1 == 1){
                stat += "Camera ";
                show_flag = true;
            }

            stat += "data! \n Make sure the data doesn't include these sensors!";

            std::string emptystat = "File ";
            if((status>>4)  & 1 == 1){
                emptystat += "Lidar ";
                show_flag = false;
            }
            if((status>>5) & 1 == 1){
                emptystat += "Radar ";
                show_flag = false;
            }
            if((status>>6) & 1 == 1){
                emptystat += "Ins ";
                show_flag = false;
            }
            if((status>>7) & 1 == 1){
                emptystat += "Camera ";
                show_flag = false;
            }
            emptystat+= "is empty! \n Make sure the data path is right!";

            if(show_flag){
                QMessageBox::StandardButton result = QMessageBox::warning(NULL,
                        "Warning", QString::fromStdString(stat));
                switch (result) {
                default:
                    break;
                }
            }else{
                QMessageBox::StandardButton result = QMessageBox::critical(NULL,
                        "Error", QString::fromStdString(emptystat));
                switch (result) {
                default:
                    break;
                }
            }
        }

        ui->ProgressSlider->SetMaxValue((pub_module_.max_time()-pub_module_.min_time())*1e-3);
        open_file_ = true;

        ui->ProgressSlider->SetMinValue(0);
        ui->ProgressSlider->setSingleStep(100);
        ui->ProgressSlider->setPageStep(100);
        pub_module_.Start();
        std::cout<<"pub_module_.Start() "<<std::endl;
        ui->actionEnd->setIcon(QIcon(imgs_[5]));
    }
    cout<<"xxxxxxxxxxxxxx"<<endl;
    ui->actionFileOpen->setIcon(QIcon(imgs_[0]));
    cout<<"xxxxxxxxxxxxxx"<<endl;

}

void MainWindow::UpdateProgress(){
    while(true){
        if(stop_ || !open_file_){
            usleep(1e4);
            continue;
        }
        std::unique_lock<std::mutex> qlock(seq_mutex);
        int64_t duration = pub_module_.duration();
		pretime = duration;
		string cur_time = toDateTime((pub_module_.min_time() + duration *1e3) * 1e3);
		emit timeUpdate(QString::fromStdString("  " + cur_time));
		emit progressUpdate((int)duration);
		qlock.unlock();
		usleep(1e5);
    }
    return;
}

void MainWindow::on_actionStop_triggered()
{
    if(!open_file_)
        return;
    pub_module_.Stop();
    stop_ = !stop_;

    if(stop_){
        ui->actionStop->setIcon(QIcon(imgs_[4]));
    }else{
        ui->actionStop->setIcon(QIcon(imgs_[3]));
    }
}

void MainWindow::on_actionEnd_triggered()
{
    if(!open_file_)
        return;
    end_ = false;
    open_file_ = false;
    pub_module_.End();
    ui->ProgressSlider->setValue(0);
    ui->ProgressSlider->SetMaxValue(100);
    ui->ProgressSlider->SetMinValue(0);
    label->setText("");
    ui->actionEnd->setIcon(QIcon(imgs_[7]));
    if(!stop_){
        stop_ = true;
        pub_module_.Stop();
        ui->actionStop->setIcon(QIcon(imgs_[4]));
    }
}

void MainWindow::SlotLidarState(int state){
    if(state == Qt::Checked){
        std::unique_lock<std::mutex> qlock(seq_mutex);
        pub_module_.lidar_flag_ = true;
        qlock.unlock();
    }else{
        std::unique_lock<std::mutex> qlock(seq_mutex);
        pub_module_.lidar_flag_ = false;
        qlock.unlock();
    }
}

void MainWindow::SlotRadarState(int state){
    if(state == Qt::Checked){
        std::unique_lock<std::mutex> qlock(seq_mutex);
        pub_module_.radar_flag_ = true;
        qlock.unlock();
    }else{
        std::unique_lock<std::mutex> qlock(seq_mutex);
        pub_module_.radar_flag_ = false;
        qlock.unlock();
    }
}

void MainWindow::SlotImuState(int state){
    if(state == Qt::Checked){
        std::unique_lock<std::mutex> qlock(seq_mutex);
        pub_module_.imu_flag_ = true;
        qlock.unlock();
    }else{
        std::unique_lock<std::mutex> qlock(seq_mutex);
        pub_module_.imu_flag_ = false;
        qlock.unlock();
    }
}

void MainWindow::SlotSeqValue(int value){
    if(!open_file_)
        return;
    if(!stop_)
        return;
    int64_t time_duration = value * 1e3;
    std::unique_lock<std::mutex> qlock(seq_mutex);
    pub_module_.FindStartSeq(time_duration);
    string time = toDateTime((pub_module_.min_time() + time_duration) * 1e3);
    label->setText(QString::fromStdString(time));
    qlock.unlock();
}

void MainWindow::SlotTimelabel(QString value){
    label->setText(value);
}

void MainWindow::SlotSliderUpdate(int value){
    ui->ProgressSlider->setValue(value);
}

void MainWindow::on_actionpanel_triggered()
{
    if(panel_flag_){
        ui->dockWidget->hide();
        panel_flag_ = false;
    }else{
        ui->dockWidget->show();
        panel_flag_ = true;
    }
}

void MainWindow::on_dockWidget_visibilityChanged(bool visible)
{
    panel_flag_ = !panel_flag_;
}

void MainWindow::on_actionNextFrame_triggered()
{
    if(!open_file_)
        return;
    if(!stop_)
        return;
    int64_t timev = ui->ProgressSlider->value();
    if(timev >= ui->ProgressSlider->maximum())
        return;
    int64_t time_duration = (timev + 10) * 1e3;
    ui->ProgressSlider->setValue(timev + 10);
    std::unique_lock<std::mutex> qlock(seq_mutex);

    pub_module_.FindStartSeq(time_duration);
    string time = toDateTime((pub_module_.min_time() + time_duration) * 1e3);
    label->setText(QString::fromStdString(time));
    qlock.unlock();
}

void MainWindow::on_actionPreFrame_triggered()
{
    if(!open_file_)
        return;
    if(!stop_)
        return;
    int64_t timev = ui->ProgressSlider->value();
    if(timev <= ui->ProgressSlider->minimum())
        return;
    int64_t time_duration = (timev - 10) * 1e3;
    ui->ProgressSlider->setValue(timev - 10);
    std::unique_lock<std::mutex> qlock(seq_mutex);
    pub_module_.FindStartSeq(time_duration);
    string time = toDateTime((pub_module_.min_time() + time_duration) * 1e3);
    label->setText(QString::fromStdString(time));
    qlock.unlock();
}

void MainWindow::on_actionInfo_triggered()
{
    QMessageBox::StandardButton result =
            QMessageBox::information(NULL, "About",
                    "This is a data replay tool which is fitted for Dnet. \n\n\n\n\n\n "
                    "software version: V1.0.0:20220508");
    switch (result) {
    default:
        break;
    }
}

void MainWindow::on_actiontool_bar_triggered()
{
    ui->toolBar->show();
}
