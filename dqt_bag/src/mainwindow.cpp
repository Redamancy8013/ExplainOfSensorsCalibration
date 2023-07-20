#include "mainwindow.h"

using namespace std;

MainWindow::~MainWindow()
{
	if(slide_widget_) delete slide_widget_;
	if(data_slider_) delete data_slider_;
	if(file_button_) delete file_button_;
	if(end_button_) delete end_button_;
	if(pre_button_) delete pre_button_;
	if(next_button_) delete next_button_;
	if(date_label_) delete date_label_;
	if(time_label_) delete time_label_;
	if(frame_spinbox_) delete frame_spinbox_;
	if(lidar_checkbox_) delete lidar_checkbox_;
	if(radar_checkbox_) delete radar_checkbox_;
	if(imu_checkbox_) delete imu_checkbox_;
};

/**
 * @brief The main window of QT
 * @details 
 * @param 
 */
MainWindow::MainWindow( QWidget *parent) : QMainWindow( parent )
{
	int wdghorizonsize = 380;

	//main window
	setWindowTitle(tr("dqt_bag"));//title for mainwindow

	//主窗口
	QWidget * main_widget = new QWidget(this); 
	QVBoxLayout *mainlay = new QVBoxLayout(main_widget);

	//按键总插件
	QWidget * button_widget = new QWidget(this); 
	QHBoxLayout *button_hlay = new QHBoxLayout(button_widget);
	button_hlay->setDirection(QBoxLayout::LeftToRight);

	file_button_ = new QPushButton("");
	file_button_->setFixedSize(30 ,30); 
	file_button_->setFocusPolicy(Qt::NoFocus); 
	button_hlay->addWidget(file_button_);
  	connect(file_button_, &QPushButton::clicked, this, &MainWindow::OpenFile);

	pre_button_ = new QPushButton();
	pre_button_->setFixedSize(30 ,30); 
	pre_button_->setFocusPolicy(Qt::NoFocus); 
	button_hlay->addWidget(pre_button_);
  	connect(pre_button_, &QPushButton::clicked, this, &MainWindow::BackProcess);

	start_button_ = new QPushButton();
	start_button_->setFixedSize(30 ,30); 
	start_button_->setFocusPolicy(Qt::NoFocus); 
	button_hlay->addWidget(start_button_);
  	connect(start_button_, &QPushButton::clicked, this, &MainWindow::StartProcess);

	end_button_ = new QPushButton();
	end_button_->setFixedSize(30 ,30); 
	end_button_->setFocusPolicy(Qt::NoFocus); 
	button_hlay->addWidget(end_button_);
  	connect(end_button_, &QPushButton::clicked, this, &MainWindow::EndProcess);

	next_button_ = new QPushButton();
	next_button_->setFixedSize(30 ,30); 
	next_button_->setFocusPolicy(Qt::NoFocus); 
	button_hlay->addWidget(next_button_);
  	connect(next_button_, &QPushButton::clicked, this, &MainWindow::NextProcess);

	// frame_spinbox_ = new QSpinBox(this);
	// button_hlay->addWidget(frame_spinbox_);

	lidar_checkbox_ = new QCheckBox("lidar", this);
	lidar_checkbox_->setChecked(Qt::Checked);
	connect(lidar_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(SlotLidarState(int)));
	radar_checkbox_ = new QCheckBox("radar", this);
	radar_checkbox_->setChecked(Qt::Checked);
	connect(radar_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(SlotRadarState(int)));
	imu_checkbox_ = new QCheckBox("imu", this);	
	imu_checkbox_->setChecked(Qt::Checked);
	connect(imu_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(SlotImuState(int)));
	button_hlay->addWidget(lidar_checkbox_);
	button_hlay->addWidget(radar_checkbox_);
	button_hlay->addWidget(imu_checkbox_);

	date_label_ = new QLabel(this);
	date_label_->setFixedSize(150 ,30);
	button_hlay->addWidget(date_label_);

  	button_hlay->addStretch();
	button_widget->setLayout(button_hlay);
	button_widget->setMinimumSize(1000, 50); 
	button_widget->setStyleSheet("background-color: rgb(230,230,230);");	

	mainlay->addWidget(button_widget);

	//滑动条
	slide_widget_ = new QWidget(this);  
	QVBoxLayout *slide_vlay = new QVBoxLayout(slide_widget_);
	slide_widget_->setMinimumSize(300, 100); 
	data_slider_ = new QSlider(this);
	data_slider_->setOrientation(Qt::Horizontal);
	connect(data_slider_, SIGNAL(valueChanged(int)), this, SLOT(SlotSeqValue(int)));

	slide_vlay->addWidget(data_slider_);
	slide_vlay->addStretch();
	slide_widget_->setLayout(slide_vlay);

  	mainlay->addWidget(slide_widget_);

	//时间显示
	QWidget * time_widget = new QWidget(this); 
	QHBoxLayout *time_hlay = new QHBoxLayout(time_widget);
	time_hlay->setDirection(QBoxLayout::RightToLeft);
	time_label_ = new QLabel(this);
	time_label_->setFixedSize(220 ,30);
	time_hlay->addWidget(time_label_);
	time_hlay->addStretch();
	time_widget->setLayout(time_hlay);

  	mainlay->addWidget(time_widget);
	this->ReadIcons();
  	this->DesignAppearance();
  	main_widget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
	slide_widget_->setStyleSheet("background-color: rgb(230,230,230);");	

	setCentralWidget(main_widget);
	
	std::thread progress_t(&MainWindow::UpdateProgress, this);
	progress_t.detach();
	maxframe_ = 0;
	seq_ = 0;
	pre_seq_ = 0;
}

void MainWindow::ReadIcons(){
 	QPixmap file_pixmap("../icons/file.png");
	imgs_.push_back(file_pixmap);

 	QPixmap arrow_r_pixmap("../icons/next.png");
	imgs_.push_back(arrow_r_pixmap);

 	QPixmap arrow_l_pixmap("../icons/pre.png");
	imgs_.push_back(arrow_l_pixmap);

 	QPixmap start_pixmap("../icons/start.png");
	imgs_.push_back(start_pixmap);

	QPixmap stop_pixmap("../icons/stop.png");
	imgs_.push_back(stop_pixmap);

	QPixmap end_pixmap("../icons/end.png");
	imgs_.push_back(end_pixmap);

	QPixmap openfile_pixmap("../icons/openfile.png");
	imgs_.push_back(openfile_pixmap);
}

void MainWindow::OpenFile(){
	file_button_->setIcon(QIcon(imgs_[6]));
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
		cout<<str_dir[0].toStdString()<<endl;
		string file_path = str_dir[0].toStdString();
		int cut_start = file_path.size()-1;
		for(; cut_start>=0; --cut_start){
			if(file_path[cut_start]=='/')
				break;
		}
		string folder_name;
		if(file_path.size()>0)
			folder_name = " " + file_path.substr(cut_start+1, file_path.size()-cut_start+1) + " ";
		else
			folder_name = " " + file_path + " ";
		QString date = QString::fromStdString(folder_name);
		date_label_->setText(date);

		pub_module_.InputFilePath(file_path);
		pub_module_.PreProcess();
		data_slider_->setMaximum((pub_module_.max_time()-pub_module_.min_time())*1e-3);
		open_file_ = true;

		data_slider_->setMinimum(0);
		data_slider_->setSingleStep(1);
		data_slider_->setPageStep(1);
		pub_module_.Start();
	}
	file_button_->setIcon(QIcon(imgs_[0]));
}

void MainWindow::StartProcess(){
	if(!open_file_)
		return;
	stop_ = !stop_;
	if(stop_){
		start_button_->setIcon(QIcon(imgs_[4]));
	}else{
		start_button_->setIcon(QIcon(imgs_[3]));
	}
	pub_module_.Stop();
}

void MainWindow::EndProcess(){
	if(!open_file_)
		return;
	end_ = false;
	open_file_ = false;
	pub_module_.End();
}

void MainWindow::UpdateProgress(){
	while(true){
		if(stop_ || !open_file_)
			continue;
		std::unique_lock<std::mutex> qlock(seq_mutex);
		int64_t duration = pub_module_.duration();
		if(fabs(duration - pretime)>100){
			data_slider_->setValue(duration);
			pretime = duration;
			string time = toDateTime((pub_module_.min_time() + duration *1e3) * 1e3);
			time_label_->setText(QString::fromStdString(time));
		}
		qlock.unlock();
	}
	return;
}

void MainWindow::BackProcess(){
	if(!stop_)
		return;
	if((seq_--) == 0)
		return;
	std::unique_lock<std::mutex> qlock(seq_mutex);
	string time;
	pub_module_.bag_seq_ = seq_;
	// pub_module_.FindStartSeq();
	qlock.unlock();
	// frame_spinbox_->setValue(seq_);
	pre_seq_ = seq_;
}

void MainWindow::NextProcess(){
	if(!stop_)
		return;
	if((seq_++) == (maxframe_ - 1))
		return;
	std::unique_lock<std::mutex> qlock(seq_mutex);
	string time;
	pub_module_.bag_seq_ = seq_;
	// pub_module_.FindStartSeq();
	qlock.unlock();
	// frame_spinbox_->setValue(seq_);
	pre_seq_ = seq_;
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
	time_label_->setText(QString::fromStdString(time));
	qlock.unlock();
	pre_seq_ = seq_;
}

