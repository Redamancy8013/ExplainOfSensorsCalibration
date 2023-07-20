#include "mainwindow.h"

void MainWindow::DesignAppearance(){
	data_slider_->setStyleSheet("  \
		QSlider::groove:horizontal{\
			background:rgb(200,200,200);\
			height:30px;\
			border-radius:10px;\
			left: 1px;\
			right: 1px;\
		}\
		QSlider::handle:horizontal{\
			width: 10px;\
			height: px;\
			border-radius: 10px;\
			background: rgb(150,150,150);\
		}\
		QSlider::sub-page:horizontal{\
			border-radius:10px;\
			background: rgb(2,119,189);\
		}\
    ");

	slide_widget_->setStyleSheet("\
		border-color: rgb(100,100,100);\
		background:rgb(255,255,255);");

	date_label_->setStyleSheet("\
		color: rgb(0,0,0);\
		background:rgb(200,200,200);");

	time_label_->setStyleSheet("\
		color: rgb(0,0,0);\
		border-color: rgb(100,100,100);\
		background:rgb(200,200,200);");

	file_button_->setIcon(QIcon(imgs_[0]));
  	file_button_->setStyleSheet("\
		QPushButton{\
				background-color: rgb(230,230,230);\
				border-radius:1px;\
				color:white;\
		}\
		QPushButton:hover\
		{\
			background-color: rgb(255,255,255);\
			border-radius:1px; \
			color:white;\
		}\
		QPushButton:hover:pressed\
		{\
			background-color:rgb(230,230,230); \
			border-radius:1px;\
			color:white;\
   	}");	
    next_button_->setIcon(QIcon(imgs_[1]));
	next_button_->setStyleSheet("\
		QPushButton{\
				background-color: rgb(230,230,230);\
				border-radius:1px;\
				color:white;\
		}\
		QPushButton:hover\
		{\
			background-color: rgb(255,255,255);\
			border-radius:1px; \
			color:white;\
		}\
		QPushButton:hover:pressed\
		{\
			background-color:rgb(230,230,230); \
			border-radius:1px;\
			color:white;\
   	}");	
    pre_button_->setIcon(QIcon(imgs_[2]));
	pre_button_->setStyleSheet("\
		QPushButton{\
				background-color: rgb(230,230,230);\
				border-radius:1px;\
				color:white;\
		}\
		QPushButton:hover\
		{\
			background-color: rgb(255,255,255);\
			border-radius:1px; \
			color:white;\
		}\
		QPushButton:hover:pressed\
		{\
			background-color:rgb(230,230,230); \
			border-radius:1px;\
			color:white;\
   	}");	
   	start_button_->setIcon(QIcon(imgs_[4]));
	start_button_->setStyleSheet("\
		QPushButton{\
				background-color: rgb(230,230,230);\
				border-radius:1px;\
				color:white;\
		}\
		QPushButton:hover\
		{\
			background-color: rgb(255,255,255);\
			border-radius:1px; \
			color:white;\
		}\
		QPushButton:hover:pressed\
		{\
			background-color:rgb(230,230,230); \
			border-radius:1px;\
			color:white;\
   	}");	
    end_button_->setIcon(QIcon(imgs_[5]));
	end_button_->setStyleSheet("\
		QPushButton{\
				background-color: rgb(230,230,230);\
				border-radius:1px;\
				color:white;\
		}\
		QPushButton:hover\
		{\
			background-color: rgb(255,255,255);\
			border-radius:1px; \
			color:white;\
		}\
		QPushButton:hover:pressed\
		{\
			background-color:rgb(230,230,230); \
			border-radius:1px;\
			color:white;\
   	}");
	slide_widget_->setStyleSheet("background-color: rgb(230,230,230);");	
	date_label_->setStyleSheet("background-color: rgb(200,200,200);");
	time_label_->setStyleSheet("background-color: rgb(200,200,200);");
}
