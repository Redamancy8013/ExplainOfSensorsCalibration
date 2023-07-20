#include "progress.h"

Progress::Progress(QWidget* parent): QSlider(parent){
    setStyleSheet("  \
        QSlider::groove:horizontal{\
            background:rgb(200,200,200);\
            height:20px;\
            border-radius:1px;\
            left: 0px;\
            right: 0px;\
        }\
        QSlider::handle:horizontal{\
            width: 10px;\
            height: 20px;\
            border-radius: 3px;\
            background: rgb(250,250,250);\
        }\
        QSlider::sub-page:horizontal{\
            border-radius:1px;\
            background: rgb(2,119,189);\
        }\
    ");

//    std::cout<<"x"<<std::endl;
}

void Progress::SetMinValue(float min){
    minv_ = min;
    this->setMinimum(static_cast<int>(min));
}
void Progress::SetMaxValue(float max){
    maxv_ = max;
//    std::cout<<maxv_<<std::endl;
    this->setMaximum(static_cast<int>(max));
}
void Progress::SetStep(float step){
    step_ = step;
    this->setSingleStep(static_cast<int>(step));
}

void Progress::mousePressEvent(QMouseEvent *event){
    int pointPos = ((double)event->pos().x() / (double)this->width())* (maxv_ - minv_) + minv_;
    if(pointPos != 0){
        if(abs(pointPos - this->value()) > step_){
            this->setValue(pointPos);
        }
    }
    else{
        QSlider::mousePressEvent(event);
    }
}

void Progress::mouseMoveEvent(QMouseEvent *event){
    int pointPos = ((double)event->pos().x() / (double)this->width())* (maxv_ - minv_) + minv_;
    if(pointPos != 0){
        if(abs(pointPos - this->value()) > step_){
            this->setValue(pointPos);
        }
    }
    else{
        QSlider::mousePressEvent(event);
    }
}

void Progress::paintEvent(QPaintEvent *ev){
    QStylePainter painter(this);
    QStyleOptionSlider opt;
    initStyleOption(&opt);

    // 获取滑块的大小
    QRect handle = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);

    QFont font;

    font.setPointSize(8);

    //设置drawText 字体
    painter.setFont(font);

    int interval = (maximum() - minimum())/10;
    int num_count = 0;
    if (tickPosition() != NoTicks)
    {
        for (int i = minimum(); i <= maximum(); i += interval)
        {
            int x = std::round((double)((double)((double)(i - this->minimum())
                                            / (double)(this->maximum() - this->minimum()))
                                   * (double)(this->width() - handle.width()) + (double)(handle.width() / 2.0))) - 1;
            int h = 4;
            painter.setPen(QColor("#a5a294"));
            if (tickPosition() == TicksBothSides || tickPosition() == TicksAbove)
            {
                int y = this->rect().top();
                painter.drawLine(x, y + h*5, x, y + 6*h);

                double mtime = (interval*num_count * 1e-3)/60.;
                std::string time = std::to_string(mtime);
                std::string show;
                int s_index = 0;
                for(int sc=0; sc<time.size(); ++sc){
                    if(time[sc] == '.'){
                        s_index = sc;
                    }
                }
                show = time.substr(0, s_index) + time.substr(s_index, s_index+2);
                if(num_count ==0){
                    painter.drawText(QPoint(x-1, y+h*3), QString::fromStdString(show));
                }else if(num_count == 10){
                    painter.drawText(QPoint(x-20, y+h*3), QString::fromStdString(show));
                }else{
                    painter.drawText(QPoint(x-10, y+h*3), QString::fromStdString(show));
                }
            }
            if (tickPosition() == TicksBothSides || tickPosition() == TicksBelow)
            {
                int y = this->rect().bottom();
                painter.drawLine(x, y, x, y - h);
            }
            num_count++;
        }
    }
    // draw the slider (this is basically copy/pasted from QSlider::paintEvent)
    opt.subControls = QStyle::SC_SliderGroove;
    painter.drawComplexControl(QStyle::CC_Slider, opt);
    // draw the slider handle
    opt.subControls = QStyle::SC_SliderHandle;
    painter.drawComplexControl(QStyle::CC_Slider, opt);
}
