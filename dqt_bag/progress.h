#ifndef PROGRESS_H
#define PROGRESS_H
#include <QSlider>
#include <QMouseEvent>
#include <QStylePainter>
#include <QStyleOptionSlider>

#include <iostream>
#include <stdlib.h>
#include <cmath>

class Progress:public QSlider
{
    Q_OBJECT
public:
    explicit Progress(QWidget *parent = 0);
    virtual ~Progress(){};
	/**
	 * @brief set slider min value
	 */
    void SetMinValue(float min);
	/**
	 * @brief set slider max value
	 */
    void SetMaxValue(float max);
    void SetStep(float step);

signals:
    void sliderValue(float);

private:
	/**
	 * @brief mousePressEvent
	 * @details when mouse press the slider, the slider value 
	 *			will be set as the current mouse press position
	 */
    void mousePressEvent(QMouseEvent *event) override;

	/**
	 * @brief mouseMoveEvent
	 */
    void mouseMoveEvent(QMouseEvent *ev) override;

	/**
	 * @brief paint the tick
	 */
    void paintEvent(QPaintEvent *ev);

    float minv_;
    float maxv_;
    float step_;
};
#endif // PROGRESS_H
