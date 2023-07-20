/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <progress.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionFileOpen;
    QAction *actionNextFrame;
    QAction *actionPreFrame;
    QAction *actionStop;
    QAction *actionEnd;
    QAction *actionpanel;
    QAction *actionInfo;
    QAction *actiontool_bar;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    Progress *ProgressSlider;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuAbout;
    QMenu *menutool;
    QToolBar *toolBar;
    QStatusBar *statusBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *LidarcheckBox;
    QCheckBox *RadarcheckBox;
    QCheckBox *InscheckBox;
    QCheckBox *CameracheckBox;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(568, 203);
        actionFileOpen = new QAction(MainWindow);
        actionFileOpen->setObjectName(QStringLiteral("actionFileOpen"));
        actionNextFrame = new QAction(MainWindow);
        actionNextFrame->setObjectName(QStringLiteral("actionNextFrame"));
        actionPreFrame = new QAction(MainWindow);
        actionPreFrame->setObjectName(QStringLiteral("actionPreFrame"));
        actionStop = new QAction(MainWindow);
        actionStop->setObjectName(QStringLiteral("actionStop"));
        actionEnd = new QAction(MainWindow);
        actionEnd->setObjectName(QStringLiteral("actionEnd"));
        actionpanel = new QAction(MainWindow);
        actionpanel->setObjectName(QStringLiteral("actionpanel"));
        actionInfo = new QAction(MainWindow);
        actionInfo->setObjectName(QStringLiteral("actionInfo"));
        actiontool_bar = new QAction(MainWindow);
        actiontool_bar->setObjectName(QStringLiteral("actiontool_bar"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        ProgressSlider = new Progress(centralwidget);
        ProgressSlider->setObjectName(QStringLiteral("ProgressSlider"));
        ProgressSlider->setMinimumSize(QSize(0, 80));
        ProgressSlider->setOrientation(Qt::Horizontal);
        ProgressSlider->setTickPosition(QSlider::TicksAbove);

        verticalLayout->addWidget(ProgressSlider);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 568, 22));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuAbout = new QMenu(menubar);
        menuAbout->setObjectName(QStringLiteral("menuAbout"));
        menutool = new QMenu(menubar);
        menutool->setObjectName(QStringLiteral("menutool"));
        MainWindow->setMenuBar(menubar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QStringLiteral("toolBar"));
#ifndef QT_NO_TOOLTIP
        toolBar->setToolTip(QStringLiteral(""));
#endif // QT_NO_TOOLTIP
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        statusBar->setLayoutDirection(Qt::RightToLeft);
        MainWindow->setStatusBar(statusBar);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName(QStringLiteral("dockWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dockWidget->sizePolicy().hasHeightForWidth());
        dockWidget->setSizePolicy(sizePolicy);
        dockWidget->setMinimumSize(QSize(106, 127));
        dockWidget->setStyleSheet(QStringLiteral("border:rgb(186, 189, 182);background:rgb(255, 255, 255)"));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        LidarcheckBox = new QCheckBox(dockWidgetContents);
        LidarcheckBox->setObjectName(QStringLiteral("LidarcheckBox"));

        verticalLayout_2->addWidget(LidarcheckBox);

        RadarcheckBox = new QCheckBox(dockWidgetContents);
        RadarcheckBox->setObjectName(QStringLiteral("RadarcheckBox"));

        verticalLayout_2->addWidget(RadarcheckBox);

        InscheckBox = new QCheckBox(dockWidgetContents);
        InscheckBox->setObjectName(QStringLiteral("InscheckBox"));

        verticalLayout_2->addWidget(InscheckBox);

        CameracheckBox = new QCheckBox(dockWidgetContents);
        CameracheckBox->setObjectName(QStringLiteral("CameracheckBox"));

        verticalLayout_2->addWidget(CameracheckBox);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(1), dockWidget);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menutool->menuAction());
        menubar->addAction(menuAbout->menuAction());
        menuFile->addAction(actionFileOpen);
        menuAbout->addAction(actionInfo);
        menutool->addAction(actionpanel);
        menutool->addAction(actiontool_bar);
        toolBar->addAction(actionFileOpen);
        toolBar->addAction(actionPreFrame);
        toolBar->addAction(actionStop);
        toolBar->addAction(actionEnd);
        toolBar->addAction(actionNextFrame);
        toolBar->addAction(actionpanel);
        toolBar->addSeparator();

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        actionFileOpen->setText(QApplication::translate("MainWindow", "FileOpen", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionFileOpen->setToolTip(QApplication::translate("MainWindow", "open file", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_SHORTCUT
        actionFileOpen->setShortcut(QApplication::translate("MainWindow", "Ctrl+F", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        actionNextFrame->setText(QApplication::translate("MainWindow", "NextFrame", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionNextFrame->setToolTip(QApplication::translate("MainWindow", "next frame", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionPreFrame->setText(QApplication::translate("MainWindow", "PreFrame", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionPreFrame->setToolTip(QApplication::translate("MainWindow", "previoud frame", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionStop->setText(QApplication::translate("MainWindow", "Stop", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionStop->setToolTip(QApplication::translate("MainWindow", "start/stop", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_SHORTCUT
        actionStop->setShortcut(QApplication::translate("MainWindow", "Ctrl+D", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        actionEnd->setText(QApplication::translate("MainWindow", "End", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionEnd->setToolTip(QApplication::translate("MainWindow", "end", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionpanel->setText(QApplication::translate("MainWindow", "Panel", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionpanel->setToolTip(QApplication::translate("MainWindow", "open panel", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionInfo->setText(QApplication::translate("MainWindow", "Info", Q_NULLPTR));
        actiontool_bar->setText(QApplication::translate("MainWindow", "Tool bar", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", Q_NULLPTR));
        menuAbout->setTitle(QApplication::translate("MainWindow", "About", Q_NULLPTR));
        menutool->setTitle(QApplication::translate("MainWindow", "Tool", Q_NULLPTR));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "tool bar", Q_NULLPTR));
        dockWidget->setWindowTitle(QApplication::translate("MainWindow", "sensor check", Q_NULLPTR));
        LidarcheckBox->setText(QApplication::translate("MainWindow", "Lidar", Q_NULLPTR));
        RadarcheckBox->setText(QApplication::translate("MainWindow", "Radar", Q_NULLPTR));
        InscheckBox->setText(QApplication::translate("MainWindow", "Ins", Q_NULLPTR));
        CameracheckBox->setText(QApplication::translate("MainWindow", "Camera", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
