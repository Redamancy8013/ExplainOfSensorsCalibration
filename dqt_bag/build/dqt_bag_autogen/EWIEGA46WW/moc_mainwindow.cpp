/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[22];
    char stringdata0[386];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 10), // "timeUpdate"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 14), // "progressUpdate"
QT_MOC_LITERAL(4, 38, 27), // "on_actionFileOpen_triggered"
QT_MOC_LITERAL(5, 66, 23), // "on_actionStop_triggered"
QT_MOC_LITERAL(6, 90, 22), // "on_actionEnd_triggered"
QT_MOC_LITERAL(7, 113, 24), // "on_actionpanel_triggered"
QT_MOC_LITERAL(8, 138, 31), // "on_dockWidget_visibilityChanged"
QT_MOC_LITERAL(9, 170, 7), // "visible"
QT_MOC_LITERAL(10, 178, 28), // "on_actionNextFrame_triggered"
QT_MOC_LITERAL(11, 207, 27), // "on_actionPreFrame_triggered"
QT_MOC_LITERAL(12, 235, 23), // "on_actionInfo_triggered"
QT_MOC_LITERAL(13, 259, 27), // "on_actiontool_bar_triggered"
QT_MOC_LITERAL(14, 287, 14), // "SlotLidarState"
QT_MOC_LITERAL(15, 302, 5), // "state"
QT_MOC_LITERAL(16, 308, 14), // "SlotRadarState"
QT_MOC_LITERAL(17, 323, 12), // "SlotImuState"
QT_MOC_LITERAL(18, 336, 12), // "SlotSeqValue"
QT_MOC_LITERAL(19, 349, 5), // "value"
QT_MOC_LITERAL(20, 355, 13), // "SlotTimelabel"
QT_MOC_LITERAL(21, 369, 16) // "SlotSliderUpdate"

    },
    "MainWindow\0timeUpdate\0\0progressUpdate\0"
    "on_actionFileOpen_triggered\0"
    "on_actionStop_triggered\0on_actionEnd_triggered\0"
    "on_actionpanel_triggered\0"
    "on_dockWidget_visibilityChanged\0visible\0"
    "on_actionNextFrame_triggered\0"
    "on_actionPreFrame_triggered\0"
    "on_actionInfo_triggered\0"
    "on_actiontool_bar_triggered\0SlotLidarState\0"
    "state\0SlotRadarState\0SlotImuState\0"
    "SlotSeqValue\0value\0SlotTimelabel\0"
    "SlotSliderUpdate"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   99,    2, 0x06 /* Public */,
       3,    1,  102,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,  105,    2, 0x08 /* Private */,
       5,    0,  106,    2, 0x08 /* Private */,
       6,    0,  107,    2, 0x08 /* Private */,
       7,    0,  108,    2, 0x08 /* Private */,
       8,    1,  109,    2, 0x08 /* Private */,
      10,    0,  112,    2, 0x08 /* Private */,
      11,    0,  113,    2, 0x08 /* Private */,
      12,    0,  114,    2, 0x08 /* Private */,
      13,    0,  115,    2, 0x08 /* Private */,
      14,    1,  116,    2, 0x08 /* Private */,
      16,    1,  119,    2, 0x08 /* Private */,
      17,    1,  122,    2, 0x08 /* Private */,
      18,    1,  125,    2, 0x08 /* Private */,
      20,    1,  128,    2, 0x08 /* Private */,
      21,    1,  131,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::Int,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Int,   19,
    QMetaType::Void, QMetaType::QString,   19,
    QMetaType::Void, QMetaType::Int,   19,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->timeUpdate((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->progressUpdate((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->on_actionFileOpen_triggered(); break;
        case 3: _t->on_actionStop_triggered(); break;
        case 4: _t->on_actionEnd_triggered(); break;
        case 5: _t->on_actionpanel_triggered(); break;
        case 6: _t->on_dockWidget_visibilityChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_actionNextFrame_triggered(); break;
        case 8: _t->on_actionPreFrame_triggered(); break;
        case 9: _t->on_actionInfo_triggered(); break;
        case 10: _t->on_actiontool_bar_triggered(); break;
        case 11: _t->SlotLidarState((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->SlotRadarState((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->SlotImuState((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->SlotSeqValue((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->SlotTimelabel((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 16: _t->SlotSliderUpdate((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (MainWindow::*_t)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::timeUpdate)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::progressUpdate)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::timeUpdate(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MainWindow::progressUpdate(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
