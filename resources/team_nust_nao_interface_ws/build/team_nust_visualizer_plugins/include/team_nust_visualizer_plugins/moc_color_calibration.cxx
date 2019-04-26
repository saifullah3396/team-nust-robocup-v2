/****************************************************************************
** Meta object code from reading C++ file 'color_calibration.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/team_nust_visualizer_plugins/include/team_nust_visualizer_plugins/color_calibration.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'color_calibration.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_team_nust_visualizer_plugins__ColorCalibration[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      48,   47,   47,   47, 0x0a,
      70,   47,   47,   47, 0x0a,
      85,   47,   47,   47, 0x0a,
     100,   47,   47,   47, 0x0a,
     120,   47,   47,   47, 0x0a,
     140,   47,   47,   47, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_team_nust_visualizer_plugins__ColorCalibration[] = {
    "team_nust_visualizer_plugins::ColorCalibration\0"
    "\0updateConfiguration()\0updateCamera()\0"
    "updateLayout()\0updateTableLayout()\0"
    "saveConfiguration()\0publishSettings()\0"
};

void team_nust_visualizer_plugins::ColorCalibration::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ColorCalibration *_t = static_cast<ColorCalibration *>(_o);
        switch (_id) {
        case 0: _t->updateConfiguration(); break;
        case 1: _t->updateCamera(); break;
        case 2: _t->updateLayout(); break;
        case 3: _t->updateTableLayout(); break;
        case 4: _t->saveConfiguration(); break;
        case 5: _t->publishSettings(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData team_nust_visualizer_plugins::ColorCalibration::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject team_nust_visualizer_plugins::ColorCalibration::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_team_nust_visualizer_plugins__ColorCalibration,
      qt_meta_data_team_nust_visualizer_plugins__ColorCalibration, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &team_nust_visualizer_plugins::ColorCalibration::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *team_nust_visualizer_plugins::ColorCalibration::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *team_nust_visualizer_plugins::ColorCalibration::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_team_nust_visualizer_plugins__ColorCalibration))
        return static_cast<void*>(const_cast< ColorCalibration*>(this));
    typedef rviz::Panel QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int team_nust_visualizer_plugins::ColorCalibration::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rviz::Panel QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
