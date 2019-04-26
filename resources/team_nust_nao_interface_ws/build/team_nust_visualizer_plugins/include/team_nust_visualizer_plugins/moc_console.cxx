/****************************************************************************
** Meta object code from reading C++ file 'console.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/team_nust_visualizer_plugins/include/team_nust_visualizer_plugins/console.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'console.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_team_nust_visualizer_plugins__Console[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      45,   39,   38,   38, 0x0a,
      63,   38,   38,   38, 0x09,
      80,   38,   38,   38, 0x09,
      94,   38,   38,   38, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_team_nust_visualizer_plugins__Console[] = {
    "team_nust_visualizer_plugins::Console\0"
    "\0topic\0setTopic(QString)\0displayMessage()\0"
    "updateTopic()\0publishCommand()\0"
};

void team_nust_visualizer_plugins::Console::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Console *_t = static_cast<Console *>(_o);
        switch (_id) {
        case 0: _t->setTopic((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->displayMessage(); break;
        case 2: _t->updateTopic(); break;
        case 3: _t->publishCommand(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData team_nust_visualizer_plugins::Console::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject team_nust_visualizer_plugins::Console::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_team_nust_visualizer_plugins__Console,
      qt_meta_data_team_nust_visualizer_plugins__Console, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &team_nust_visualizer_plugins::Console::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *team_nust_visualizer_plugins::Console::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *team_nust_visualizer_plugins::Console::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_team_nust_visualizer_plugins__Console))
        return static_cast<void*>(const_cast< Console*>(this));
    typedef rviz::Panel QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int team_nust_visualizer_plugins::Console::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rviz::Panel QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
