/****************************************************************************
** Meta object code from reading C++ file 'ItemSelectionModel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.12)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/Base/ItemSelectionModel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ItemSelectionModel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.12. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cnoid__ItemSelectionModel_t {
    QByteArrayData data[12];
    char stringdata0[168];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__ItemSelectionModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__ItemSelectionModel_t qt_meta_stringdata_cnoid__ItemSelectionModel = {
    {
QT_MOC_LITERAL(0, 0, 25), // "cnoid::ItemSelectionModel"
QT_MOC_LITERAL(1, 26, 16), // "onCurrentChanged"
QT_MOC_LITERAL(2, 43, 0), // ""
QT_MOC_LITERAL(3, 44, 11), // "QModelIndex"
QT_MOC_LITERAL(4, 56, 5), // "index"
QT_MOC_LITERAL(5, 62, 8), // "previous"
QT_MOC_LITERAL(6, 71, 22), // "onCurrentColumnChanged"
QT_MOC_LITERAL(7, 94, 19), // "onCurrentRowChanged"
QT_MOC_LITERAL(8, 114, 18), // "onSelectionChanged"
QT_MOC_LITERAL(9, 133, 14), // "QItemSelection"
QT_MOC_LITERAL(10, 148, 8), // "selected"
QT_MOC_LITERAL(11, 157, 10) // "deselected"

    },
    "cnoid::ItemSelectionModel\0onCurrentChanged\0"
    "\0QModelIndex\0index\0previous\0"
    "onCurrentColumnChanged\0onCurrentRowChanged\0"
    "onSelectionChanged\0QItemSelection\0"
    "selected\0deselected"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__ItemSelectionModel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   34,    2, 0x08 /* Private */,
       6,    2,   39,    2, 0x08 /* Private */,
       7,    2,   44,    2, 0x08 /* Private */,
       8,    2,   49,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3,    4,    5,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3,    4,    5,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3,    4,    5,
    QMetaType::Void, 0x80000000 | 9, 0x80000000 | 9,   10,   11,

       0        // eod
};

void cnoid::ItemSelectionModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ItemSelectionModel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onCurrentChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2]))); break;
        case 1: _t->onCurrentColumnChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2]))); break;
        case 2: _t->onCurrentRowChanged((*reinterpret_cast< const QModelIndex(*)>(_a[1])),(*reinterpret_cast< const QModelIndex(*)>(_a[2]))); break;
        case 3: _t->onSelectionChanged((*reinterpret_cast< const QItemSelection(*)>(_a[1])),(*reinterpret_cast< const QItemSelection(*)>(_a[2]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject cnoid::ItemSelectionModel::staticMetaObject = { {
    &QItemSelectionModel::staticMetaObject,
    qt_meta_stringdata_cnoid__ItemSelectionModel.data,
    qt_meta_data_cnoid__ItemSelectionModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *cnoid::ItemSelectionModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::ItemSelectionModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__ItemSelectionModel.stringdata0))
        return static_cast<void*>(this);
    return QItemSelectionModel::qt_metacast(_clname);
}

int cnoid::ItemSelectionModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QItemSelectionModel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
