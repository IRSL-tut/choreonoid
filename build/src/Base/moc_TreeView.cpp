/****************************************************************************
** Meta object code from reading C++ file 'TreeView.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.12)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/Base/TreeView.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'TreeView.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.12. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cnoid__TreeView_t {
    QByteArrayData data[12];
    char stringdata0[134];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cnoid__TreeView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cnoid__TreeView_t qt_meta_stringdata_cnoid__TreeView = {
    {
QT_MOC_LITERAL(0, 0, 15), // "cnoid::TreeView"
QT_MOC_LITERAL(1, 16, 11), // "onCollapsed"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 11), // "QModelIndex"
QT_MOC_LITERAL(4, 41, 5), // "index"
QT_MOC_LITERAL(5, 47, 10), // "onExpanded"
QT_MOC_LITERAL(6, 58, 11), // "onActivated"
QT_MOC_LITERAL(7, 70, 9), // "onClicked"
QT_MOC_LITERAL(8, 80, 15), // "onDoubleClicked"
QT_MOC_LITERAL(9, 96, 9), // "onEntered"
QT_MOC_LITERAL(10, 106, 9), // "onPressed"
QT_MOC_LITERAL(11, 116, 17) // "onViewportEntered"

    },
    "cnoid::TreeView\0onCollapsed\0\0QModelIndex\0"
    "index\0onExpanded\0onActivated\0onClicked\0"
    "onDoubleClicked\0onEntered\0onPressed\0"
    "onViewportEntered"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cnoid__TreeView[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x08 /* Private */,
       5,    1,   57,    2, 0x08 /* Private */,
       6,    1,   60,    2, 0x08 /* Private */,
       7,    1,   63,    2, 0x08 /* Private */,
       8,    1,   66,    2, 0x08 /* Private */,
       9,    1,   69,    2, 0x08 /* Private */,
      10,    1,   72,    2, 0x08 /* Private */,
      11,    0,   75,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

       0        // eod
};

void cnoid::TreeView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TreeView *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onCollapsed((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 1: _t->onExpanded((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 2: _t->onActivated((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 3: _t->onClicked((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 4: _t->onDoubleClicked((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 5: _t->onEntered((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 6: _t->onPressed((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 7: _t->onViewportEntered(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject cnoid::TreeView::staticMetaObject = { {
    &QTreeView::staticMetaObject,
    qt_meta_stringdata_cnoid__TreeView.data,
    qt_meta_data_cnoid__TreeView,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *cnoid::TreeView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cnoid::TreeView::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cnoid__TreeView.stringdata0))
        return static_cast<void*>(this);
    return QTreeView::qt_metacast(_clname);
}

int cnoid::TreeView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTreeView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
