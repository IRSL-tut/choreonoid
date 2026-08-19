#include "ViewArea.h"
#include "View.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "MessageView.h"
#include "MainWindow.h"
#include "Timer.h"
#include "QtEventUtil.h"
#include <QApplication>
#include <QBoxLayout>
#include <QSplitter>
#include <QTabWidget>
#include <QTabBar>
#include <QRubberBand>
#include <QMouseEvent>
#include <QScreen>
#include <QLabel>
#include <QDrag>
#include <QMimeData>
#include <QPainter>
#include <QCursor>
#include <QGuiApplication>
#include <QWindow>
#include <QPointer>

#include <memory>
#include <bitset>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

enum DropArea { OVER = -1, LEFT = 0, TOP, RIGHT, BOTTOM, NUM_DROP_AREAS };
const int SPLIT_DISTANCE_THRESHOLD = 35;
const int OUTER_EDGE_DISTANCE_THRESHOLD = 8;
const int OUTSIDE_RUBBER_BAND_OFFSET = 8;

/**
   The MIME type used to identify the drag operation of a view tab. The drag operation is
   processed by the ViewArea objects themselves and the MIME data is not actually used, so
   the data attached to this type is always empty.
*/
const char* viewDragMimeType = "application/x-choreonoid-view";

vector<ViewArea*> viewAreas;
bool isBeforeDoingInitialLayout = true;

bool isWaylandPlatform()
{
    static const bool isWayland =
        QGuiApplication::platformName().startsWith(QLatin1String("wayland"));
    return isWayland;
}

/**
   Wayland does not tell a client application where its windows are located on the screen,
   and the functions that return global positions such as QWidget::mapToGlobal and
   QCursor::pos just return meaningless values on it. The view drag operation itself is
   implemented without using global positions so that it works on any platform, but the
   preview of a window created by dropping a view outside the existing windows can only be
   displayed on the platforms where the global positions are available.
*/
bool isGlobalPositionAvailable()
{
    return !isWaylandPlatform();
}

/**
   Wayland does not have the concept of a primary output. QGuiApplication::primaryScreen
   just returns one of the outputs regardless of the primary display specified in the
   desktop environment, and the returned screen is not meaningful on it.
*/
bool isPrimaryScreenAvailable()
{
    return !isWaylandPlatform();
}


/**
   This function returns the screen that is assumed to be the screen of an independent view
   area window when the window does not have its own screen information in a project. The
   screen of such a window is stored in a project only when it is different from this screen,
   so that a layout in which all the windows are on the same screen does not depend on the
   screen configuration.

   The primary screen is used for this purpose as long as it is available. On Wayland, where
   the primary screen does not correspond to the primary display of the desktop environment,
   the screen of the main window is used instead.
*/
QScreen* referenceScreen()
{
    if(!isPrimaryScreenAvailable()){
        if(auto mainWindow = MainWindow::instance()){
            if(auto windowHandle = mainWindow->windowHandle()){
                if(auto screen = windowHandle->screen()){
                    return screen;
                }
            }
        }
    }
    return QGuiApplication::primaryScreen();
}

class TabWidget : public QTabWidget
{
public:
    TabWidget(QWidget* parent) : QTabWidget(parent) { }
    QTabBar* tabBar() { return QTabWidget::tabBar(); }
};
    

class ViewPane : public QWidget
{
public:
    ViewPane(ViewArea::Impl* viewAreaImpl, QWidget* parent = nullptr);
    int addView(View* view);
    void onViewTitleChanged(const QString &title);
    void removeView(View* view);
    void setTabVisible(bool on);
    void makeDirect();

    QTabBar* tabBar() { return tabWidget->tabBar(); }
    const QTabBar* tabBar() const { return tabWidget->tabBar(); }
    int currentIndex() const {
        return directView ? 0 : tabWidget->currentIndex();
    }
    void setCurrentIndex(int index) {
        if(!directView){
            tabWidget->setCurrentIndex(index);
        }
    }
    View* currentView() const {
        return directView ? directView : static_cast<View*>(tabWidget->currentWidget());
    }
    int count() const {
        return directView ? 1 : tabWidget->count();
    }
    View* view(int index) const {
        if(directView){
            if(index == 0){
                return directView;
            }
            return nullptr;
        } else {
            return static_cast<View*>(tabWidget->widget(index));
        }
    }
    
    virtual bool eventFilter(QObject* object, QEvent* event);

    virtual QSize minimumSizeHint () const {
        QSize s = QWidget::minimumSizeHint();
        if(!tabBar()->isVisible()){
            s.rheight() -= tabBar()->minimumSizeHint().height();
        }
        return s;
    }

    TabWidget* tabWidget;
    QVBoxLayout* vbox;
    View* directView;
    ViewArea::Impl* viewAreaImpl;
};

}

namespace cnoid {

class ViewArea::Impl
{
public:
    Impl(ViewArea* self);
    ~Impl();

    ViewArea* self;
    QVBoxLayout* vbox;
    QSplitter* topSplitter;

    int numViews;
    bool viewTabsVisible;
    bool isMaximizedBeforeFullScreen;
    bool needToUpdateDefaultPaneAreas;

    ViewPane* areaToPane[View::NumLayoutAreas];

    struct AreaDetectionInfo {
        AreaDetectionInfo() {
            for(int i=0; i < View::NumLayoutAreas; ++i){
                scores[i] = 0;
            }
        }
        ViewPane* pane;
        int scores[View::NumLayoutAreas];
    };

    typedef bitset<NUM_DROP_AREAS> EdgeContactState;

    QPoint tabDragStartPosition;
    bool isViewDragging;
    bool isViewDropAccepted;
    bool isViewDragCanceled;
    View* draggedView;
    QSize draggedViewWindowSize;
    ViewArea* dragDestViewArea;
    ViewPane* dragSrcPane;
    ViewPane* dragDestPane;
    bool isViewDraggingOnOuterEdge;
    int dropEdge;
    QRubberBand* rubberBand;
    QRubberBand* outsideRubberBand;
    Timer outsideRubberBandTimer;

    vector<QLabel*> viewSizeLabels;
    Timer viewSizeLabelTimer;
    
    MenuManager viewMenuManager;

    void setSingleView(View* view);
    void createDefaultPanes();

    void detectExistingPaneAreas();
    ViewPane* updateAreaDetectionInfos(QSplitter* splitter, const EdgeContactState& edge, vector<AreaDetectionInfo>& infos);
    void setBestAreaMatchPane(vector<AreaDetectionInfo>& infos, View::LayoutArea area, ViewPane* firstPane);

    void setViewTabsVisible(QSplitter* splitter, bool on);
    void setFullScreen(bool on);
    bool addView(View* view);
    void addView(ViewPane* pane, View* view, bool makeCurrent);
    bool removeView(View* view);
    bool removeView(ViewPane* pane, View* view, bool isMovingInViewArea);
    View* findFirstView(QSplitter* splitter);

    void getVisibleViews(vector<View*>& out_views, QSplitter* splitter = nullptr);
    void getVisibleViewsIter(QSplitter* splitter, vector<View*>& out_views);
    void showViewSizeLabels(QSplitter* splitter);
    void hideViewSizeLabels();
    
    bool viewTabMousePressEvent(ViewPane* pane, QMouseEvent* event);
    bool viewTabMouseMoveEvent(ViewPane* pane, QMouseEvent* event);

    void startViewDrag(ViewPane* pane, View* view);
    QPixmap createViewDragPixmap(View* view);
    ViewArea* findDragDestViewArea(QWindow* window, const QPoint& posInWindow, QPoint& out_posInViewArea);
    void updateDragDestination(ViewArea* viewArea, const QPoint& posInViewArea);
    bool onViewDragMoveEvent(QWindow* window, QDragMoveEvent* event);
    bool onViewDragLeaveEvent();
    bool onViewDropEvent(QWindow* window, QDropEvent* event);
    void updateOutsideRubberBand();
    void finishViewDrag();

    void showRectangle(QRect r);
    void dragView(const QPoint& posInDestViewArea);
    void dragViewInsidePane(const QPoint& posInDestPane);
    void dropViewInsidePane(ViewPane* pane, View* view, int dropEdge);
    void dragViewOnOuterEdge();
    void dropViewToOuterEdge(View* view);
    void dropViewOutside();
    void separateView(View* view);
    void separateView(View* view, const QPoint& pos, const QSize& size);
    void clearAllPanes();
    void clearAllPanesSub(QSplitter* splitter);
    void getAllViews(vector<View*>& out_views);
    void getAllViewsSub(QSplitter* splitter, vector<View*>& out_views);

    void storeLayout(Archive* archive);
    MappingPtr storeSplitterState(QSplitter* splitter, Archive* archive);
    MappingPtr storePaneState(ViewPane* pane, Archive* archive);
    void restoreLayout(Archive* archive);
    QWidget* restoreViewContainer(const Mapping& state, Archive* archive);
    QWidget* restoreSplitter(const Mapping& state, Archive* archive);
    ViewPane* restorePane(const Mapping& state, Archive* archive);
    void resetLayout();

    void removePaneIfEmpty(ViewPane* pane);
    void removePaneSub(QWidget* widgetToRemove, QWidget* widgetToRaise);
    void clearEmptyPanes();
    QWidget* clearEmptyPanesSub(QSplitter* splitter);
};

}


namespace {

class CustomSplitter : public QSplitter
{
public:
    ViewArea::Impl* viewAreaImpl;
    bool defaultOpaqueResize;
    
    CustomSplitter(ViewArea::Impl* viewAreaImpl, QWidget* parent = nullptr)
        : QSplitter(parent),
          viewAreaImpl(viewAreaImpl)
    {
        defaultOpaqueResize = opaqueResize();
    }

    CustomSplitter(ViewArea::Impl* viewAreaImpl, Qt::Orientation orientation, QWidget* parent = nullptr)
        : QSplitter(orientation, parent),
          viewAreaImpl(viewAreaImpl)
    {
        defaultOpaqueResize = opaqueResize();
    }

    bool moveSplitterPosition(int d)
    {
        QList<int> s = sizes();
        if(s.size() >= 2){
            s[0] += d;
            s[1] -= d;
        }
        if(s[0] >= 0 && s[1] >= 0){
            setSizes(s);
        }
        return true;
    }

    virtual QSplitterHandle* createHandle() override;
};


class CustomSplitterHandle : public QSplitterHandle
{
    CustomSplitter* splitter;
    bool isDragging;

public:
    CustomSplitterHandle(CustomSplitter* splitter)
        : QSplitterHandle(splitter->orientation(), splitter),
          splitter(splitter)
    {
        setFocusPolicy(Qt::WheelFocus);
        isDragging = false;
    }

    virtual void mousePressEvent(QMouseEvent* event) override
    {
        if(event->button() == Qt::LeftButton){
            isDragging = true;
            splitter->viewAreaImpl->showViewSizeLabels(splitter);
            if(event->modifiers() & Qt::ShiftModifier){
                splitter->setOpaqueResize(!splitter->defaultOpaqueResize);
            } else {
                splitter->setOpaqueResize(splitter->defaultOpaqueResize);
            }
        }

        QSplitterHandle::mousePressEvent(event);
    }

    virtual void mouseMoveEvent(QMouseEvent* event) override
    {
        QSplitterHandle::mouseMoveEvent(event);

        if(isDragging){
            splitter->viewAreaImpl->showViewSizeLabels(splitter);
        }
    }
    
    virtual void mouseReleaseEvent(QMouseEvent* event) override
    {
        QSplitterHandle::mouseReleaseEvent(event);

        isDragging = false;
        splitter->setOpaqueResize(splitter->defaultOpaqueResize);
    }

    virtual void keyPressEvent(QKeyEvent* event) override
    {
        bool processed = false;
        int r = 1;
        if(event->modifiers() & Qt::ShiftModifier){
            r = 10;
        }
        int key = event->key();
        if(orientation() == Qt::Horizontal){
            if(key == Qt::Key_Left){
                processed = splitter->moveSplitterPosition(-r);
            } else if(key == Qt::Key_Right){
                processed = splitter->moveSplitterPosition( r);
            }
        } else {
            if(key == Qt::Key_Up){
                processed = splitter->moveSplitterPosition(-r);
            } else if(key == Qt::Key_Down){
                processed = splitter->moveSplitterPosition( r);
            }
        }
        if(processed){
            splitter->viewAreaImpl->showViewSizeLabels(splitter);
        } else {
            QSplitterHandle::keyPressEvent(event);
        }
    }
};


QSplitterHandle* CustomSplitter::createHandle()
{
    return new CustomSplitterHandle(this);
}


ViewPane::ViewPane(ViewArea::Impl* viewAreaImpl, QWidget* parent)
    : QWidget(parent),
      viewAreaImpl(viewAreaImpl)
{
    vbox = new QVBoxLayout(this);
    vbox->setSpacing(0);
    vbox->setContentsMargins(0, 0, 0, 0);

    tabWidget = new TabWidget(this);
    tabWidget->setMovable(true);
    tabWidget->setUsesScrollButtons(true);
    tabWidget->tabBar()->installEventFilter(this);

    if(!viewAreaImpl->viewTabsVisible){
        tabWidget->tabBar()->hide();
    }
    
    vbox->addWidget(tabWidget);

    directView = nullptr;
}


int ViewPane::addView(View* view)
{
    int index = 0;
    if(viewAreaImpl->viewTabsVisible){
        index = tabWidget->addTab(view, view->windowTitle());
    } else {
        if(viewAreaImpl->self->isWindow() && tabWidget->count() == 0 && !directView){
            tabWidget->hide();
            directView = view;
            directView->setParent(this);
            vbox->addWidget(directView);
            directView->show();
        } else {
            tabWidget->show();
            if(directView){
                tabWidget->addTab(directView, directView->windowTitle());
                directView = nullptr;
            }
            index = tabWidget->addTab(view, view->windowTitle());
        }
    }
    connect(view, &QWidget::windowTitleChanged, this, &ViewPane::onViewTitleChanged);
    return index;
}


/**
   \todo Implement this in View.cpp. See the View::bringToFront function.
*/
void ViewPane::onViewTitleChanged(const QString &title)
{
    if(auto view = dynamic_cast<View*>(sender())){
        int index = tabWidget->indexOf(view);
        tabWidget->setTabText(index, view->windowTitle());
    }
}


void ViewPane::removeView(View* view)
{
    if(view == directView){
        vbox->removeWidget(directView);
        directView = nullptr;
    } else {
        tabWidget->removeTab(tabWidget->indexOf(view));
        if(viewAreaImpl->self->isWindow() && tabWidget->count() == 1 && !viewAreaImpl->viewTabsVisible){
            makeDirect();
        }
    }
    disconnect(view, &QWidget::windowTitleChanged, this, &ViewPane::onViewTitleChanged);
}


void ViewPane::setTabVisible(bool on)
{
    if(on){
        if(directView){
            tabWidget->addTab(directView, directView->windowTitle());
            directView = nullptr;
            tabWidget->show();
        }
        tabBar()->show();
    } else {
        if(!directView){
            if(viewAreaImpl->self->isWindow() && tabWidget->count() == 1){
                makeDirect();
            } else {
                tabBar()->hide();
            }
        }
    }
}


void ViewPane::makeDirect()
{
    if(!directView && tabWidget->count() == 1){
        directView = static_cast<View*>(tabWidget->widget(0));
        tabWidget->removeTab(0);
        directView->setParent(this);
        directView->show();
        vbox->addWidget(directView);
        tabWidget->hide();
    }
}


bool ViewPane::eventFilter(QObject* object, QEvent* event)
{
    if(object == tabWidget->tabBar()){
        switch(event->type()){
        case QEvent::MouseButtonPress:
            return viewAreaImpl->viewTabMousePressEvent(this, static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonDblClick:
            break;
        case QEvent::MouseMove:
            return viewAreaImpl->viewTabMouseMoveEvent(this, static_cast<QMouseEvent*>(event));
        default:
            break;
        }
    }
    return false;
}

}


ViewArea::ViewArea(QWidget* parent)
    : QWidget(parent)
{
    if(!parent){
        //setParent(MainWindow::instance());
        //setWindowFlags(Qt::Tool);
        setAttribute(Qt::WA_DeleteOnClose);
    }

    // This is necessary to receive the drag events of a view tab dragged from any view area
    setAcceptDrops(true);

    impl = new Impl(this);
}


ViewArea::Impl::Impl(ViewArea* self)
    : self(self)
{
    numViews = 0;
    viewTabsVisible = true;
    isMaximizedBeforeFullScreen = false;
    isViewDragging = false;
    isViewDropAccepted = false;
    isViewDragCanceled = false;
    draggedView = nullptr;
    dragSrcPane = nullptr;
    dragDestViewArea = nullptr;
    dragDestPane = nullptr;
    isViewDraggingOnOuterEdge = false;
    dropEdge = OVER;

    vbox = new QVBoxLayout(self);
    vbox->setSpacing(0);
    vbox->setContentsMargins(0, 0, 0, 0);

    rubberBand = new QRubberBand(QRubberBand::Rectangle, self);
    rubberBand->hide();

    // The outside rubber band is a top level window that must be placed at the global
    // position of the pointer, so that it cannot be used on the platforms that do not
    // provide the global positions such as Wayland. It is not created at all on such a
    // platform, and the null pointer means that the outline of a separated window is not
    // displayed during a drag operation.
    if(!isGlobalPositionAvailable()){
        outsideRubberBand = nullptr;
    } else {
        outsideRubberBand = new QRubberBand(QRubberBand::Rectangle);
        outsideRubberBand->setWindowFlags(
            Qt::ToolTip | Qt::FramelessWindowHint | Qt::WindowTransparentForInput |
            Qt::WindowDoesNotAcceptFocus);
        outsideRubberBand->hide();

        outsideRubberBandTimer.setInterval(30);
        outsideRubberBandTimer.sigTimeout().connect([this](){ updateOutsideRubberBand(); });
    }

    viewSizeLabelTimer.setSingleShot(true);
    viewSizeLabelTimer.setInterval(1000);
    viewSizeLabelTimer.sigTimeout().connect([&](){ hideViewSizeLabels(); });

    topSplitter = nullptr;
    needToUpdateDefaultPaneAreas = true;

    viewAreas.push_back(self);
}


ViewArea::~ViewArea()
{
    viewAreas.erase(std::find(viewAreas.begin(), viewAreas.end(), this));
    delete impl;
}


ViewArea::Impl::~Impl()
{
    clearAllPanes();
    delete rubberBand;
    delete outsideRubberBand;
}


void ViewArea::setSingleView(View* view)
{
    impl->setSingleView(view);
}


void ViewArea::Impl::setSingleView(View* view)
{
    clearAllPanes();

    viewTabsVisible = false;
    
    topSplitter = new CustomSplitter(this, self);
    vbox->addWidget(topSplitter);

    ViewPane* pane = new ViewPane(this, topSplitter);
    topSplitter->addWidget(pane);
    for(int i=0; i < View::NumLayoutAreas; ++i){
        areaToPane[i] = pane;
    }
    needToUpdateDefaultPaneAreas = false;

    addView(pane, view, true);
}


void ViewArea::createDefaultPanes()
{
    impl->createDefaultPanes();
}


void ViewArea::Impl::createDefaultPanes()
{
    clearAllPanes();

    topSplitter = new CustomSplitter(this, self);
    vbox->addWidget(topSplitter);
    topSplitter->setOrientation(Qt::Horizontal);
    
    auto vSplitter0 = new CustomSplitter(this, Qt::Vertical, topSplitter);
    topSplitter->addWidget(vSplitter0);
    
    auto topLeftPane = new ViewPane(this, vSplitter0);
    vSplitter0->addWidget(topLeftPane);

    auto bottomLeftPane = new ViewPane(this, vSplitter0);
    vSplitter0->addWidget(bottomLeftPane);
    
    auto vSplitter1 = new CustomSplitter(this, Qt::Vertical, topSplitter);
    topSplitter->addWidget(vSplitter1);

    auto hSplitter1 = new CustomSplitter(this, Qt::Horizontal, vSplitter1);
    vSplitter1->addWidget(hSplitter1);

    auto bottomCenterPane = new ViewPane(this, vSplitter1);
    vSplitter1->addWidget(bottomCenterPane);

    auto centerPane = new ViewPane(this, hSplitter1);
    hSplitter1->addWidget(centerPane);

    auto rightPane = new ViewPane(this, hSplitter1);
    hSplitter1->addWidget(rightPane);

    areaToPane[View::TopLeftArea] = topLeftPane;
    areaToPane[View::MiddleLeftArea] = topLeftPane;
    areaToPane[View::BottomLeftArea] = bottomLeftPane;
    areaToPane[View::TopCenterArea] = centerPane;
    areaToPane[View::CenterArea] = centerPane;
    areaToPane[View::BottomCenterArea] = bottomCenterPane;
    areaToPane[View::TopRightArea] = rightPane;
    areaToPane[View::MiddleRightArea] = rightPane;
    areaToPane[View::BottomRightArea] = rightPane;

    QList<int> sizes;
    sizes << 100 << 600;
    topSplitter->setSizes(sizes);
    sizes.last() = 100;
    vSplitter0->setSizes(sizes);
    sizes.last() = 40;
    vSplitter1->setSizes(sizes);
    sizes.last() = 160;
    hSplitter1->setSizes(sizes);

    needToUpdateDefaultPaneAreas = false;
}


void ViewArea::Impl::detectExistingPaneAreas()
{
    for(int i=0; i < View::NumLayoutAreas; ++i){
        areaToPane[i] = nullptr;
    }

    vector<AreaDetectionInfo> infos;
    EdgeContactState edge;
    edge.set();

    ViewPane* firstPane = nullptr;
    if(topSplitter){
        firstPane = updateAreaDetectionInfos(topSplitter, edge, infos);
    }

    if(infos.empty()){
        /*
          No existing panes were found. This can happen when a project saved
          with all views hidden is restored, leaving topSplitter null.
          Create a single pane so that views can be added to it on demand.
        */
        clearAllPanes();
        topSplitter = new CustomSplitter(this, self);
        vbox->addWidget(topSplitter);
        auto pane = new ViewPane(this, topSplitter);
        topSplitter->addWidget(pane);
        for(int i=0; i < View::NumLayoutAreas; ++i){
            areaToPane[i] = pane;
        }

    } else {
        setBestAreaMatchPane(infos, View::CenterArea, firstPane);
        setBestAreaMatchPane(infos, View::TopLeftArea, firstPane);
        setBestAreaMatchPane(infos, View::BottomLeftArea, firstPane);
        setBestAreaMatchPane(infos, View::TopRightArea, firstPane);
        setBestAreaMatchPane(infos, View::BottomCenterArea, firstPane);

        areaToPane[View::MiddleLeftArea] = areaToPane[View::TopLeftArea];
        areaToPane[View::TopCenterArea] = areaToPane[View::CenterArea];
        areaToPane[View::MiddleRightArea] = areaToPane[View::TopRightArea];
        areaToPane[View::BottomRightArea] = areaToPane[View::TopRightArea];
    }

    needToUpdateDefaultPaneAreas = false;
}


/**
   @return The first found pane
*/
ViewPane* ViewArea::Impl::updateAreaDetectionInfos
(QSplitter* splitter, const EdgeContactState& edge, vector<AreaDetectionInfo>& infos)
{
    ViewPane* firstPane = nullptr;
    
    QWidget* childWidgets[2];
    childWidgets[0] = (splitter->count() > 0) ? splitter->widget(0) : 0;
    childWidgets[1] = (splitter->count() > 1) ? splitter->widget(1) : 0;
    bool isSingle = !(childWidgets[0] && childWidgets[1]);

    for(int i=0; i < 2; ++i){
        EdgeContactState currentEdge(edge);
        if(!isSingle){
            if(splitter->orientation() == Qt::Vertical){
                currentEdge.reset((i == 0) ? BOTTOM : TOP);
            } else {
                currentEdge.reset((i == 0) ? RIGHT : LEFT);
            }
        }
        if(childWidgets[i]){
            QSplitter* childSplitter = dynamic_cast<QSplitter*>(childWidgets[i]);
            if(childSplitter){
                ViewPane* pane = updateAreaDetectionInfos(childSplitter, currentEdge, infos);
                if(!firstPane){
                    firstPane = pane;
                }
            } else {
                ViewPane* pane = dynamic_cast<ViewPane*>(childWidgets[i]);
                if(pane){
                    AreaDetectionInfo info;
                    info.pane = pane;

                    // calculate scores for area matching
                    static const int offset = 100000000;
                    int width = pane->width();
                    int height = pane->height();

                    info.scores[View::CenterArea] = (4 - currentEdge.count()) * offset + width * height;

                    if(currentEdge.test(LEFT) && !currentEdge.test(RIGHT)){
                        info.scores[View::TopLeftArea] = offset + height;
                        info.scores[View::BottomLeftArea] = offset + height;
                        if(currentEdge.test(TOP) && !currentEdge.test(BOTTOM)){
                            info.scores[View::TopLeftArea] += 100;
                        } else if(currentEdge.test(BOTTOM) && !currentEdge.test(TOP)){
                            info.scores[View::BottomLeftArea] += 100;
                        }
                    }
                    if(currentEdge.test(RIGHT) && !currentEdge.test(LEFT)){
                        info.scores[View::TopRightArea] = offset + height;
                    }
                    if(currentEdge.test(BOTTOM) && !currentEdge.test(TOP)){
                        info.scores[View::BottomCenterArea] = offset + width;
                    }
                    
                    infos.push_back(info);

                    if(!firstPane){
                        firstPane = pane;
                    }
                }
            }
        }
    }
    return firstPane;
}


void ViewArea::Impl::setBestAreaMatchPane(vector<AreaDetectionInfo>& infos, View::LayoutArea area, ViewPane* firstPane)
{
    int topScore = 0;
    int topIndex = -1;

    for(int i=0; i < (signed)infos.size(); ++i){
        int s = infos[i].scores[area];
        if(s > topScore){
            topScore = s;
            topIndex = i;
        }
    }

    if(topIndex >= 0){
        areaToPane[area] = infos[topIndex].pane;
    } else {
        areaToPane[area] = firstPane;
    }
}


bool ViewArea::viewTabsVisible() const
{
    return impl->viewTabsVisible;
}


void ViewArea::setViewTabsVisible(bool on)
{
    if(impl->viewTabsVisible != on){
        if(impl->topSplitter){
            impl->setViewTabsVisible(impl->topSplitter, on);
        }
        impl->viewTabsVisible = on;
    }
}


void ViewArea::Impl::setViewTabsVisible(QSplitter* splitter, bool on)
{
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            setViewTabsVisible(childSplitter, on);
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane){
                pane->setTabVisible(on);
            }
        }
    }
}


void ViewArea::Impl::setFullScreen(bool on)
{
    if(self->isWindow()){
        if(on){
            if(!self->isFullScreen()){
                isMaximizedBeforeFullScreen = self->windowState() & Qt::WindowMaximized;
                self->showFullScreen();
            }
        } else if(self->isFullScreen()){
            self->showNormal();
            if(isMaximizedBeforeFullScreen){
                self->showMaximized();
            }
        }
    }
}

    
bool ViewArea::addView(View* view)
{
    return impl->addView(view);
}


bool ViewArea::Impl::addView(View* view)
{
    if(isBeforeDoingInitialLayout || view->viewArea() == self){
        return false;
    } else {
        if(needToUpdateDefaultPaneAreas){
            detectExistingPaneAreas();
        }
        addView(areaToPane[view->defaultLayoutArea()], view, false);
        return true;
    }
}


void ViewArea::Impl::addView(ViewPane* pane, View* view, bool makeCurrent)
{
    if(view->viewArea()){
        view->viewArea()->impl->removeView(view);
    }

    int index = pane->addView(view);
    if(makeCurrent){
        pane->setCurrentIndex(index);
    }
    view->setViewArea(self);

    ++numViews;

    if(numViews == 1){
        self->setWindowTitle(view->windowTitle());
    } else if(numViews == 2){
        self->setWindowTitle("Views");
        self->setViewTabsVisible(true);
    }
}


bool ViewArea::removeView(View* view)
{
    return impl->removeView(view);
}


bool ViewArea::Impl::removeView(View* view)
{
    bool removed = false;

    if(view->viewArea() == self){
        ViewPane* pane = nullptr;
        for(QWidget* widget = view->parentWidget(); widget; widget = widget->parentWidget()){
            pane = dynamic_cast<ViewPane*>(widget);
            if(pane){
                break;
            }
        }
        if(pane){
            removed = removeView(pane, view, false);
        }
    }

    return removed;
}


bool ViewArea::Impl::removeView(ViewPane* pane, View* view, bool isMovingInViewArea)
{
    bool removed = false;

    //! \todo check if the owner view area is same as this
    if(view->viewArea() == self){
        pane->removeView(view);
        if(pane->count() > 0){
            pane->setCurrentIndex(0);
        }
        --numViews;
        removed = true;

        if(!isMovingInViewArea){
            view->hide();
            view->setParent(0);
            removePaneIfEmpty(pane);
            if(self->isWindow()){
                if(numViews == 1){
                    View* existingView = findFirstView(topSplitter);
                    if(existingView){
                        self->setWindowTitle(existingView->windowTitle());
                    }
                } else if(numViews == 0){
                    self->deleteLater();
                }
            }
        }
        view->setViewArea(0);
    }
    return removed;
}


View* ViewArea::Impl::findFirstView(QSplitter* splitter)
{
    View* view = nullptr;
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            view = findFirstView(childSplitter);
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane && pane->count() > 0){
                view = pane->view(0);
            }
        }
        if(view){
            break;
        }
    }
    return view;
}


int ViewArea::numViews() const
{
    return impl->numViews;
}


void ViewArea::Impl::getVisibleViews(vector<View*>& out_views, QSplitter* splitter)
{
    getVisibleViewsIter(splitter ? splitter : topSplitter, out_views);
}


void ViewArea::Impl::getVisibleViewsIter(QSplitter* splitter, vector<View*>& out_views)
{
    QList<int> sizes = splitter->sizes();
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            getVisibleViewsIter(childSplitter, out_views);
        } else {
            if(sizes[i] > 0){
                ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
                if(pane){
                    View* view = pane->currentView();
                    if(view){
                        out_views.push_back(view);
                    }
                }
            }
        }
    }
}


void ViewArea::Impl::showViewSizeLabels(QSplitter* splitter)
{
    vector<View*> views;
    getVisibleViews(views, splitter);
    
    for(size_t i=0; i < views.size(); ++i){
        View* view = views[i];
        QLabel* label = nullptr;
        if(i < viewSizeLabels.size()){
            label = viewSizeLabels[i];
        } else {
            label = new QLabel(self);
            label->setFrameStyle(static_cast<int>(QFrame::Box) | static_cast<int>(QFrame::Raised));
            label->setLineWidth(0);
            label->setMidLineWidth(1);
            label->setAutoFillBackground(true);
            label->setAlignment(Qt::AlignCenter);
            viewSizeLabels.push_back(label);
        }
        label->setText(QString("%1 x %2").arg(view->width()).arg(view->height()));
        QPoint p = view->viewAreaPos();
        QSize s = label->size();
        int x = p.x() + view->width() / 2 - s.width() / 2;
        int y = p.y() + view->height() / 2 - s.height() / 2;
        label->move(x, y);
        label->show();
    }

    viewSizeLabelTimer.start();
}


void ViewArea::Impl::hideViewSizeLabels()
{
    for(size_t i=0; i < viewSizeLabels.size(); ++i){
        viewSizeLabels[i]->hide();
    }
}


/**
   This event filter is installed on the application object while a view is being dragged.
   The drag events are processed here before they are dispatched to the widgets in the
   window so that the widgets accepting drops for their own purposes do not intercept the
   events of the view drag operation.
*/
bool ViewArea::eventFilter(QObject* object, QEvent* event)
{
    QWindow* window = nullptr;

    switch(event->type()){

    case QEvent::DragEnter:
    case QEvent::DragMove:
        window = qobject_cast<QWindow*>(object);
        if(window && impl->onViewDragMoveEvent(window, static_cast<QDragMoveEvent*>(event))){
            return true;
        }
        break;

    case QEvent::DragLeave:
        if(qobject_cast<QWindow*>(object) && impl->onViewDragLeaveEvent()){
            return true;
        }
        break;

    case QEvent::Drop:
        window = qobject_cast<QWindow*>(object);
        if(window && impl->onViewDropEvent(window, static_cast<QDropEvent*>(event))){
            return true;
        }
        break;

    case QEvent::KeyPress:
        // The event itself is not consumed here to let the drag and drop framework cancel
        // the operation
        if(impl->isViewDragging && static_cast<QKeyEvent*>(event)->key() == Qt::Key_Escape){
            impl->isViewDragCanceled = true;
        }
        break;

    default:
        break;
    }

    return QWidget::eventFilter(object, event);
}


void ViewArea::keyPressEvent(QKeyEvent* event)
{
    event->ignore();
    
    switch(event->key()){

    case Qt::Key_F12:
        setViewTabsVisible(!impl->viewTabsVisible);
        event->accept();
        break;
    }

    if(isWindow()){
        switch(event->key()){
        case Qt::Key_F11:
            impl->setFullScreen(!isFullScreen());
            event->accept();
            break;
        }
    }
}


void ViewArea::restoreAllViewAreaLayouts(ArchivePtr archive)
{
    Impl* mainViewAreaImpl = MainWindow::instance()->viewArea()->impl;
    
    if(archive){
        Listing& layouts = *archive->findListing({ "view_areas", "viewAreas" });
        if(layouts.isValid()){
            auto screens = QGuiApplication::screens();
            auto defaultScreen = referenceScreen();
            for(int i=0; i < layouts.size(); ++i){
                Mapping& layout = *layouts[i].toMapping();
                auto contentsNode = layout.findMapping("contents");
                if(!contentsNode->isValid()){
                    continue;
                }
                Archive* contents = dynamic_cast<Archive*>(contentsNode->toMapping());
                if(contents){
                    contents->inheritSharedInfoFrom(*archive);
                    const string type = layout.get("type").toString();

                    if(type == "embedded"){
                        mainViewAreaImpl->restoreLayout(contents);
                        mainViewAreaImpl->self->setViewTabsVisible(layout.get("tabs", mainViewAreaImpl->viewTabsVisible));

                    } else if(type == "independent"){
                        ViewArea* viewWindow = new ViewArea;
                        viewWindow->impl->viewTabsVisible = layout.get("tabs", true);
                        viewWindow->impl->restoreLayout(contents);

                        if(viewWindow->impl->numViews == 0){
                            delete viewWindow;
                        } else {
                            // The screen is first identified by its name. The index is
                            // used when the name is not available or the screen of the name
                            // does not exist in the current screen configuration.
                            QScreen* screen = nullptr;
                            string screenName;
                            if(layout.read("screen_name", screenName)){
                                for(auto& candidate : screens){
                                    if(candidate->name().toStdString() == screenName){
                                        screen = candidate;
                                        break;
                                    }
                                }
                            }
                            if(!screen){
                                int screenIndex;
                                if(layout.read("screen", screenIndex)){
                                    if(screenIndex >= 0 && screenIndex < screens.size()){
                                        screen = screens[screenIndex];
                                    }
                                }
                            }
                            if(!screen){
                                screen = defaultScreen;
                            }
                            // The screen is explicitly specified for the window handle because
                            // the position given to setGeometry is ignored on Wayland and the
                            // window is not restored on the intended screen without this.
                            // Note that specifying the screen is only effective for a full
                            // screen window on Wayland.
                            viewWindow->createWinId();
                            viewWindow->windowHandle()->setScreen(screen);

                            const Listing& geo = *layout.findListing("geometry");
                            if(geo.isValid() && geo.size() == 4){
                                const QRect s = screen->geometry();
                                const QRect r(geo[0].toInt(), geo[1].toInt(), geo[2].toInt(), geo[3].toInt());
                                viewWindow->setGeometry(r.translated(s.x(), s.y()));
                            }
                            if(layout.get({ "full_screen", "fullScreen" }, false)){
                                layout.read("maximized", viewWindow->impl->isMaximizedBeforeFullScreen);
                                viewWindow->showFullScreen();
                            } else {
                                if(layout.get("maximized", false)){
                                    viewWindow->showMaximized();
                                } else {
                                    viewWindow->show();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if(isBeforeDoingInitialLayout){
        mainViewAreaImpl->resetLayout();
    }
}


void ViewArea::restoreLayout(ArchivePtr archive)
{
    impl->restoreLayout(archive.get());
}


void ViewArea::Impl::restoreLayout(Archive* archive)
{
    clearAllPanes();

    QWidget* topWidget = restoreViewContainer(*archive, archive);

    topSplitter = dynamic_cast<QSplitter*>(topWidget);
    
    if(!topSplitter){
        topSplitter = new CustomSplitter(this);
        ViewPane* topPane = dynamic_cast<ViewPane*>(topWidget);
        if(!topPane){
            topPane = new ViewPane(this);
        }
        topSplitter->addWidget(topPane);
    }
    vbox->addWidget(topSplitter);
    
    needToUpdateDefaultPaneAreas = true;

    isBeforeDoingInitialLayout = false;
}


void ViewArea::Impl::clearAllPanes()
{
    if(topSplitter){
        clearAllPanesSub(topSplitter);
        delete topSplitter;
        topSplitter = nullptr;
    }
    numViews = 0;
    needToUpdateDefaultPaneAreas = true;
}


void ViewArea::Impl::clearAllPanesSub(QSplitter* splitter)
{
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            clearAllPanesSub(childSplitter);
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane){
                while(pane->count() > 0){
                    int index = pane->count() - 1;
                    View* view = pane->view(index);
                    if(view){
                        pane->removeView(view);
                        view->hide();
                        view->setParent(0);
                        view->setViewArea(0);
                    }
                }
            }
        }
    }
}


void ViewArea::Impl::getAllViews(vector<View*>& out_views)
{
    getAllViewsSub(topSplitter, out_views);
}


void ViewArea::Impl::getAllViewsSub(QSplitter* splitter, vector<View*>& out_views)
{
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            getAllViewsSub(childSplitter, out_views);
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane){
                for(int i=0; i < pane->count(); ++i){
                    View* view = pane->view(i);
                    if(view){
                        out_views.push_back(view);
                    }
                }
            }
        }
    }
}


QWidget* ViewArea::Impl::restoreViewContainer(const Mapping& state, Archive* archive)
{
    QWidget* widget = nullptr;
    string type;
    if(state.read("type", type)){
        if(type == "splitter"){
            widget = restoreSplitter(state, archive);
        } else if(type == "pane"){
            widget = restorePane(state, archive);
        }
    }
    return widget;
}


QWidget* ViewArea::Impl::restoreSplitter(const Mapping& state, Archive* archive)
{
    QWidget* widget = nullptr;
    QWidget* childWidgets[2] = { 0, 0 };
    
    const Listing& children = *state.findListing("children");
    if(children.isValid()){
        int numChildren = std::min(children.size(), 2);
        for(int i=0; i < numChildren; ++i){
            if(children[i].isMapping()){
                const Mapping& childState = *children[i].toMapping();
                childWidgets[i] = restoreViewContainer(childState, archive);
            }
        }
        if(!childWidgets[0] || !childWidgets[1]){
            for(int i=0; i < 2; ++i){
                if(childWidgets[i]){
                    widget = childWidgets[i];
                    break;
                }
            }
        } else {
            QSplitter* splitter = new CustomSplitter(this);
            string orientation;
            if(state.read("orientation", orientation)){
                splitter->setOrientation((orientation == "vertical") ? Qt::Vertical : Qt::Horizontal);
            }
            splitter->addWidget(childWidgets[0]);
            splitter->addWidget(childWidgets[1]);

            const Listing& sizes = *state.findListing("sizes");
            if(sizes.isValid() && sizes.size() == 2){
                QList<int> s;
                int size;
                for(int i=0; i < 2; ++i){
                    if(sizes[i].read(size)){
                        s.push_back(size);
                    }
                }
                splitter->setSizes(s);
            }
            widget = splitter;
        }
    }
    return widget;
}


ViewPane* ViewArea::Impl::restorePane(const Mapping& state, Archive* archive)
{
    ViewPane* pane = nullptr;
    const Listing& views = *state.findListing("views");
    
    if(views.isValid() && !views.empty()){
        pane = new ViewPane(this);
        int currentId = 0;
        state.read("current", currentId);
        for(int i=0; i < views.size(); ++i){
            const ValueNode& node = views[i];
            View* view = nullptr;
            int id;
            bool isCurrent = false;
            if(node.read(id)){
                view = archive->findView(id);
                if(view){
                    isCurrent = (id == currentId);
                }
            }
            if(view){
                addView(pane, view, isCurrent);
            }
        }
        if(pane->count() == 0){
            delete pane;
            pane = nullptr;
        }
    }
    return pane;
}


void ViewArea::storeAllViewAreaLayouts(ArchivePtr archive)
{
    ListingPtr layouts = new Listing;
    for(size_t i=0; i < viewAreas.size(); ++i){
        ArchivePtr layout = new Archive;
        layout->inheritSharedInfoFrom(*archive);
        viewAreas[i]->storeLayout(layout);

        // A view area without the actual layout (e.g. in the no-window mode)
        // stores nothing, and the empty layout must be omitted so that it does
        // not clear the existing layout when the project is loaded in the GUI
        if(!layout->empty()){
            layouts->append(layout);
        }
    }
    if(!layouts->empty()){
        archive->insert("view_areas", layouts);
    }
}


void ViewArea::storeLayout(ArchivePtr archive)
{
    impl->storeLayout(archive.get());
}


void ViewArea::Impl::storeLayout(Archive* archive)
{
    if(!topSplitter){
        // The view area does not have the actual layout to store, for example,
        // when the application is executed in the no-window mode
        return;
    }
    try {
        MappingPtr state = storeSplitterState(topSplitter, archive);
        if(state){
            if(!self->isWindow()){
                archive->write("type", "embedded");
            } else {
                archive->write("type", "independent");

                // The screen is obtained from the window handle because it is the screen the
                // window is actually displayed on. Note that the screen cannot be detected
                // from the window position on Wayland, where the position is not available.
                QScreen* screen = nullptr;
                if(QWindow* windowHandle = self->windowHandle()){
                    screen = windowHandle->screen();
                }

                if(screen){
                    if(screen != referenceScreen()){
                        // The name of the screen is used as the primary identifier of the
                        // screen because the index of the screen array depends on the order
                        // in which the screens are detected and it may change. The index is
                        // also stored so that the layout can be restored on a different
                        // system with a similar screen configuration, and for the projects
                        // read by the older versions that do not support the name.
                        archive->write("screen_name", screen->name().toStdString(), DOUBLE_QUOTED);
                        auto screens = QGuiApplication::screens();
                        for(int i=0; i < screens.size(); ++i){
                            if(screen == screens[i]){
                                archive->write("screen", i);
                                break;
                            }
                        }
                    }
                    const QRect s = screen->geometry();
                    const QRect r = self->geometry().translated(-s.x(), -s.y());
                    Listing* geometry = archive->createFlowStyleListing("geometry");
                    geometry->append(r.x());
                    geometry->append(r.y());
                    geometry->append(r.width());
                    geometry->append(r.height());
                }
                
                if(self->isFullScreen()){
                    archive->write("full_screen", true);
                    archive->write("maximized", isMaximizedBeforeFullScreen);
                } else {
                    archive->write("full_screen", false);
                    archive->write("maximized", self->isMaximized());
                }
            }
            archive->write("tabs", viewTabsVisible);
            archive->insert("contents", state);
        }
    }
    catch(const ValueNode::Exception& ex){
        MessageView::instance()->putln(ex.message());
    }
}


MappingPtr ViewArea::Impl::storeSplitterState(QSplitter* splitter, Archive* archive)
{
    // The actual state object must be the Archive type if it is directly used to restore the state
    MappingPtr state = new Archive;

    ListingPtr children = new Listing;

    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            if(auto childState = storeSplitterState(childSplitter, archive)){
                children->append(childState);
            }
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane && pane->count() > 0){
                if(auto childState = storePaneState(pane, archive)){
                    children->append(childState);
                }
            }
        }
    }

    const int numChildren = children->size();
    if(numChildren == 0){
        state.reset();
    } else if(numChildren == 1){
        state = children->at(0)->toMapping();
    } else if(numChildren == 2){
        state->write("type", "splitter");
        state->write("orientation", (splitter->orientation() == Qt::Vertical) ? "vertical" : "horizontal");
        Listing* sizeSeq = state->createFlowStyleListing("sizes");
        QList<int> sizes = splitter->sizes();
        for(int i=0; i < sizes.size(); ++i){
            sizeSeq->append(sizes[i]);
        }
        state->insert("children", children);
    }

    return state;
}


MappingPtr ViewArea::Impl::storePaneState(ViewPane* pane, Archive* archive)
{
    // The actual state object must be the Archive type if it is directly used to restore the state
    MappingPtr state = new Archive;
    
    state->write("type", "pane");
    
    Listing* views = state->createFlowStyleListing("views");
    const int n = pane->count();
    for(int i=0; i < n; ++i){
        View* view = pane->view(i);
        int id = archive->getViewId(view);
        if(id >= 0){
            views->append(id);
            if(n >= 2 && (i == pane->currentIndex())){
                state->write("current", id);
            }
        }
    }
    if(views->empty()){
        state.reset();
    }
    return state;
}


void ViewArea::resetAllViewAreaLayouts()
{
    ViewArea* mainViewArea = MainWindow::instance()->viewArea();
    mainViewArea->impl->resetLayout();
    
    //! \todo close all the independent view windows
}


void ViewArea::resetLayout()
{
    impl->resetLayout();
}


void ViewArea::Impl::resetLayout()
{
    if(isBeforeDoingInitialLayout){
        isBeforeDoingInitialLayout = false;
    } else {
        // TODO: Load the layout of the builtin project and add the remaining
        // views with the permanent attribute.
        vector<View*> views;
        getAllViews(views);
        clearAllPanes();
        createDefaultPanes();
        for(size_t i=0; i < views.size(); ++i){
            addView(views[i]);
        }
    }
}


bool ViewArea::Impl::viewTabMousePressEvent(ViewPane* pane, QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton){
        tabDragStartPosition = getPosition(event);

    } else if(event->button() == Qt::RightButton){
        if(View* view = pane->currentView()){
            viewMenuManager.setNewPopupMenu(self);

            view->onAttachedMenuRequest(viewMenuManager);

            if(viewMenuManager.numItems() > 0){
                viewMenuManager.addSeparator();
            }
            viewMenuManager.addItem(_("Separate the view"))
                ->sigTriggered().connect([this, view](){ separateView(view); });

            viewMenuManager.popupMenu()->popup(getGlobalPosition(event));
        }
    }
    return false;
}


bool ViewArea::Impl::viewTabMouseMoveEvent(ViewPane* pane, QMouseEvent* event)
{
    if(isViewDragging || !(event->buttons() & Qt::LeftButton)){
        return false;
    }
    QPoint pos = getPosition(event);
    if((pos - tabDragStartPosition).manhattanLength() <= QApplication::startDragDistance()){
        return false;
    }
    if(pane->tabBar()->rect().contains(pos)){
        // The tab is being moved inside the tab bar to change the order of the tabs
        return false;
    }
    if(View* view = pane->currentView()){
        startViewDrag(pane, view);
        return true;
    }
    return false;
}


/**
   The drag operation of a view is implemented with the drag and drop framework of Qt
   instead of directly processing the mouse events during the operation. The framework is
   necessary because the destination of the operation must be detected with the global
   positions if the mouse events are directly processed, but the global positions are not
   available on Wayland. The framework makes the compositor deliver the positions of the
   pointer to each window in its own local coordinate system, and no global position is
   required to detect the destination.
*/
void ViewArea::Impl::startViewDrag(ViewPane* pane, View* view)
{
    isViewDragging = true;
    isViewDropAccepted = false;
    isViewDragCanceled = false;
    draggedView = view;
    dragSrcPane = pane;
    dragDestViewArea = nullptr;
    dragDestPane = nullptr;
    isViewDraggingOnOuterEdge = false;
    dropEdge = OVER;

    // Size of the window created when the view is dropped outside the existing windows
    QWidget* topLevelWidget = pane->window();
    QSize frameSize = topLevelWidget->frameGeometry().size();
    draggedViewWindowSize.setWidth(view->width() + frameSize.width() - topLevelWidget->width());
    draggedViewWindowSize.setHeight(view->height() + frameSize.height() - topLevelWidget->height());

    // The tab bar cannot detect the end of its own tab moving operation because the mouse
    // release event is consumed by the drag operation started below. Send a pseudo release
    // event to make the tab bar finish the operation in advance.
    QTabBar* tabBar = pane->tabBar();
    QMouseEvent releaseEvent(
        QEvent::MouseButtonRelease, tabDragStartPosition, tabDragStartPosition, tabDragStartPosition,
        Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(tabBar, &releaseEvent);

    if(outsideRubberBand){
        outsideRubberBandTimer.start();
    }

    auto mimeData = new QMimeData;
    mimeData->setData(viewDragMimeType, QByteArray());
    auto drag = new QDrag(tabBar);
    drag->setMimeData(mimeData);
    // Note that the drag pixmap is not displayed on Wayland as of Qt 6.10 due to a problem
    // of the Qt Wayland plugin. The plugin attaches and commits the buffer of the drag icon
    // surface before wl_data_device.start_drag assigns the drag-and-drop role to the surface,
    // and the surface is never committed again after that. A surface is displayed only when
    // it is committed after its role is assigned, so the icon never appears. This cannot be
    // worked around by the application side because the surface is completely hidden in the
    // framework. The pixmap is displayed as expected on the other platforms.
    QPixmap pixmap = createViewDragPixmap(view);
    qreal r = pixmap.devicePixelRatio();
    drag->setPixmap(pixmap);
    drag->setHotSpot(
        QPoint(static_cast<int>(pixmap.width() / (2 * r)), static_cast<int>(pixmap.height() / (2 * r))));

    qApp->installEventFilter(self);

    // The event filter is installed again in the event loop of the drag operation so that it
    // precedes the internal event filter of the drag and drop framework. The internal filter
    // consumes the key press event of the escape key, and the cancel operation with the key
    // cannot be detected without preceding it. Note that the key event is not delivered to
    // the application at all on the platforms where the system processes the drag operation
    // by itself, and the cancel operation cannot be detected on such platforms.
    QMetaObject::invokeMethod(
        self,
        [this](){
            if(isViewDragging){
                qApp->installEventFilter(self);
            }
        },
        Qt::QueuedConnection);

    // The drag object is not destroyed by the drag and drop framework and it must be
    // destroyed here. QPointer is used just in case the framework destroys it in the future.
    QPointer<QDrag> dragHolder(drag);

    drag->exec(Qt::MoveAction);

    if(dragHolder){
        dragHolder->deleteLater();
    }

    isViewDragging = false;
    qApp->removeEventFilter(self);

    finishViewDrag();
}


QPixmap ViewArea::Impl::createViewDragPixmap(View* view)
{
    const int hMargin = 8;
    const int vMargin = 4;

    QString title = view->windowTitle();
    QFont font = view->font();
    QFontMetrics metrics(font);
    QSize size(metrics.horizontalAdvance(title) + hMargin * 2, metrics.height() + vMargin * 2);

    qreal r = self->devicePixelRatioF();
    QPixmap pixmap(size * r);
    pixmap.setDevicePixelRatio(r);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    QPalette palette = view->palette();
    painter.setPen(palette.color(QPalette::WindowText));
    painter.setBrush(palette.color(QPalette::Button));
    painter.setOpacity(0.9);
    painter.drawRoundedRect(QRectF(0.5, 0.5, size.width() - 1.0, size.height() - 1.0), 3.0, 3.0);
    painter.setOpacity(1.0);
    painter.setFont(font);
    painter.drawText(QRect(QPoint(0, 0), size), Qt::AlignCenter, title);

    return pixmap;
}


/**
   This function detects the view area at the given position of the given window. Note that
   the position is converted only with the local coordinate systems of the window and the
   widgets in it so that the detection also works on Wayland.
*/
ViewArea* ViewArea::Impl::findDragDestViewArea(QWindow* window, const QPoint& posInWindow, QPoint& out_posInViewArea)
{
    for(auto& viewArea : viewAreas){
        QWidget* topLevelWidget = viewArea->window();
        if(topLevelWidget->windowHandle() != window){
            continue;
        }
        // The coordinate system of a window is the same as that of its top level widget
        QPoint pos = viewArea->mapFrom(topLevelWidget, posInWindow);
        if(viewArea->rect().contains(pos)){
            out_posInViewArea = pos;
            return viewArea;
        }
    }
    return nullptr;
}


void ViewArea::Impl::updateDragDestination(ViewArea* viewArea, const QPoint& posInViewArea)
{
    if(dragDestViewArea && dragDestViewArea != viewArea){
        dragDestViewArea->impl->rubberBand->hide();
        dragDestPane = nullptr;
    }
    dragDestViewArea = viewArea;

    if(!viewArea){
        dragDestPane = nullptr;
        return;
    }

    QWidget* pointed = viewArea->childAt(posInViewArea);
    while(pointed){
        if(auto pane = dynamic_cast<ViewPane*>(pointed)){
            dragDestPane = pane;
            break;
        }
        pointed = pointed->parentWidget();
    }
    // Note that the previous destination pane is kept if there is no pane at the position.
    // The position is on a splitter handle in that case, and keeping the pane is consistent
    // with the rubber band that is still displayed for the pane.
    if(dragDestPane){
        dragView(posInViewArea);
    }
}


bool ViewArea::Impl::onViewDragMoveEvent(QWindow* window, QDragMoveEvent* event)
{
    if(!isViewDragging || !event->mimeData()->hasFormat(viewDragMimeType)){
        return false;
    }
    if(isViewDropAccepted){
        // The destination has already been fixed by a drop event
        event->ignore();
        return true;
    }
    QPoint posInViewArea;
    ViewArea* viewArea = findDragDestViewArea(window, getPosition(event), posInViewArea);
    updateDragDestination(viewArea, posInViewArea);

    if(dragDestPane){
        event->setDropAction(Qt::MoveAction);
        event->accept();
    } else {
        event->ignore();
    }
    return true;
}


/**
   Note that a drag leave event may be delivered after a drop event on some platforms
   including Wayland. The destination must not be cleared in that case.
*/
bool ViewArea::Impl::onViewDragLeaveEvent()
{
    if(!isViewDragging){
        return false;
    }
    if(!isViewDropAccepted){
        if(dragDestViewArea){
            dragDestViewArea->impl->rubberBand->hide();
        }
        dragDestViewArea = nullptr;
        dragDestPane = nullptr;
    }
    return true;
}


bool ViewArea::Impl::onViewDropEvent(QWindow* window, QDropEvent* event)
{
    if(!isViewDragging || !event->mimeData()->hasFormat(viewDragMimeType)){
        return false;
    }
    QPoint posInViewArea;
    ViewArea* viewArea = findDragDestViewArea(window, getPosition(event), posInViewArea);
    updateDragDestination(viewArea, posInViewArea);

    if(dragDestPane){
        isViewDropAccepted = true;
        event->setDropAction(Qt::MoveAction);
        event->accept();
    } else {
        event->ignore();
    }
    return true;
}


/**
   This function displays the outline of the window created when the view is dropped at the
   current position. The outline is displayed only when the pointer is outside the view
   areas, but the drag events are not delivered in that case and the position must be
   obtained with a timer. Note that this function is only called by the timer, which is
   started only when the outside rubber band exists.
*/
void ViewArea::Impl::updateOutsideRubberBand()
{
    if(!isViewDragging || dragDestPane){
        if(outsideRubberBand->isVisible()){
            outsideRubberBand->hide();
        }
    } else {
        // The outline is shifted so that the pointer is not on the rubber band window. If the
        // pointer is on it, the window under it cannot be detected as the drop destination.
        QPoint pos = QCursor::pos() + QPoint(OUTSIDE_RUBBER_BAND_OFFSET, OUTSIDE_RUBBER_BAND_OFFSET);
        outsideRubberBand->setGeometry(QRect(pos, draggedViewWindowSize));
        if(!outsideRubberBand->isVisible()){
            outsideRubberBand->show();
        }
    }
}


void ViewArea::Impl::finishViewDrag()
{
    outsideRubberBandTimer.stop();
    if(outsideRubberBand){
        outsideRubberBand->hide();
    }
    if(dragDestViewArea){
        dragDestViewArea->impl->rubberBand->hide();
    }

    if(isViewDropAccepted && dragDestPane){
        bool isMovingInViewArea = (self == dragDestViewArea);
        removeView(dragSrcPane, draggedView, isMovingInViewArea);
        Impl* destImpl = dragDestViewArea->impl;
        destImpl->needToUpdateDefaultPaneAreas = true;
        if(isViewDraggingOnOuterEdge){
            destImpl->dropViewToOuterEdge(draggedView);
        } else {
            destImpl->dropViewInsidePane(dragDestPane, draggedView, dropEdge);
        }
        if(isMovingInViewArea){
            removePaneIfEmpty(dragSrcPane);
        }
    } else if(!isViewDragCanceled){
        removeView(dragSrcPane, draggedView, false);
        dropViewOutside();
    }

    draggedView = nullptr;
    dragSrcPane = nullptr;
    dragDestViewArea = nullptr;
    dragDestPane = nullptr;
}


void ViewArea::Impl::showRectangle(QRect r)
{
    rubberBand->setParent(self);
    rubberBand->setGeometry(r);
    rubberBand->show();
}


void ViewArea::Impl::dragView(const QPoint& posInDestViewArea)
{
    dropEdge = LEFT;

    const QPoint& p = posInDestViewArea;
    const int w = dragDestViewArea->width();
    const int h = dragDestViewArea->height();

    int distance[4];
    distance[LEFT] = p.x();
    distance[TOP] = p.y();
    distance[RIGHT] = w - p.x();
    distance[BOTTOM] = h - p.y();

    for(int i=TOP; i <= BOTTOM; ++i){
        if(distance[dropEdge] > distance[i]){
            dropEdge = i;
        }
    }

    if(distance[dropEdge] < OUTER_EDGE_DISTANCE_THRESHOLD){
        isViewDraggingOnOuterEdge = true;
        dragViewOnOuterEdge();
    } else {
        isViewDraggingOnOuterEdge = false;
        dragViewInsidePane(dragDestPane->mapFrom(dragDestViewArea, posInDestViewArea));
    }
}

    
void ViewArea::Impl::dragViewInsidePane(const QPoint& posInDestPane)
{
    dropEdge = LEFT;
        
    const int w = dragDestPane->width();
    const int h = dragDestPane->height();
    
    int distance[4];
    distance[LEFT] = posInDestPane.x();
    distance[TOP] = posInDestPane.y();
    distance[RIGHT] = w - posInDestPane.x();
    distance[BOTTOM] = h - posInDestPane.y();
        
    for(int i=TOP; i <= BOTTOM; ++i){
        if(distance[dropEdge] > distance[i]){
            dropEdge = i;
        }
    }

    QRect r;
    if(SPLIT_DISTANCE_THRESHOLD < distance[dropEdge]){
        r.setRect(0, 0, w, h);
        dropEdge = OVER;
    } else if(dropEdge == LEFT){
        r.setRect(0, 0, w / 2, h);
    } else if(dropEdge == TOP){
        r.setRect(0, 0, w, h /2);
    } else if(dropEdge == RIGHT){
        r.setRect(w / 2, 0, w / 2, h);
    } else if(dropEdge == BOTTOM){
        r.setRect(0, h / 2, w, h / 2);
    }

    r.translate(dragDestPane->mapTo(dragDestViewArea, QPoint(0, 0)));
    dragDestViewArea->impl->showRectangle(r);
}


void ViewArea::Impl::dropViewInsidePane(ViewPane* pane, View* view, int dropEdge)
{
    if(dropEdge == OVER){
        addView(pane, view, true);

    } else {
        QSize destSize = pane->size();

        QSplitter* parentSplitter = static_cast<QSplitter*>(pane->parentWidget());

        if(parentSplitter->count() >= 2){
            QList<int> sizes = parentSplitter->sizes();
            QSplitter* newSplitter = new CustomSplitter(this, parentSplitter);
            parentSplitter->insertWidget(parentSplitter->indexOf(pane), newSplitter);
            newSplitter->addWidget(pane);
            parentSplitter->setSizes(sizes);
            parentSplitter = newSplitter;
        }
        
        ViewPane* newViewPane = new ViewPane(this, parentSplitter);

        if(dropEdge == LEFT){
            parentSplitter->setOrientation(Qt::Horizontal);
            parentSplitter->insertWidget(0, newViewPane);
        } else if(dropEdge == RIGHT){
            parentSplitter->setOrientation(Qt::Horizontal);
            parentSplitter->insertWidget(1, newViewPane);
        } else if(dropEdge == TOP){
            parentSplitter->setOrientation(Qt::Vertical);
            parentSplitter->insertWidget(0, newViewPane);
        } else {
            parentSplitter->setOrientation(Qt::Vertical);
            parentSplitter->insertWidget(1, newViewPane);
        }
        addView(newViewPane, view, true);

        int half;
        if(parentSplitter->orientation() == Qt::Horizontal){
            half = destSize.height() / 2;
        } else {
            half = destSize.width() / 2;
        }
        QList<int> sizes;
        sizes << half << half;
        parentSplitter->setSizes(sizes);
    }
}


void ViewArea::Impl::dragViewOnOuterEdge()
{
    QRect r;
    int w = dragDestViewArea->width();
    int h = dragDestViewArea->height();
    if(dropEdge == LEFT){
        r.setRect(0, 0, w / 2, h);
    } else if(dropEdge == TOP){
        r.setRect(0, 0, w, h /2);
    } else if(dropEdge == RIGHT){
        r.setRect(w / 2, 0, w / 2, h);
    } else if(dropEdge == BOTTOM){
        r.setRect(0, h / 2, w, h / 2);
    }
    dragDestViewArea->impl->showRectangle(r);
}


void ViewArea::Impl::dropViewToOuterEdge(View* view)
{
    QSize size = topSplitter->size();

    if(topSplitter->count() >= 2){
        QSplitter* newTopSplitter = new CustomSplitter(this, self);
        newTopSplitter->addWidget(topSplitter);
        topSplitter = newTopSplitter;
        vbox->addWidget(topSplitter);
    }
    ViewPane* newViewPane = new ViewPane(this, topSplitter);

    if(dropEdge == LEFT){
        topSplitter->setOrientation(Qt::Horizontal);
        topSplitter->insertWidget(0, newViewPane);
    } else if(dropEdge == RIGHT){
        topSplitter->setOrientation(Qt::Horizontal);
        topSplitter->addWidget(newViewPane);
    } else if(dropEdge == TOP){
        topSplitter->setOrientation(Qt::Vertical);
        topSplitter->insertWidget(0, newViewPane);
    } else {
        topSplitter->setOrientation(Qt::Vertical);
        topSplitter->addWidget(newViewPane);
    }
    addView(newViewPane, view, true);

    int half;
    if(topSplitter->orientation() == Qt::Horizontal){
        half = size.height() / 2;
    } else {
        half = size.width() / 2;
    }
    QList<int> sizes;
    sizes << half << half;
    topSplitter->setSizes(sizes);
}


void ViewArea::Impl::dropViewOutside()
{
    separateView(draggedView, QCursor::pos(), draggedViewWindowSize);
}


void ViewArea::Impl::separateView(View* view)
{
    QPoint pos = view->mapToGlobal(QPoint(0, 0));
    removeView(view);
    separateView(view, pos, view->size());
}


void ViewArea::Impl::separateView(View* view, const QPoint& pos, const QSize& size)
{
    ViewArea* viewWindow = new ViewArea;
    viewWindow->setSingleView(view);

    // Make separated views always in front of the main window.
    // This only works on Windows.
#ifdef Q_OS_WIN32
    viewWindow->setParent(MainWindow::instance());
    viewWindow->setWindowFlags(Qt::Window);
#endif
    
    if(isGlobalPositionAvailable()){
        viewWindow->setGeometry(pos.x(), pos.y(), size.width(), size.height());
    } else {
        // The position of a window cannot be specified by a client application on Wayland
        viewWindow->resize(size);
    }
    viewWindow->show();
}


void ViewArea::Impl::removePaneIfEmpty(ViewPane* pane)
{
    if(pane->count() == 0){
        removePaneSub(pane, nullptr);
        needToUpdateDefaultPaneAreas = true;
    }
}


void ViewArea::Impl::removePaneSub(QWidget* widgetToRemove, QWidget* widgetToRaise)
{
    if(QSplitter* splitter = dynamic_cast<QSplitter*>(widgetToRemove->parentWidget())){
        QList<int> sizes;
        if(widgetToRaise){
            sizes = splitter->sizes();
            splitter->insertWidget(splitter->indexOf(widgetToRemove), widgetToRaise);
        }
        widgetToRemove->hide();
        widgetToRemove->setParent(nullptr);
        widgetToRemove->deleteLater();

        if(splitter->count() >= 2){
            splitter->setSizes(sizes);
        } else if(splitter->count() == 1){
            removePaneSub(splitter, splitter->widget(0));
        } else {
            removePaneSub(splitter, 0);
        }
    }
}


void ViewArea::Impl::clearEmptyPanes()
{
    QWidget* widget = clearEmptyPanesSub(topSplitter);
    if(widget != topSplitter){
        if(widget){
            if(QSplitter* splitter = dynamic_cast<QSplitter*>(widget)){
                splitter->setParent(self);
                topSplitter->hide();
                topSplitter->deleteLater();
                vbox->addWidget(splitter);
                topSplitter = splitter;
            }
        }
    }
}


QWidget* ViewArea::Impl::clearEmptyPanesSub(QSplitter* splitter)
{
    QList<int> sizes = splitter->sizes();
    int i = 0;
    while(i < splitter->count()){
        if(QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i))){
            QWidget* widget = clearEmptyPanesSub(childSplitter);
            if(widget == childSplitter){
                ++i;
            } else {
                if(widget){
                    splitter->insertWidget(i++, widget);
                }
                // "delete childSplitter" causes a crash
                childSplitter->hide();
                childSplitter->setParent(0);
                childSplitter->deleteLater();
            }
        } else if(ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i))){
            if(pane->count() > 0){
                ++i;
            } else {
                // "delete pane" causes a crash
                pane->hide();
                pane->setParent(0);
                pane->deleteLater();
                needToUpdateDefaultPaneAreas = true;
            }
        }
    }
    QWidget* validWidget = nullptr;
    if(splitter->count() >= 2){
        splitter->setSizes(sizes);
        validWidget = splitter;
    } else if(splitter->count() == 1){
        validWidget = splitter->widget(0);
        needToUpdateDefaultPaneAreas = true;
    }
    return validWidget;
}
