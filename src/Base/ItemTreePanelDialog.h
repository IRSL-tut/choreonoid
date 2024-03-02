#ifndef CNOID_BASE_ITEM_TREE_PANEL_DIALOG_H
#define CNOID_BASE_ITEM_TREE_PANEL_DIALOG_H

#include <cnoid/Dialog>
#include "exportdecl.h"

namespace cnoid {

class ItemTreeWidget;
class Item;
class ItemTreePanelBase;

class CNOID_EXPORT ItemTreePanelDialog : public Dialog
{
public:
    ItemTreePanelDialog();
    ItemTreePanelDialog(QWidget* parent, Qt::WindowFlags f = Qt::WindowFlags());
    ~ItemTreePanelDialog();

    enum ModeFlags {
        PanelOnlyDisplayMode = 1,
        LastValidPanelKeepingMode = 2,
        // The dialog is always closed when the operation on a single panel is accepted or rejected
        SinglePanelSyncMode = 4
    };

    void setMode(int flags);
    int mode() const;

    ItemTreeWidget* itemTreeWidget();

    void registerPanel(
        std::function<ItemTreePanelBase*()> instanceFunction,
        std::function<bool(Item* item)> pred);

    template<class TargetItemType>
    [[deprecated]]
    void registerPanel(
        std::function<ItemTreePanelBase*(TargetItemType* item)> panelFunction,
        std::function<QSize()> minimumSizeHintFunction)
    {
        registerPanel_(
            typeid(TargetItemType),
            [panelFunction](Item* item){ return panelFunction(static_cast<TargetItemType*>(item)); },
            minimumSizeHintFunction);
    }

    void addTopAreaWidget(QWidget* widget);
    void addTopAreaSpacing(int spacing);
    void setTopAreaLayoutStretchEnabled(bool on);
    
    bool setTopItem(Item* topItem, bool isTopVisible = false);
    void show();
    bool setCurrentItem(Item* item, bool isNewItem = false);
    Item* currentItem();

protected:
    virtual void onCurrentPanelCaptionChanged();
    virtual void onCurrentItemChanged(Item* item);
    virtual bool onDialogClosed();

    virtual void closeEvent(QCloseEvent *event) override;
    
private:
    class Impl;
    Impl* impl;

    void registerPanel_(
        const std::type_info& type,
        const std::function<ItemTreePanelBase*(Item* item)>& panelFunction,
        const std::function<QSize()>& minimumSizeHintFunction);

    friend class ItemTreePanelBase;
};

class CNOID_EXPORT ItemTreePanelBase : public QWidget
{
public:
    ItemTreePanelBase(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
    bool activate(Item* topItem, Item* targetItem, bool isNewItem, ItemTreePanelDialog* currentDialog);
    void setCaption(const std::string& caption);
    const std::string& caption() const { return caption_; }

    virtual bool onActivated(Item* topItem, Item* targetItem, bool isNewItem) = 0;

    enum DeactivationType { Undetermined, Accepted, Rejected };
    virtual void onDeactivated(int deactivationType);
    
    ItemTreePanelDialog* dialog(){ return currentDialog; }
    ItemTreeWidget* itemTreeWidget();

protected:
    void accept();
    void reject();
    
private:
    void finish(int deactivationType);
    
    ItemTreePanelDialog* currentDialog;
    std::string caption_;
};

template<class TopItemType, class TargetItemType>
class ItemTreePanel : public ItemTreePanelBase
{
public:
    ItemTreePanel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags())
        : ItemTreePanelBase(parent, f)
    { }

    virtual bool onActivated(Item* topItem, Item* targetItem, bool isNewItem) override final {
        if(auto derivedTopItem = dynamic_cast<TopItemType*>(topItem)){
            if(auto derivedTargetItem = dynamic_cast<TargetItemType*>(targetItem)){
                return onActivated(derivedTopItem, derivedTargetItem, isNewItem);
            }
        }
        return false;
    }
    
    virtual bool onActivated(TopItemType* topItem, TargetItemType* targetItem, bool isNewItem) = 0;
};

}

#endif
