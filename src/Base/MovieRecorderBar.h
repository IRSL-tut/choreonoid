#ifndef CNOID_BASE_MOVIE_RECORDER_BAR_H
#define CNOID_BASE_MOVIE_RECORDER_BAR_H

#include "ToolBar.h"
#include <cnoid/ConnectionSet>
#include <QIcon>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class MovieRecorder;

class CNOID_EXPORT MovieRecorderBar : public ToolBar
{
public:
    static void initializeClass(ExtensionManager* ext);
    static MovieRecorderBar* instance() { return instance_; }
    
    MovieRecorderBar();

protected:
    virtual void showEvent(QShowEvent* event) override;

private:
    void onRecordingStateChanged(bool on);
    void onRecordingConfigurationChanged();
    void onRecordingButtonToggled(bool on);
    void onViewMarkerButtonToggled(bool on);
    virtual void showMovieRecorderDialog();
    
    static MovieRecorderBar* instance_;
    MovieRecorder* recorder;
    ToolButton* recordingToggle;
    ToolButton* viewMarkerToggle;
    ScopedConnectionSet recorderConnections;
    QIcon recordIcon;
    QIcon recordBlinkIcon;
};

}

#endif
