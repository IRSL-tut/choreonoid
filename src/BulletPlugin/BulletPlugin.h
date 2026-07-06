#ifndef CNOID_BULLET_PLUGIN_BULLET_PLUGIN_H
#define CNOID_BULLET_PLUGIN_BULLET_PLUGIN_H

#include <cnoid/Plugin>

namespace cnoid {

class BulletPlugin : public Plugin
{
public:
    BulletPlugin();
    virtual bool initialize() override;
    virtual const char* description() const override;
};

}

#endif
