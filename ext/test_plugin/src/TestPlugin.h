#ifndef CNOID_TEST_PLUGIN_H
#define CNOID_TEST_PLUGIN_H

#include <cnoid/Plugin>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TestPlugin : public Plugin
{
public:
    TestPlugin();
    ~TestPlugin();

    static TestPlugin* instance();

    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual const char* description() const override;

    class Impl;
private:
    Impl *impl;
};

}

#endif
