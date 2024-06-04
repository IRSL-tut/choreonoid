#ifndef CNOID_TEST_PLUGIN_H
#define CNOID_TEST_PLUGIN_H

#include <cnoid/Plugin>

namespace cnoid {

class TestPlugin : public Plugin
{
public:
    TestPlugin();
    ~TestPlugin();

    static TestPlugin* instance();

    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual const char* description() const override;
private:
    class Impl;
    Impl *impl;
};

}

#endif
