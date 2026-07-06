#include "BulletPlugin.h"
#include "BulletSimulatorItem.h"
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <LinearMath/btScalar.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;


BulletPlugin::BulletPlugin()
    : Plugin("Bullet")
{
    require("Body");
}


bool BulletPlugin::initialize()
{
    int version = btGetVersion();
    MessageOut::master()->put(
        formatR(_("Bullet Physics {0}.{1} is available.\n"), version / 100, version % 100));

    BulletSimulatorItem::initialize(this);

    return true;
}


const char* BulletPlugin::description() const
{
    static std::string text =
        formatC("Bullet Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyright (c) 2026 Choreonoid Inc.\n"
        "\n" +
        MITLicenseText() +
        "\n" +
        "This plugin uses the Bullet Physics library for physics simulation, "
        "which is provided under the following license:\n"
        "\n"
        "zlib License\n"
        "\n"
        "Copyright (c) 2003-2025 Erwin Coumans and the Bullet Physics contributors.\n"
        "\n"
        "This software is provided 'as-is', without any express or implied warranty. "
        "In no event will the authors be held liable for any damages arising from "
        "the use of this software. Permission is granted to anyone to use this "
        "software for any purpose, including commercial applications, and to alter "
        "it and redistribute it freely, subject to the following restrictions:\n"
        "\n"
        "1. The origin of this software must not be misrepresented; you must not "
        "claim that you wrote the original software. If you use this software in a "
        "product, an acknowledgment in the product documentation would be "
        "appreciated but is not required.\n"
        "2. Altered source versions must be plainly marked as such, and must not be "
        "misrepresented as being the original software.\n"
        "3. This notice may not be removed or altered from any source distribution.\n";
    return text.c_str();
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(BulletPlugin);
