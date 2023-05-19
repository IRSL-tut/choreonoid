#include <cnoid/AssimpSceneWriter>
#include <cnoid/BodyLoader>

#include <cnoid/Body>
#include <cnoid/SceneGraph>

#include <iostream>

using namespace cnoid;

int main(int argc, char **argv)
{
    AssimpSceneWriter asw;
    asw.setMessageSink(std::cerr);
    asw.setVerbose(true);
    asw.generatePrimitiveMesh(true);

    BodyLoader bl;
    bl.setMessageSink(std::cerr);
    bl.setVerbose(true);

    //Body *bd = bl.load("/home/leus/src/choreonoid/share/model/AizuSpider/AizuSpiderSS.body");
    Body *bd = bl.load("/home/leus/src/choreonoid/share/model/SR1/SR1.body");

    if (!bd) {
        std::cerr << "body open error!" << std::endl;
        return -1;
    }

    Link *rl = bd->rootLink();

    if (!rl) {
        std::cerr << "root link not found!" << std::endl;
        return -1;
    }

    SgNode *vs = rl->visualShape();

    if (!vs) {
        std::cerr << "visual shape of root link not found!" << std::endl;
        return -1;
    }

    bool res = asw.writeScene("/tmp/SR1_root_shape.stl", vs);

    if (!res) {
        std::cerr << "Scene write error!" << std::endl;
        return -1;
    }

    return 0;
}
