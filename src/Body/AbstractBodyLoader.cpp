/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "AbstractBodyLoader.h"
#include <cnoid/SceneGraph>

using namespace cnoid;


AbstractBodyLoader::AbstractBodyLoader()
{

}


AbstractBodyLoader::~AbstractBodyLoader()
{

}


void AbstractBodyLoader::setMessageSink(std::ostream& /* os */)
{

}


void AbstractBodyLoader::setVerbose(bool /* on */)
{

}


void AbstractBodyLoader::setShapeLoadingEnabled(bool /* on */)
{

}


void AbstractBodyLoader::setDefaultDivisionNumber(int /* n */)
{

}


void AbstractBodyLoader::setDefaultCreaseAngle(double /* theta */)
{

}


void AbstractBodyLoader::setUriInformationToMeshScene
(SgObject* scene, const std::string& uri, const std::string& filePath)
{
    SgObject* uriObject = scene->findObject(
        [&filePath](SgObject* object){
            return object->hasUri() && object->localFilePath() == filePath;
        });
    if(!uriObject){
        uriObject = scene;
    }
    uriObject->setUri(uri, filePath);
}
