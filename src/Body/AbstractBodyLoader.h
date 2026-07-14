/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_ABSTRACT_BODY_LOADER_H
#define CNOID_BODY_ABSTRACT_BODY_LOADER_H

#include <string>
#include <memory>
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class Body;
class SgObject;

class CNOID_EXPORT AbstractBodyLoader
{
public:
    AbstractBodyLoader();
    virtual ~AbstractBodyLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual void setShapeLoadingEnabled(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);
    virtual bool load(Body* body, const std::string& filename) = 0;

protected:
    /**
       Sets the original URI description of a mesh file written in a model file (e.g. a
       "model://" or "package://" URI) to the scene loaded from the mesh file. The scene
       loader sets the URI of the loaded file to the scene object that represents the whole
       file (the mesh object or the scene top node); this function finds that object by the
       resolved file path and replaces its URI with the original URI description so that the
       mesh file reference can be restored when the body is saved. Matching by the file path
       avoids picking an unrelated URI object such as a texture image. The top object is used
       as a fallback in case the scene loader does not give the URI information.
       \param scene The scene loaded from the mesh file
       \param uri The original URI description of the mesh file written in the model file
       \param filePath The actual file path of the mesh file resolved from the URI
    */
    static void setUriInformationToMeshScene(
        SgObject* scene, const std::string& uri, const std::string& filePath);
};

typedef std::shared_ptr<AbstractBodyLoader> AbstractBodyLoaderPtr;

}

#endif
