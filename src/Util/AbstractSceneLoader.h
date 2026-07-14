#ifndef CNOID_UTIL_ABSTRACT_SCENE_LOADER_H
#define CNOID_UTIL_ABSTRACT_SCENE_LOADER_H

#include "SceneGraph.h"
#include <iosfwd>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AbstractSceneLoader
{
public:
    AbstractSceneLoader();
    virtual ~AbstractSceneLoader();

    virtual void setMessageSink(std::ostream& os);
    virtual void setDefaultDivisionNumber(int n);
    virtual void setDefaultCreaseAngle(double theta);

    /**
       Hints the loader at additional directories to consider when a texture image referenced
       by the scene file cannot be located beside the scene file itself. The default
       implementation does nothing; concrete loaders that handle this hint (currently
       AssimpSceneLoader) override it. See AssimpSceneLoader::addImageSearchDirectory for the
       lookup rules and threading contract.
    */
    virtual void addImageSearchDirectory(const std::string& directory);
    virtual void clearImageSearchDirectories();

    enum LengthUnitType { Meter, Millimeter, Inch, NumLengthUnitTypes };
    virtual void setLengthUnitHint(LengthUnitType hint);
    LengthUnitType lengthUnitHint() const { return lengthUnitHint_; }
    
    enum UpperAxisType { Z_Upper, Y_Upper, NumUpperAxisTypes };
    virtual void setUpperAxisHint(UpperAxisType hint);
    UpperAxisType upperAxisHint() const { return upperAxisHint_; }

    void clearHintsForLoading();
    void restoreLengthUnitAndUpperAxisHints(Mapping* metadata);

    virtual SgNode* load(const std::string& filename) = 0;

protected:
    SgNode* insertTransformNodesToAdjustLengthUnitAndUpperAxis(SgNode* node);
    SgNode* insertTransformNodeToAdjustUpperAxis(SgNode* node);
    void storeLengthUnitAndUpperAxisHintsAsMetadata(SgObject* object);

    /**
       Sets the URI information of the loaded file to the scene object that represents the
       file, together with the length unit / upper axis hint metadata. Every concrete loader
       should call this on the object corresponding to the whole file (usually the top node
       of the loaded scene) so that downstream components such as scene / body writers can
       restore the reference to the original file instead of embedding the loaded data.
    */
    void setFileUriInformationToScene(SgObject* object, const std::string& filename);

private:
    LengthUnitType lengthUnitHint_;
    UpperAxisType upperAxisHint_;
};

}

#endif
