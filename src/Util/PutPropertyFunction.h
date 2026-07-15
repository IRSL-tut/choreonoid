#ifndef CNOID_UTIL_PUT_PROPERTY_FUNCTION_H
#define CNOID_UTIL_PUT_PROPERTY_FUNCTION_H

#include "Selection.h"
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class FilePathProperty
{
    std::string filename_;
    std::string baseDirectory_;
    std::vector<std::string> filters_;
    bool isFullpathDisplayMode_ = false;
    bool isExistingFileMode_ = true;
    bool isExtensionRemovalModeForFileDialogSelection_ = false;

public:
    FilePathProperty() { }
    FilePathProperty(const std::string& filename) : filename_(filename) { }
    FilePathProperty(const std::string& filename, const std::vector<std::string>& filters)
        : filename_(filename), filters_(filters) { }

    const std::string& filename() const { return filename_; };
    void setFilename(const std::string& filename) { filename_ = filename; }

    void setExtensionRemovalModeForFileDialogSelection(bool on) {
        isExtensionRemovalModeForFileDialogSelection_ = on; }
    bool isExtensionRemovalModeForFileDialogSelection() const {
        return isExtensionRemovalModeForFileDialogSelection_; }
    
    const std::string& baseDirectory() const { return baseDirectory_; };
    void setBaseDirectory(const std::string& dir) { baseDirectory_ = dir; }

    const std::vector<std::string>& filters() const { return filters_; }
    void setFilters(const std::vector<std::string>& filters){ filters_ = filters; }

    bool isFullpathDisplayMode() const { return isFullpathDisplayMode_; }
    void setFullpathDisplayMode(bool on) { isFullpathDisplayMode_ = on; }
    
    bool isExistingFileMode() const { return isExistingFileMode_; }
    void setExistingFileMode(bool on) { isExistingFileMode_ = on; }
};


class CNOID_EXPORT PutPropertyFunction
{
public:
    virtual ~PutPropertyFunction();

    virtual PutPropertyFunction& min(double min) = 0;
    virtual PutPropertyFunction& max(double max) = 0;
    virtual PutPropertyFunction& range(double min, double max) = 0;
    virtual PutPropertyFunction& decimals(int d) = 0;

    /**
       Set the callback function called when the value of a property is
       modified by the property editing interface. Like the other decoration
       functions such as min and decimals, the callback is applied to the
       properties declared after this function is called, and it is cleared
       by the reset function. The callback is called after the change
       function given with a property returns true. This is mainly used by
       an object which delegates the property declaration to another object
       and has to do some processing when a delegated property is modified,
       which cannot be written in the change functions given by the
       delegated object.
    */
    virtual PutPropertyFunction& callOnChange(std::function<void()> callback) = 0;

    virtual PutPropertyFunction& reset() = 0;

    // bool
    virtual void operator()(const std::string& name, bool value) = 0;
    virtual void operator()(const std::string& name, bool value,
                            const std::function<bool(bool)>& changeFunc) = 0;
    // int
    virtual void operator()(const std::string& name, int value) = 0;
    virtual void operator()(const std::string& name, int value,
                            const std::function<bool(int)>& changeFunc) = 0;
    // double
    virtual void operator()(const std::string& name, double value) = 0;
    virtual void operator()(const std::string& name, double value,
                            const std::function<bool(double)>& changeFunc) = 0;
    // string
    virtual void operator()(const std::string& name, const std::string& value) = 0;
    virtual void operator()(const std::string& name, const std::string& value,
                            const std::function<bool(const std::string&)>& changeFunc) = 0;

    void operator()(const std::string& name, const char* value){
        operator()(name, std::string(value));
    }

    // Selection
    virtual void operator()(const std::string& name, const Selection& selection) = 0;
    virtual void operator()(const std::string& name, const Selection& selection,
                            const std::function<bool(int which)>& changeFunc) = 0;
    // FilePath
    virtual void operator()(const std::string& name, const FilePathProperty& filepath) = 0;
    virtual void operator()(const std::string& name, const FilePathProperty& filepath,
                            const std::function<bool(const std::string&)>& changeFunc) = 0;
};


template <class ValueType>
class ChangeProperty
{
    ValueType& variable;
public:
    ChangeProperty(ValueType& variable) : variable(variable) { }
    bool operator()(const ValueType& value){
        variable = value;
        return true;
    }
};

template <>
class ChangeProperty<Selection>
{
    Selection& selection;
public:
    ChangeProperty(Selection& variable) : selection(variable) { }
    bool operator()(int value){
        selection.select(value);
        return true;
    }
};
    
template<class ValueType>
ChangeProperty<ValueType> changeProperty(ValueType& variable) {
    return ChangeProperty<ValueType>(variable);
}

}

#endif
