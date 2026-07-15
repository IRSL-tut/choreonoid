#ifndef CNOID_UTIL_PUT_PROPERTY_FUNCTION_H
#define CNOID_UTIL_PUT_PROPERTY_FUNCTION_H

#include "Selection.h"
#include <functional>
#include <string_view>
#include "exportdecl.h"

namespace cnoid {

//! A file path value with the options on how to display and select the path
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
    FilePathProperty(std::string_view filename) : filename_(filename) { }

    //! \param filters The file dialog filters such as "Body files (*.body)"
    FilePathProperty(std::string_view filename, std::vector<std::string> filters)
        : filename_(filename), filters_(std::move(filters)) { }

    const std::string& filename() const { return filename_; };
    void setFilename(std::string_view filename) { filename_ = filename; }

    /**
       When this mode is enabled, the filename extension is removed from the
       file path selected in the file dialog.
    */
    void setExtensionRemovalModeForFileDialogSelection(bool on) {
        isExtensionRemovalModeForFileDialogSelection_ = on; }
    bool isExtensionRemovalModeForFileDialogSelection() const {
        return isExtensionRemovalModeForFileDialogSelection_; }

    //! The directory where the file dialog begins when the filename is a relative path
    const std::string& baseDirectory() const { return baseDirectory_; };
    void setBaseDirectory(std::string_view dir) { baseDirectory_ = dir; }

    const std::vector<std::string>& filters() const { return filters_; }
    void setFilters(std::vector<std::string> filters){ filters_ = std::move(filters); }

    /**
       When this mode is enabled, the whole path is displayed as the property
       value. Otherwise only the filename part is displayed and the whole
       path is shown as a tooltip.
    */
    bool isFullpathDisplayMode() const { return isFullpathDisplayMode_; }
    void setFullpathDisplayMode(bool on) { isFullpathDisplayMode_ = on; }

    /**
       When this mode is enabled, the file dialog only accepts an existing
       file. Disable this mode for a property which specifies a new file to
       be created such as a log file.
    */
    bool isExistingFileMode() const { return isExistingFileMode_; }
    void setExistingFileMode(bool on) { isExistingFileMode_ = on; }
};


/**
   The interface for an object to declare its properties to the property
   editing / viewing interface. Each property is declared by the operator()
   with the property name, the current value, and optionally the function
   to change the value. The change function returns true when the given
   value is accepted.

   The decoration functions (min, max, range, decimals, and callOnChange)
   affect the properties declared after the decoration call, and all the
   decorations are cleared by the reset function.
*/
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
       modified by the property editing interface. The callback is called
       after the change function given with a property returns true. This is
       mainly used by an object which delegates the property declaration to
       another object and has to do some processing when a delegated
       property is modified, which cannot be written in the change functions
       given by the delegated object.
    */
    virtual PutPropertyFunction& callOnChange(std::function<void()> callback) = 0;

    virtual PutPropertyFunction& reset() = 0;

    // bool
    virtual void operator()(std::string_view name, bool value) = 0;
    virtual void operator()(std::string_view name, bool value,
                            std::function<bool(bool)> changeFunc) = 0;
    // int
    virtual void operator()(std::string_view name, int value) = 0;
    virtual void operator()(std::string_view name, int value,
                            std::function<bool(int)> changeFunc) = 0;
    // double
    virtual void operator()(std::string_view name, double value) = 0;
    virtual void operator()(std::string_view name, double value,
                            std::function<bool(double)> changeFunc) = 0;
    // string
    virtual void operator()(std::string_view name, std::string_view value) = 0;
    virtual void operator()(std::string_view name, std::string_view value,
                            std::function<bool(const std::string&)> changeFunc) = 0;

    /**
       This overload is necessary to process a string literal value as a
       string value because the implicit conversion from const char* to bool
       is preferred to the conversion to string_view in overload resolution.
    */
    void operator()(std::string_view name, const char* value){
        operator()(name, std::string_view(value));
    }

    // Selection
    virtual void operator()(std::string_view name, const Selection& selection) = 0;
    virtual void operator()(std::string_view name, const Selection& selection,
                            std::function<bool(int which)> changeFunc) = 0;
    // FilePath
    virtual void operator()(std::string_view name, const FilePathProperty& filepath) = 0;
    virtual void operator()(std::string_view name, const FilePathProperty& filepath,
                            std::function<bool(const std::string&)> changeFunc) = 0;
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
