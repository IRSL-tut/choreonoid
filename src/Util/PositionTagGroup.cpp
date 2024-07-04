#include "PositionTagGroup.h"
#include "ValueTree.h"
#include "UTF8.h"
#include "Tokenizer.h"
#include "EigenUtil.h"
#include "Format.h"
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class PositionTagGroup::Impl
{
public:
    string name;
    Signal<void(int index)> sigTagAdded;
    Signal<void(int index, PositionTag* tag, bool isChaningOrder)> sigTagRemoved;
    Signal<void(int index)> sigTagPositionChanged;
    Signal<void(int index)> sigTagPositionUpdated;
    
    Impl();
    Impl(const Impl& org);
};

}


PositionTagGroup::PositionTagGroup()
{
    impl = new Impl;
}


PositionTagGroup::Impl::Impl()
{

}


PositionTagGroup::PositionTagGroup(const PositionTagGroup& org)
{
    impl = new Impl(*org.impl);

    tags_.reserve(org.tags_.size());
    for(auto& tag : org.tags_){
        append(new PositionTag(*tag));
    }
}


PositionTagGroup::Impl::Impl(const Impl& org)
    : name(org.name)
{

}


PositionTagGroup::~PositionTagGroup()
{
    delete impl;
}


Referenced* PositionTagGroup::doClone(CloneMap*) const
{
    return new PositionTagGroup(*this);
}


const std::string& PositionTagGroup::name() const
{
    return impl->name;
}


void PositionTagGroup::setName(const std::string& name)
{
    impl->name = name;
}


void PositionTagGroup::clearTags()
{
    while(!tags_.empty()){
        removeAt(tags_.size() - 1);
    }
}


void PositionTagGroup::insert(int index, PositionTag* tag)
{
    int n = tags_.size();
    if(index > n){
        index = n;
    }
    tags_.insert(tags_.begin() + index, tag);

    impl->sigTagAdded(index);
}


void PositionTagGroup::insert(int index, PositionTagGroup* group)
{
    int n = tags_.size();
    if(index > n){
        index = n;
    }
    auto it = tags_.begin() + index;
    for(auto& tag : *group){
        it = tags_.insert(it, tag);
        impl->sigTagAdded(index++);
        it++;
    }
}


void PositionTagGroup::append(PositionTag* tag)
{
    insert(tags_.size(), tag);
}


bool PositionTagGroup::removeAt(int index)
{
    if(index >= static_cast<int>(tags_.size())){
        return false;
    }
    PositionTagPtr tag = tags_[index];
    tags_.erase(tags_.begin() + index);
    impl->sigTagRemoved(index, tag, false);
    return true;
}


bool PositionTagGroup::changeOrder(int orgIndex, int newIndex)
{
    int n = tags_.size();
    if(orgIndex >= n || newIndex >= n){
        return false;
    }
    if(orgIndex == newIndex){
        return true;
    }

    PositionTagPtr tag = tags_[orgIndex];
    tags_.erase(tags_.begin() + orgIndex);
    impl->sigTagRemoved(orgIndex, tag, true);
    insert(newIndex, tag);

    return true;
}


SignalProxy<void(int index)> PositionTagGroup::sigTagAdded()
{
    return impl->sigTagAdded;
}


SignalProxy<void(int index, PositionTag* tag, bool isChaingingOrder)> PositionTagGroup::sigTagRemoved()
{
    return impl->sigTagRemoved;
}


SignalProxy<void(int index)> PositionTagGroup::sigTagPositionChanged()
{
    return impl->sigTagPositionChanged;
}


SignalProxy<void(int index)> PositionTagGroup::sigTagPositionUpdated()
{
    return impl->sigTagPositionUpdated;
}


void PositionTagGroup::notifyTagPositionChange(int index)
{
    if(static_cast<size_t>(index) < tags_.size()){
        impl->sigTagPositionChanged(index);
    }
}


void PositionTagGroup::notifyTagPositionUpdate(int index, bool doNotifyPositionChange)
{
    if(static_cast<size_t>(index) < tags_.size()){
        if(doNotifyPositionChange){
            impl->sigTagPositionChanged(index);
        }
        impl->sigTagPositionUpdated(index);
    }
}


bool PositionTagGroup::read(const Mapping* archive)
{
    auto& typeNode = archive->get("type");
    if(typeNode.toString() != "PositionTagGroup"){
        typeNode.throwException(
            formatR(_("{0} cannot be loaded as a position tag group"), typeNode.toString()));
    }

    auto versionNode = archive->find("format_version");
    auto version = versionNode->toDouble();
    if(version != 1.0){
        versionNode->throwException(formatR(_("Format version {0} is not supported."), version));
    }

    archive->read("name", impl->name);

    clearTags();

    auto listing = archive->findListing("tags");
    if(listing->isValid()){
        for(auto& node : *listing){
            PositionTagPtr tag = new PositionTag;
            if(tag->read(node->toMapping())){
                append(tag);
            }
        }
    }

    return true;
}


bool PositionTagGroup::write(Mapping* archive) const
{
    archive->write("type", "PositionTagGroup");
    archive->write("format_version", 1.0);
    archive->write("name", impl->name);

    if(!tags_.empty()){
        auto listing = archive->createListing("tags");
        for(auto& tag : tags_){
            MappingPtr node = new Mapping;
            if(!tag->write(node)){
                return false;
            }
            listing->append(node);
        }
    }
    
    return true;
}


bool PositionTagGroup::loadCsvFile
(const std::string& filename, CsvFormat csvFormat, std::ostream& os)
{
    ifstream is(fromUTF8(filename).c_str());
    if(!is){
        os << formatR(_("\"{}\" cannot be opened."), filename) << endl;
        return false;
    }
  
    int lineNumber = 0;
    string line;
    Tokenizer<CharSeparator<char>> tokens(CharSeparator<char>(","));
    vector<Vector6> data;

    try {
        while(getline(is, line)){
            lineNumber++;
            tokens.assign(line);
            Vector6 xyzrpy;
            int i = 0;
            for(auto& token : tokens){
                if(i > 5){
                    os << formatR(_("Too many elements at line {0} of \"{1}\"."), lineNumber, filename) << endl;
                    return false;
                }
                xyzrpy[i++] = std::stod(token);
            }
            if(i > 0){ // Check to skip an empty line
                while(i < 6){
                    xyzrpy[i++] = 0.0;
                }
                data.push_back(xyzrpy);
            }
        }
    }
    catch(std::logic_error& ex){
        os << formatR(_("{0} at line {1} of \"{2}\"."), ex.what(), lineNumber, filename) << endl;
        return false;
    }

    if(csvFormat == XYZMMRPYDEG){
        for(auto& xyzrpy : data){
            auto tag = new PositionTag;
            tag->setTranslation(xyzrpy.head<3>() / 1000.0);
            tag->setRotation(rotFromRpy(radian(xyzrpy.tail<3>())));
            append(tag);
        }
    } else if(csvFormat == XYZMM){
        for(auto& xyzrpy : data){
            auto tag = new PositionTag;
            tag->setTranslation(xyzrpy.head<3>() / 1000.0);
            append(tag);
        }
    } else {
        os << _("Unsupported CSV format is specified.") << endl;
        return false;
    }
        
    return true;
}
