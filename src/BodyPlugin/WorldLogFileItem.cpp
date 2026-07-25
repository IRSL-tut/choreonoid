#include "WorldLogFileItem.h"
#include "SimulatorItem.h"
#include "SubSimulatorItem.h"
#include "ControllerItem.h"
#include <cnoid/MainWindow>
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/ProjectManager>
#include <cnoid/ProjectPacker>
#include <cnoid/RootItem>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/SceneItem>
#include <cnoid/FolderItem>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/TimeSyncItemEngine>
#include <cnoid/Timer>
#include <cnoid/PutPropertyFunction>
#include <cnoid/FileDialog>
#include <cnoid/Archive>
#include <cnoid/MessageOut>
#include <cnoid/UTF8>
#include <cnoid/Format>
#include <filesystem>
#include <QDateTime>
#include <QElapsedTimer>
#include <QMessageBox>
#include <fstream>
#include <stack>
#include <map>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

static const int frameHeaderSize =
      sizeof(int)   // offset to the prev frame
    + sizeof(float) // time
    + sizeof(int)   // data size
    ;

enum DataTypeID {
    BODY_STATE,
    LINK_POSITIONS,
    JOINT_POSITIONS,
    DEVICE_STATES
};

struct CorruptLogException { };

class ReadBuf
{
public:
    vector<char> data;
    ifstream& ifs;
    int pos;

    ReadBuf(ifstream& ifs)
        : ifs(ifs) {
        pos = 0;
    }

    bool checkSize(int size){
        int left = data.size() - pos;
        if(left < size){
            int len = size - left;
            data.resize(data.size() + len);
            ifs.read(&data[pos], len);
            if(!ifs.fail()){
                return true;
            } else {
                ifs.clear();
                return false;
            }
        }
        return true;
    }

    void ensureSize(int size){
        if(!checkSize(size)){
            throw CorruptLogException();
        }
    }

    void seekToNextBlock(){
        int size = readSeekOffset();
        seek(pos + size);
    }

    int readNextBlockPos(){
        int size = readSeekOffset();
        return pos + size;
    }

    char* buf() {
        return &data.front();
    }

    void clear(){
        data.clear();
        pos = 0;
    }

    int size() const {
        return data.size();
    }

    char* current() {
        return &data[pos];
    }

    char* end() {
        return &data.front() + data.size();
    }

    bool isEnd() {
        return (pos >= static_cast<int>(data.size()));
    }

    void seek(int pos = 0) { this->pos = pos; }

    char readID(){
        ensureSize(1);
        return data[pos++];
    }

    bool readBool(){
        ensureSize(1);
        return data[pos++];
    }

    char readOctet(){
        ensureSize(1);
        return data[pos++];
    }

    short readShort(){
        ensureSize(2);
        unsigned char low = data[pos++];
        unsigned char high = data[pos++];
        short value = low + (high << 8);
        return value;
    }

    int readInt(){
        ensureSize(4);
        unsigned char d0 = data[pos++];
        unsigned char d1 = data[pos++];
        unsigned char d2 = data[pos++];
        unsigned char d3 = data[pos++];
        int value = d0 + (d1 << 8) + (d2 << 16) + (d3 << 24);
        return value;
    }

    int readSeekOffset(){
        int offset = readInt();
        if(offset < 0){
            throw CorruptLogException();
        }            
        return offset;
    }

    float readFloat(){
        ensureSize(sizeof(float));
        float value;
        char* p = (char*)&value;
        const int n = sizeof(float);
        for(int i=0; i < n; ++i){
            p[i] = data[pos++];
        }
        return value;
    }

    SE3 readSE3(){
        SE3 position;
        Vector3& p = position.translation();
        p.x() = readFloat();
        p.y() = readFloat();
        p.z() = readFloat();
        Quaternion& q = position.rotation();
        q.w() = readFloat();
        q.x() = readFloat();
        q.y() = readFloat();
        q.z() = readFloat();
        return position;
    }

    std::string readString(){
        ensureSize(2);
        const int size = readShort();
        if(size < 0){
            throw CorruptLogException();
        }
        ensureSize(size);
        std::string str;
        str.reserve(size);
        for(int i=0; i < size; ++i){
            str.append(1, data[pos++]);
        }
        return str;
    }
};


class WriteBuf
{
public:
    vector<char> data;
    ofstream& ofs;
    size_t seekOffset;

    WriteBuf(ofstream& ofs)
        : ofs(ofs) {
        seekOffset = 0;
    }
    
    char* buf() {
        return &data.front();
    }

    size_t pos() {
        return data.size();
    }

    size_t seekPos() {
        return seekOffset + data.size();
    }

    void clear(){
        data.clear();
        seekOffset = ofs.tellp();
    }

    int size() const {
        return data.size();
    }

    void flush(){
        ofs.write(&data.front(), data.size());
        ofs.flush();
        clear();
    }
        
    void writeID(DataTypeID id){
        writeOctet((char)id);
    }

    void writeBool(bool value){
        data.push_back(value);
    }

    void writeOctet(char value){
        data.push_back(value);
    }

    void writeShort(short value){
        data.push_back(value & 0xff);
        data.push_back(value >> 8);
    }

    void writeInt(int value){
        data.push_back(value & 0xff);
        data.push_back((value >> 8) & 0xff);
        data.push_back((value >> 16) & 0xff);
        data.push_back((value >> 24) & 0xff);
    }

    void writeInt(int pos, int value){
        data[pos++] = value & 0xff;
        data[pos++] = (value >> 8) & 0xff;
        data[pos++] = (value >> 16) & 0xff;
        data[pos++] = (value >> 24) & 0xff;
    }

    void writeSeekPos(int pos){
        writeInt(pos);
    }

    void writeSeekOffset(int offset){
        writeInt(offset);
    }

    void writeSeekOffset(int pos, int offset){
        writeInt(pos, offset);
    }
    
    void writeFloat(float value){
        char* p = (char*)&value;
        const int n = sizeof(float);
        for(int i=0; i < n; ++i){
            data.push_back(p[i]);
        }
    }

    void writeSE3(const SE3& position){
        const Vector3& p = position.translation();
        writeFloat(p.x());
        writeFloat(p.y());
        writeFloat(p.z());
        const Quaternion& q = position.rotation();
        writeFloat(q.w());
        writeFloat(q.x());
        writeFloat(q.y());
        writeFloat(q.z());
    }

    void writeString(const std::string& str){
        const int size = str.size();
        data.reserve(data.size() + size + 1);
        writeShort((unsigned char)size);
        for(int i=0; i < size; ++i){
            writeOctet(str[i]);
        }
    }
};


class DeviceInfo {
public:
    size_t lastStateSeekPos;
    vector<double> lastState;
    bool isConsistent;
    DeviceInfo() {
        lastStateSeekPos = 0;
        isConsistent = false;
    }
};


class BodyInfo : public Referenced
{
public:
    BodyItem* bodyItem;
    Body* body;
    vector<DeviceInfo> deviceInfos;
    
    BodyInfo(BodyItem* bodyItem){
        this->bodyItem = bodyItem;
        if(bodyItem){
            body = bodyItem->body();
            deviceInfos.resize(body->numDevices());
        } else {
            body = nullptr;
        }
    }
    
    DeviceInfo& deviceInfo(int deviceIndex){
        if(deviceIndex >= static_cast<int>(deviceInfos.size())){
            deviceInfos.resize(deviceIndex + 1);
        }
        return deviceInfos[deviceIndex];
    }
};
typedef ref_ptr<BodyInfo> BodyInfoPtr;


ItemList<BodyItem>::iterator findItemOfName(ItemList<BodyItem>& items, const std::string& name)
{
    for(ItemList<BodyItem>::iterator p = items.begin(); p != items.end(); ++p){
        if((*p)->name() == name){
            return p;
        }
    }
    return items.end();
}


class WorldLogFileEngine : public TimeSyncItemEngine
{
public:
    WorldLogFileItem* logItem;
    WorldLogFileEngine(WorldLogFileItem* item);
    virtual bool onTimeChanged(double time) override;
    virtual double onPlaybackStopped(double time, bool isStoppedManually) override;
};
typedef ref_ptr<WorldLogFileEngine> WorldLogFileEnginePtr;

/**
   The packer to save the current project as a log playback archive.
   The model files and the log file are copied into the archive by the
   ProjectPacker functions, and the items only used for executing a
   simulation are removed from the archived project.
*/
class LogPlaybackArchivePacker : public ProjectPacker
{
public:
    LogPlaybackArchivePacker(WorldLogFileItem* logFileItem, const std::string& logFile);
    bool analyzeItemTree();

protected:
    virtual void getItemDependentFiles(Item* item, std::vector<std::string>& out_files) override;
    virtual Item* getPackingItem(Item* item) override;

private:
    enum Disposition { Keep, ReplaceWithFolder, Remove };

    int analyzeSubTree(Item* item);
    void checkRosPackageDirectoryConsistency(Item* item, const std::vector<std::string>& files);

    WorldLogFileItem* logFileItem;
    string logFile;
    map<Item*, Disposition> dispositionMap;
    int numModelItems;
};

}

namespace cnoid {

class WorldLogFileItem::Impl
{
public:
    WorldLogFileItem* self;
    QDateTime recordingStartTime;
    bool isTimeStampSuffixEnabled;
    vector<string> bodyNames;
    
    ofstream ofs;
    WriteBuf writeBuf;
    size_t lastOutputFramePos;
    double recordingFrameRate;
    stack<int> sizeHeaderStack;

    // for device state recording and playback
    struct DeviceStateCache : public Referenced {
        DeviceStatePtr state;
        // This position is stored as a 4-byte int in the log file for format compatibility,
        // so cached device state references may overflow for files exceeding 2GB.
        // This is not a problem for devices whose state changes every frame (no cache hit).
        size_t seekPos;
    };
    typedef ref_ptr<DeviceStateCache> DeviceStateCachePtr;
    
    vector<DeviceStateCachePtr> deviceStateCacheArrays[2];
    vector<DeviceStateCachePtr>* pLastDeviceStateCacheArray;
    vector<DeviceStateCachePtr>* pCurrentDeviceStateCacheArray;
    int deviceIndex;
    int numDeviceStateCaches;
    int currentDeviceStateCacheArrayIndex;
    vector<double> doubleWriteBuf;

    filesystem::path readFilePath;
    ifstream ifs;
    ReadBuf readBuf;
    ReadBuf readBuf2;
    size_t currentReadFramePos;
    size_t currentReadFrameDataSize;
    size_t prevReadFrameOffset;
    double currentReadFrameTime;
    bool isCurrentFrameDataLoaded;
    bool isOverRange;

    vector<BodyInfoPtr> bodyInfos;
    ScopedConnection worldSubTreeChangedConnection;
    bool isBodyInfoUpdateNeeded;

    WorldLogFileEnginePtr logEngine;
    Timer* livePlaybackTimer;
    size_t livePlaybackLastFramePos;
    std::uintmax_t livePlaybackLogFileSize;
    int livePlaybackReadInterval; // msec
    double livePlaybackReadTimeout; // sec
    QElapsedTimer livePlaybackReadTimeoutTimer;
    bool doCheckLivePlaybackReadTimeout;

    MessageOut* mout;
    
    Impl(WorldLogFileItem* self);
    Impl(WorldLogFileItem* self, Impl& org);
    ~Impl();
    bool setLogFile(const std::string& name, bool isLoading = false);
    string getActualFilename();
    void updateBodyInfos();
    void onWorldSubTreeChanged();
    bool readTopHeader();
    bool readFrameHeader(size_t pos);
    bool seek(double time);
    bool seekToLivePlaybackLastFrame();
    bool loadCurrentFrameData();
    bool recallStateAtTime(double time);
    void readBodyStates(double time);
    void readBodyState(BodyInfo* bodyInfo, double time);
    int readLinkPositions(Body* body);
    int readJointPositions(Body* body);
    void readDeviceStates(BodyInfo* bodyInfo, double time);
    void readDeviceState(DeviceInfo& devInfo, Device* device, ReadBuf& buf, int size);
    void readLastDeviceState(DeviceInfo& devInfo, Device* device);
    void clearOutput();
    void reserveSizeHeader();
    void fixSizeHeader();
    void endHeaderOutput();
    void beginFrameOutput(double time);
    void outputDeviceState(DeviceState* state);
    void exchangeDeviceStateCacheArrays();
    void showPlaybackArchiveSaveDialog();
    void saveProjectAsPlaybackArchive(const string& filename);
    WorldLogFileEngine* getOrCreateLogEngine();
    bool setLivePlaybackReadInterval(int interval);
    bool setLivePlaybackReadTimeout(double timeout);
    void startLivePlayback();
    void stopLivePlayback();    
    void onLivePlaybackTimerTimeOut();
};

}


void WorldLogFileItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<WorldLogFileItem>(N_("WorldLogFileItem"));
    im.addCreationPanel<WorldLogFileItem>();
    auto loadLogFile =
        [](WorldLogFileItem* item, const std::string& filename, std::ostream&, Item*){
            return item->impl->setLogFile(filename, true);
        };
    im.addLoader<WorldLogFileItem>(_("World Log"), "CNOID-WORLD-LOG", "log", loadLogFile);

    // A log file does not necessarily have the ".log" extension. The following
    // registration shares the caption with the above one so that it appears as
    // an additional name filter for loading a log file with any extension in
    // the same file dialog.
    im.addLoader<WorldLogFileItem>(_("World Log"), "CNOID-WORLD-LOG", "*", loadLogFile);

    ItemTreeView::customizeContextMenu<WorldLogFileItem>(
        [](WorldLogFileItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.setPath("/");
            menuManager.addItem(_("Start Live Playback"))->sigTriggered().connect(
                [item]{ item->impl->startLivePlayback(); });
            menuManager.addItem(_("Save project as log playback archive"))->sigTriggered().connect(
                [item](){ item->impl->showPlaybackArchiveSaveDialog(); });
            menuManager.setPath("/");
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });

    TimeSyncItemEngineManager::instance()
        ->registerFactory<WorldLogFileItem, WorldLogFileEngine>(
            [](WorldLogFileItem* item, WorldLogFileEngine* /* engine0 */){
                return item->impl->getOrCreateLogEngine();
            });
}


WorldLogFileItem::WorldLogFileItem()
{
    impl = new Impl(this);
    setName("WorldLogFile");
}


WorldLogFileItem::Impl::Impl(WorldLogFileItem* self)
    : self(self),
      writeBuf(ofs),
      readBuf(ifs),
      readBuf2(ifs)
{
    isTimeStampSuffixEnabled = false;
    recordingFrameRate = 0.0;
    isBodyInfoUpdateNeeded = true;
    livePlaybackTimer = nullptr;
    livePlaybackLogFileSize = 0;
    livePlaybackReadInterval = 10;
    livePlaybackReadTimeout = 0.0;
    mout = MessageOut::master();
}


WorldLogFileItem::WorldLogFileItem(const WorldLogFileItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


WorldLogFileItem::Impl::Impl(WorldLogFileItem* self, Impl& org)
    : self(self),
      writeBuf(ofs),
      readBuf(ifs),
      readBuf2(ifs)
{
    isTimeStampSuffixEnabled = org.isTimeStampSuffixEnabled;
    recordingFrameRate = org.recordingFrameRate;
    currentReadFramePos = 0;
    isBodyInfoUpdateNeeded = true;
    livePlaybackTimer = nullptr;
    livePlaybackLogFileSize = 0;
    livePlaybackReadInterval = org.livePlaybackReadInterval;
    livePlaybackReadTimeout = org.livePlaybackReadTimeout;
    mout = MessageOut::master();
}


WorldLogFileItem::~WorldLogFileItem()
{
    delete impl;
}


WorldLogFileItem::Impl::~Impl()
{
    if(livePlaybackTimer){
        delete livePlaybackTimer;
        livePlaybackTimer = nullptr;
    }
}


Item* WorldLogFileItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new WorldLogFileItem(*this);
}


void WorldLogFileItem::notifyUpdate()
{
    impl->isBodyInfoUpdateNeeded = true;
    Item::notifyUpdate();
}


const std::string& WorldLogFileItem::logFile() const
{
    return filePath();
}


bool WorldLogFileItem::setLogFile(const std::string& filename)
{
    return impl->setLogFile(filename);
}


bool WorldLogFileItem::Impl::setLogFile(const std::string& filename, bool isLoading)
{
    self->updateFileInformation(filename, "CNOID-WORLD-LOG");
    bool loaded = readTopHeader();
    return isLoading ? loaded : true;
}


void WorldLogFileItem::setTimeStampSuffixEnabled(bool on)
{
    impl->isTimeStampSuffixEnabled = on;
}


bool WorldLogFileItem::isTimeStampSuffixEnabled() const
{
    return impl->isTimeStampSuffixEnabled;
}


string WorldLogFileItem::Impl::getActualFilename()
{
    if(isTimeStampSuffixEnabled && recordingStartTime.isValid()){
        filesystem::path filepath(fromUTF8(self->filePath()));
        string suffix = recordingStartTime.toString("-yyyy-MM-dd-hh-mm-ss").toStdString();
        string fname = filepath.stem().string() + suffix;
        fname += filepath.extension().string();
        return toUTF8((filepath.parent_path() / fname).generic_string());
    } else {
        return self->filePath();
    }
}


void WorldLogFileItem::setRecordingFrameRate(double rate)
{
    impl->recordingFrameRate = rate;
}


double WorldLogFileItem::recordingFrameRate() const
{
    return impl->recordingFrameRate;
}


void WorldLogFileItem::Impl::updateBodyInfos()
{
    bodyInfos.clear();
    
    if(!bodyNames.empty()){
        if(auto worldItem = self->findOwnerItem<WorldItem>()){
            auto items = worldItem->descendantItems<BodyItem>();
            if(!items.empty()){
                for(size_t i=0; i < bodyNames.size(); ++i){
                    ItemList<BodyItem>::iterator p = findItemOfName(items, bodyNames[i]);
                    if(p != items.end()){
                        bodyInfos.push_back(new BodyInfo(*p));
                        items.erase(p);
                    } else {
                        bodyInfos.push_back(0);
                    }
                }
            }
        }
    }

    isBodyInfoUpdateNeeded = false;
}


void WorldLogFileItem::onTreePathChanged()
{
    WorldItem* worldItem = findOwnerItem<WorldItem>();
    if(!worldItem){
        impl->worldSubTreeChangedConnection.disconnect();
    } else {
        impl->worldSubTreeChangedConnection.reset(
            worldItem->sigSubTreeChanged().connect(
                [&](){ impl->onWorldSubTreeChanged(); }));
    }
    
    impl->isBodyInfoUpdateNeeded = true;
}


void WorldLogFileItem::Impl::onWorldSubTreeChanged()
{
    isBodyInfoUpdateNeeded = true;
}


bool WorldLogFileItem::Impl::readTopHeader()
{
    bool result = false;
    
    bodyNames.clear();

    currentReadFramePos = 0;
    currentReadFrameDataSize = 0;
    prevReadFrameOffset = 0;
    currentReadFrameTime = -1.0;
    
    if(ifs.is_open()){
        ifs.close();
    }
    readFilePath = filesystem::path(fromUTF8(getActualFilename()));
    std::error_code ec;

    // An empty log file is not corrupt; it is a log file whose data has not been
    // recorded or transferred yet, and it is just treated as a log with no data
    if(filesystem::exists(readFilePath, ec) && filesystem::file_size(readFilePath, ec) > 0){
        ifs.open(readFilePath.string(), ios::in | ios::binary);
        if(ifs.is_open()){
            readBuf.clear();
            try {
                int headerSize = readBuf.readSeekOffset();
                if(readBuf.checkSize(headerSize)){
                    while(!readBuf.isEnd()){
                        bodyNames.push_back(readBuf.readString());
                    }
                    currentReadFramePos = readBuf.pos;
                    result = readFrameHeader(readBuf.pos);
                }
            } catch(CorruptLogException&){
                bodyNames.clear();
                MessageOut::master()->putErrorln(
                    formatR(_("Log file of {0} is corrupt."), self->displayName()));
            }
        }
    }

    isBodyInfoUpdateNeeded = true;

    return result;
}


bool WorldLogFileItem::Impl::readFrameHeader(size_t pos)
{
    isCurrentFrameDataLoaded = false;
    
    if(!ifs.is_open()){
        return false;
    }

    ifs.seekg(pos);

    if(ifs.eof()){
        ifs.seekg(currentReadFramePos);
        return false;
    }

    readBuf.clear();
    if(!readBuf.checkSize(frameHeaderSize)){
        ifs.seekg(currentReadFramePos);
        return false;
    }

    int prevFrameOffset;
    double frameTime;
    int dataSize;
    try {
        prevFrameOffset = readBuf.readSeekOffset();
        frameTime = readBuf.readFloat();
        dataSize = readBuf.readSeekOffset();
    } catch(CorruptLogException&){
        bodyNames.clear();
        mout->putErrorln(formatR(_("Log file of {0} is corrupt."), self->displayName()));
        ifs.seekg(currentReadFramePos);
        return false;
    }

    if(!readBuf.checkSize(currentReadFrameDataSize)){
        ifs.seekg(currentReadFramePos);
        return false;
    }
    
    currentReadFramePos = pos;
    prevReadFrameOffset = prevFrameOffset;
    currentReadFrameTime = frameTime;
    currentReadFrameDataSize = dataSize;

    return true;
}
        
        
bool WorldLogFileItem::Impl::seek(double time)
{
    isOverRange = false;

    if(!readFrameHeader(currentReadFramePos)){
        readTopHeader();
    }
    
    if(currentReadFrameTime == time){
        return true;
    }

    if(currentReadFrameTime < time){
        while(true){
            size_t pos = currentReadFramePos;
            if(!readFrameHeader(currentReadFramePos + frameHeaderSize + currentReadFrameDataSize)){
                isOverRange = true;
                return (currentReadFrameTime >= 0.0);
            }
            if(currentReadFrameTime == time){
                return true;
            } else if(currentReadFrameTime > time){
                return readFrameHeader(pos);
            }
        }
    }

    // currentReadFrameTime > time
    while(true){
        if(prevReadFrameOffset <= 0){
            isOverRange = true;
            return (currentReadFrameTime >= 0.0);
        }
        if(!readFrameHeader(currentReadFramePos - prevReadFrameOffset)){
            return false;
        }
        if(currentReadFrameTime <= time){
            return true;
        }
    }
}


bool WorldLogFileItem::Impl::loadCurrentFrameData()
{
    ifs.seekg(currentReadFramePos + frameHeaderSize);
    readBuf.clear();
    isCurrentFrameDataLoaded = readBuf.checkSize(currentReadFrameDataSize);
    return isCurrentFrameDataLoaded;
}


/**
   @return True if the time is within the data range and the frame is correctly recalled.
   False if the time is outside the data range or the frame cannot be recalled.
*/
bool WorldLogFileItem::recallStateAtTime(double time)
{
    return impl->recallStateAtTime(time);
}


bool WorldLogFileItem::Impl::recallStateAtTime(double time)
{
    bool isValid = false;
    
    try {
        if(seek(time)){
            if(isCurrentFrameDataLoaded || loadCurrentFrameData()){
                readBuf.seek(0);
                if(isBodyInfoUpdateNeeded){
                    updateBodyInfos();
                }
                readBodyStates(time);
                isValid = !isOverRange;
            }
        }
    }
    catch(CorruptLogException&){
        mout->putErrorln(formatR(_("Corrupt log at time {0} in {1}."), time, self->displayName()));
    }

    return isValid;
}


void WorldLogFileItem::Impl::readBodyStates(double time)
{
    int bodyIndex = 0;
    while(!readBuf.isEnd()){
        int dataTypeID = readBuf.readID();
        switch(dataTypeID){
        case BODY_STATE:
        {
            BodyInfo* bodyInfo = nullptr;
            if(bodyIndex < static_cast<int>(bodyInfos.size())){
                bodyInfo = bodyInfos[bodyIndex];
            }
            if(bodyInfo){
                readBodyState(bodyInfo, time);
            } else {
                readBuf.seekToNextBlock();
            }
            ++bodyIndex;
            break;
        }
        default:
            readBuf.seekToNextBlock();
        }
    }
}


void WorldLogFileItem::Impl::readBodyState(BodyInfo* bodyInfo, double time)
{
    int endPos = readBuf.readNextBlockPos();
    bool updated = false;
    bool doForwardKinematics = true;
    int numLinks;
    
    while(readBuf.pos < endPos){
        int dataType = readBuf.readID();
        switch(dataType){
        case LINK_POSITIONS:
            numLinks = readLinkPositions(bodyInfo->body);
            if(numLinks > 0){
                updated = true;
                if(numLinks > 1){
                    doForwardKinematics = false;
                }
            }
            break;
        case JOINT_POSITIONS:
            if(readJointPositions(bodyInfo->body)){
                updated = true;
            }
            break;
        case DEVICE_STATES:
            if(updated){
                bodyInfo->bodyItem->notifyKinematicStateChange(doForwardKinematics);
                updated = false;
            }
            readDeviceStates(bodyInfo, time);
            break;
        default:
            readBuf.seekToNextBlock();
            break;
        }
    }
    if(updated){
        bodyInfo->bodyItem->notifyKinematicStateChange(doForwardKinematics);
    }
}


int WorldLogFileItem::Impl::readLinkPositions(Body* body)
{
    int endPos = readBuf.readNextBlockPos();
    int size = readBuf.readShort();
    int n = std::min(size, body->numLinks());
    for(int i=0; i < n; ++i){
        SE3 position = readBuf.readSE3();
        Link* link = body->link(i);
        link->p() = position.translation();
        link->R() = position.rotation().toRotationMatrix();
    }
    readBuf.seek(endPos);
    return n;
}


int WorldLogFileItem::Impl::readJointPositions(Body* body)
{
    int endPos = readBuf.readNextBlockPos();
    int size = readBuf.readShort();
    int n = std::min(size, body->numAllJoints());
    for(int i=0; i < n; ++i){
        body->joint(i)->q() = readBuf.readFloat();
    }
    readBuf.seek(endPos);
    return n;
}


void WorldLogFileItem::Impl::readDeviceStates(BodyInfo* bodyInfo, double time)
{
    const int endPos = readBuf.readNextBlockPos();
    Body* body = bodyInfo->body;
    const int numDevices = body->numDevices();
    int deviceIndex = 0;
    while(readBuf.pos < endPos && deviceIndex < numDevices){
        DeviceInfo& devInfo = bodyInfo->deviceInfo(deviceIndex);
        Device* device = bodyInfo->body->device(deviceIndex);
        const int header = readBuf.readShort();
        if(header < 0){
            readLastDeviceState(devInfo, device);
        } else {
            const int size = header;
            int nextPos = readBuf.pos + sizeof(float) * size;
            readDeviceState(devInfo, device, readBuf, size);
            readBuf.seek(nextPos);
        }
        device->notifyTimeChange(time);
        ++deviceIndex;
    }
    readBuf.seek(endPos);
}


void WorldLogFileItem::Impl::readDeviceState(DeviceInfo& devInfo, Device* device, ReadBuf& buf, int size)
{
    const int stateSize = device->stateSize();
    vector<double>& state = devInfo.lastState;
    state.resize(stateSize);
    for(int i=0; i < stateSize; ++i){
        state[i] = buf.readFloat();
    }
    device->readState(&state.front(), size);
    device->notifyStateChange();
    devInfo.isConsistent = true;
}


void WorldLogFileItem::Impl::readLastDeviceState(DeviceInfo& devInfo, Device* device)
{
    size_t pos = readBuf.readSeekOffset();
    if(pos == devInfo.lastStateSeekPos){
        if(!devInfo.isConsistent){
            auto& lastState = devInfo.lastState;
            device->readState(lastState.data(), lastState.size());
            device->notifyStateChange();
            devInfo.isConsistent = true;
        }
    } else {
        ifs.seekg(pos);
        devInfo.lastStateSeekPos = pos;
        readBuf2.clear();
        int size = readBuf2.readShort();
        if(size > 0){
            readDeviceState(devInfo, device, readBuf2, size);
        }
    }
}


void WorldLogFileItem::invalidateLastStateConsistency()
{
    vector<BodyInfoPtr>& bodyInfos = impl->bodyInfos;
    for(size_t i=0; i < bodyInfos.size(); ++i){
        vector<DeviceInfo>& devInfos = bodyInfos[i]->deviceInfos;
        for(size_t j=0; j < devInfos.size(); ++j){
            devInfos[j].isConsistent = false;
        }
    }
}


void WorldLogFileItem::clearOutput()
{
    impl->clearOutput();
}


void WorldLogFileItem::Impl::clearOutput()
{
    bodyNames.clear();

    if(ifs.is_open()){
        ifs.close();
    }
    if(ofs.is_open()){
        ofs.close();
    }
    recordingStartTime = QDateTime::currentDateTime();
    
    ofs.open(fromUTF8(getActualFilename()).c_str(), ios::out | ios::binary | ios::trunc);
    writeBuf.clear();
    lastOutputFramePos = 0;

    currentDeviceStateCacheArrayIndex = 0;
    exchangeDeviceStateCacheArrays();
}


void WorldLogFileItem::Impl::reserveSizeHeader()
{
    sizeHeaderStack.push(writeBuf.size());
    writeBuf.writeSeekOffset(0);
}


void WorldLogFileItem::Impl::fixSizeHeader()
{
    if(!sizeHeaderStack.empty()){
        writeBuf.writeSeekOffset(sizeHeaderStack.top(), writeBuf.size() - (sizeHeaderStack.top() + sizeof(int)));
        sizeHeaderStack.pop();
    }
}


void WorldLogFileItem::beginHeaderOutput()
{
    impl->writeBuf.clear();
    impl->reserveSizeHeader();
}


int WorldLogFileItem::outputBodyHeader(const std::string& name)
{
    int index = impl->bodyNames.size();
    impl->bodyNames.push_back(name);
    impl->writeBuf.writeString(name);
    return index;
}


void WorldLogFileItem::endHeaderOutput()
{
    impl->endHeaderOutput();
}


void WorldLogFileItem::Impl::endHeaderOutput()
{
    fixSizeHeader();
    writeBuf.flush();
}


int WorldLogFileItem::numBodies() const
{
    return impl->bodyNames.size();
}


const std::string& WorldLogFileItem::bodyName(int bodyIndex) const
{
    return impl->bodyNames[bodyIndex];
}


void WorldLogFileItem::beginFrameOutput(double time)
{
    impl->beginFrameOutput(time);
}


void WorldLogFileItem::Impl::beginFrameOutput(double time)
{
    size_t pos = writeBuf.seekPos();
    
    if(lastOutputFramePos){
        writeBuf.writeSeekOffset(pos - lastOutputFramePos);
    } else {
        writeBuf.writeSeekOffset(0);
    }
    lastOutputFramePos = pos;
    
    deviceIndex = 0;
    writeBuf.writeFloat(time);
    reserveSizeHeader(); // area for the frame data size
}


void WorldLogFileItem::beginBodyStateOutput()
{
    impl->writeBuf.writeID(BODY_STATE);
    impl->reserveSizeHeader();
}


void WorldLogFileItem::outputLinkPositions(double* positions, int numLinkPositions)
{
    impl->writeBuf.writeID(LINK_POSITIONS);
    impl->reserveSizeHeader();
    impl->writeBuf.writeShort(numLinkPositions);
    for(int i=0; i < numLinkPositions; ++i){
        impl->writeBuf.writeFloat(positions[0]); // x
        impl->writeBuf.writeFloat(positions[1]); // y
        impl->writeBuf.writeFloat(positions[2]); // z
        impl->writeBuf.writeFloat(positions[6]); // qw
        impl->writeBuf.writeFloat(positions[3]); // qx
        impl->writeBuf.writeFloat(positions[4]); // qy
        impl->writeBuf.writeFloat(positions[5]); // qz
        positions += 7;
    }
    impl->fixSizeHeader();
}    


void WorldLogFileItem::outputJointPositions(double* values, int size)
{
    impl->writeBuf.writeID(JOINT_POSITIONS);
    impl->reserveSizeHeader();
    impl->writeBuf.writeShort(size);
    for(int i=0; i < size; ++i){
        impl->writeBuf.writeFloat(values[i]);
    }
    impl->fixSizeHeader();
}


void WorldLogFileItem::beginDeviceStateOutput()
{
    impl->writeBuf.writeID(DEVICE_STATES);
    impl->reserveSizeHeader();
}


void WorldLogFileItem::outputDeviceState(DeviceState* state)
{
    impl->outputDeviceState(state);
}


void WorldLogFileItem::Impl::outputDeviceState(DeviceState* state)
{
    DeviceStateCache* cache = nullptr;
        
    if(deviceIndex >= numDeviceStateCaches){
        cache = new DeviceStateCache;
    } else {
        cache = (*pLastDeviceStateCacheArray)[deviceIndex];
        if(state == cache->state){
            writeBuf.writeShort(-1);
            writeBuf.writeSeekOffset(cache->seekPos);
            goto endOutputDeviceState;
        }
    }
    cache->state = state;
    cache->seekPos = writeBuf.seekPos();
    if(!state){
        writeBuf.writeShort(0);
    } else {
        int size = state->stateSize();
        writeBuf.writeShort(size);
        doubleWriteBuf.resize(size);
        state->writeState(&doubleWriteBuf.front());
        for(int i=0; i < size; ++i){
            writeBuf.writeFloat(doubleWriteBuf[i]);
        }
    }
endOutputDeviceState:

    pCurrentDeviceStateCacheArray->push_back(cache);
    ++deviceIndex;
}


void WorldLogFileItem::endDeviceStateOutput()
{
    impl->fixSizeHeader();
}


void WorldLogFileItem::endBodyStateOutput()
{
    impl->fixSizeHeader();
}


void WorldLogFileItem::endFrameOutput()
{
    impl->fixSizeHeader();
    impl->writeBuf.flush();
    impl->exchangeDeviceStateCacheArrays();
}


void WorldLogFileItem::Impl::exchangeDeviceStateCacheArrays()
{
    int i = 1 - currentDeviceStateCacheArrayIndex;
    pCurrentDeviceStateCacheArray = &deviceStateCacheArrays[i];
    pLastDeviceStateCacheArray = &deviceStateCacheArrays[1-i];
    numDeviceStateCaches = pLastDeviceStateCacheArray->size();
    currentDeviceStateCacheArrayIndex = i;
}
    

void WorldLogFileItem::doPutProperties(PutPropertyFunction& putProperty)
{
    FilePathProperty logFileProperty(filePath(), { string(_("World Log File (*.log)")) });
    logFileProperty.setExistingFileMode(false);
    putProperty(_("Log file"), logFileProperty,
                [this](const string& file){ return impl->setLogFile(file); });
    putProperty(_("Actual log file"), FilePathProperty(impl->getActualFilename()));
    putProperty(_("Time-stamp suffix"), impl->isTimeStampSuffixEnabled,
                changeProperty(impl->isTimeStampSuffixEnabled));
    putProperty(_("Recording frame rate"), impl->recordingFrameRate,
                changeProperty(impl->recordingFrameRate));
    putProperty.min(1)(_("Live playback read interval (ms)"), impl->livePlaybackReadInterval,
                       [this](int value){ return setLivePlaybackReadInterval(value); });
    putProperty.min(0.0)(_("Live playback read timeout"), impl->livePlaybackReadTimeout,
                         [this](double value){ return setLivePlaybackReadTimeout(value); });
}


bool WorldLogFileItem::store(Archive& archive)
{
    archive.writeFileInformation(this);
    archive.write("time_stamp_suffix", impl->isTimeStampSuffixEnabled);
    archive.write("recording_frame_rate", impl->recordingFrameRate);
    archive.write("live_playback_read_interval_ms", impl->livePlaybackReadInterval);
    archive.write("live_playback_read_timeout", impl->livePlaybackReadTimeout);
    return true;
}


bool WorldLogFileItem::restore(const Archive& archive)
{
    archive.read({"time_stamp_suffix", "timeStampSuffix" }, impl->isTimeStampSuffixEnabled);
    archive.read({"recording_frame_rate", "recordingFrameRate" }, impl->recordingFrameRate);
    setLivePlaybackReadInterval(archive.get("live_playback_read_interval_ms", impl->livePlaybackReadInterval));
    setLivePlaybackReadTimeout(archive.get("live_playback_read_timeout", impl->livePlaybackReadTimeout));

    std::string filename;
    if(archive.read({ "file", "filename" }, filename)){
        impl->setLogFile(archive.resolveRelocatablePath(filename));
    }
    return true;
}


void WorldLogFileItem::showPlaybackArchiveSaveDialog()
{
    impl->showPlaybackArchiveSaveDialog();
}


void WorldLogFileItem::Impl::showPlaybackArchiveSaveDialog()
{
    string actualFilename = getActualFilename();
    if(actualFilename.empty()){
        showWarningDialog(formatR(_("The log file of {0} is not specified."), self->displayName()));
        return;
    }
    std::error_code ec;
    auto logFilePath = filesystem::absolute(fromUTF8(actualFilename), ec);
    bool logFileExists = filesystem::exists(logFilePath, ec);
    if(!logFileExists || filesystem::file_size(logFilePath, ec) == 0){
        if(!showWarningDialog(
               formatR(_("The log file of {0} is empty or does not exist, so the archive will "
                         "not contain any recorded log data. Do you want to save the archive "
                         "anyway?"),
                       self->displayName()),
               true)){
            return;
        }
    }

    if(!showWarningDialog(
           _("The current project is temporarily modified to create the archive, and it is "
             "restored by reloading its project file after the archive is saved. Note that "
             "the simulation results recorded in memory are lost by the reloading. "
             "Do you want to continue?"),
           true)){
        return;
    }

    auto pm = ProjectManager::instance();

    // The current project is temporarily modified for the archive during the packing,
    // and it is restored by reloading the project file after the packing. Therefore
    // the current project must have its project file in advance.
    if(pm->currentProjectFile().empty()){
        bool doSaveProject = showWarningDialog(
            _("The current project has not been saved, and it must first be saved to create "
              "the log playback archive. Do you want to save the project and continue?"),
            true);
        if(!doSaveProject || !pm->overwriteCurrentProject()){
            return;
        }
    } else if(!ProjectManager::checkIfItemsConsistentWithProjectArchive()){
        auto clicked = QMessageBox::question(
            MainWindow::instance(), _("Log playback archive"),
            _("The current project has been modified. Do you want to save the project "
              "to restore it after creating the log playback archive?"),
            QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
        if(clicked == QMessageBox::Cancel){
            return;
        }
        if(clicked == QMessageBox::Yes){
            if(!pm->overwriteCurrentProject()){
                return;
            }
        }
    }

    FileDialog dialog;
    dialog.setWindowTitle(_("Save project as log playback archive"));
    dialog.setViewMode(QFileDialog::List);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);

    // The overwrite confirmation is done by the own check below
    dialog.setOption(QFileDialog::DontConfirmOverwrite);

    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    dialog.updatePresetDirectories(true);

    // The archive is a product of the current project, so the directory of the
    // current project file is used as the initial directory
    dialog.setDirectory(pm->currentProjectDirectory());

    dialog.setNameFilter(_("Log playback archive pack (*.zip)"));
    dialog.fileDialog()->setDefaultSuffix("zip");

    auto& projectName = pm->currentProjectName();
    if(!projectName.empty()){
        dialog.selectFile(QString("%1-log.zip").arg(projectName.c_str()));
    }

    if(dialog.exec()){
        auto filenames = dialog.selectedFiles();
        string filename(filenames.at(0).toStdString());
        if(!filename.empty()){
            filesystem::path path(fromUTF8(filename));
            if(path.extension().string() != ".zip"){
                filename += ".zip";
                path = fromUTF8(filename);
            }
            std::error_code ec;
            bool doSave = true;
            if(filesystem::exists(path, ec)){
                doSave = showWarningDialog(
                    formatR(_("\"{0}\" already exists. Do you want to replace it?"),
                            toUTF8(path.filename().string())),
                    true);
            }
            if(doSave){
                saveProjectAsPlaybackArchive(filename);
            }
        }
    }
}


void WorldLogFileItem::saveProjectAsPlaybackArchive(const std::string& projectFile)
{
    impl->saveProjectAsPlaybackArchive(projectFile);
}


void WorldLogFileItem::Impl::saveProjectAsPlaybackArchive(const string& filename)
{
    // Keep this item alive because clearing the project for restoring the original
    // project may release the item during this function execution
    WorldLogFileItemPtr selfHolder = self;

    auto worldItem = self->findOwnerItem<WorldItem>();
    if(!worldItem){
        showWarningDialog(formatR(_("The world item of {0} is not found."), self->displayName()));
        return;
    }

    string actualFilename = getActualFilename();
    if(actualFilename.empty()){
        showWarningDialog(formatR(_("The log file of {0} is not specified."), self->displayName()));
        return;
    }

    std::error_code ec;
    auto logFilePath = filesystem::absolute(fromUTF8(actualFilename), ec);
    bool logFileExists = filesystem::exists(logFilePath, ec);
    if(!logFileExists || filesystem::file_size(logFilePath, ec) == 0){
        mout->putWarningln(
            formatR(_("The log file of {0} is empty or does not exist, so the archive does not "
                      "contain any recorded log data."),
                    self->displayName()));
    }
    if(!logFileExists){
        /*
          An empty log file is created and included in the archive as a placeholder
          so that the log file property of the archived project points to the file
          to be filled later, for example, by the log file transfer for the remote
          live playback.
        */
        ofstream emptyLogFile(logFilePath, ios::out | ios::binary);
        if(!emptyLogFile.is_open()){
            mout->putErrorln(
                formatR(_("The empty log file \"{0}\" cannot be created."),
                        toUTF8(logFilePath.string())));
            return;
        }
    }

    auto pm = ProjectManager::instance();
    string orgProjectFile = pm->currentProjectFile();
    if(orgProjectFile.empty()){
        showWarningDialog(
            _("The current project must be saved as a project file "
              "before saving the log playback archive."));
        return;
    }

    mout->putln(_("Creating the log playback archive ..."));

    LogPlaybackArchivePacker packer(self, toUTF8(logFilePath.generic_string()));
    if(!packer.analyzeItemTree()){
        showWarningDialog(_("There are no model items to be archived for the log playback."));
        return;
    }

    filesystem::path path(fromUTF8(filename));
    string archiveName;
    bool packed;
    if(path.extension().string() == ".zip"){
        archiveName = filename;
        packed = packer.packProjectToZipFile(filename);
    } else {
        if(path.extension().string() == ".cnoid"){
            path.replace_extension(); // the archive directory
        }
        archiveName = toUTF8(path.generic_string());
        packed = packer.packProjectToDirectory(archiveName);
    }

    // The packing modifies the project item tree for the archive, so the original
    // project is restored by reloading its project file.
    pm->clearProject();
    pm->loadProject(orgProjectFile);

    if(packed){
        mout->putln(formatR(_("The log playback archive has been saved as \"{0}\"."), archiveName));
    } else {
        mout->putErrorln(formatR(_("Failed to save the log playback archive \"{0}\"."), archiveName));
    }
}


namespace {

LogPlaybackArchivePacker::LogPlaybackArchivePacker
(WorldLogFileItem* logFileItem, const std::string& logFile)
    : logFileItem(logFileItem),
      logFile(logFile)
{
    numModelItems = 0;
}


bool LogPlaybackArchivePacker::analyzeItemTree()
{
    dispositionMap.clear();
    numModelItems = 0;
    analyzeSubTree(RootItem::instance());
    return numModelItems > 0;
}


int LogPlaybackArchivePacker::analyzeSubTree(Item* item)
{
    if(item->isSubItem()){
        // A sub item is a part of the data of its owner item and is kept with it
        dispositionMap[item] = Keep;
        return 0;
    }

    bool keep = false;
    if(dynamic_cast<BodyItem*>(item) || dynamic_cast<SceneItem*>(item)){
        keep = true;
        ++numModelItems;
    } else if(item == logFileItem){
        keep = true;
    } else if(!item->isTemporary() &&
              item->filePath().empty() &&
              !dynamic_cast<SimulatorItem*>(item) &&
              !dynamic_cast<ControllerItem*>(item)){
        auto subSimulatorItem = dynamic_cast<SubSimulatorItem*>(item);
        if(!subSimulatorItem || subSimulatorItem->isApplicableToLogPlayback()){
            keep = true;
        }
    }

    int numKeptItems = keep ? 1 : 0;
    for(auto childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        numKeptItems += analyzeSubTree(childItem);
    }

    if(keep){
        dispositionMap[item] = Keep;
    } else if(numKeptItems > 0){
        // The item itself is removed but the sub tree structure must be kept
        // for the remaining descendant items
        dispositionMap[item] = ReplaceWithFolder;
    } else {
        dispositionMap[item] = Remove;
    }

    return numKeptItems;
}


void LogPlaybackArchivePacker::getItemDependentFiles(Item* item, std::vector<std::string>& out_files)
{
    auto it = dispositionMap.find(item);
    if(it != dispositionMap.end() && it->second != Keep){
        return;
    }
    if(auto bodyItem = dynamic_cast<BodyItem*>(item)){
        bodyItem->getDependentFiles(out_files);
        checkRosPackageDirectoryConsistency(item, out_files);
    } else if(auto sceneItem = dynamic_cast<SceneItem*>(item)){
        sceneItem->getDependentFiles(out_files);
    } else if(item == logFileItem){
        out_files.push_back(logFile);
    } else {
        ProjectPacker::getItemDependentFiles(item, out_files);
    }
}


/**
   The "package://" references of a model are resolved in a self-contained manner
   only when the referenced packages share the parent directory with the package
   of the model file. If the dependent files of a model belong to the ROS packages
   located in different directory trees, the archive may not be self-contained,
   and a warning message is put in that case.

   \todo This limitation can be resolved by storing the directories of the
   packages bundled in the archive in the archived project file and registering
   them as additional package search paths of the "package://" URI scheme
   handler when the project is loaded.
*/
void LogPlaybackArchivePacker::checkRosPackageDirectoryConsistency
(Item* item, const std::vector<std::string>& files)
{
    vector<filesystem::path> packageParentPaths;
    for(auto& file : files){
        filesystem::path path(fromUTF8(file));
        if(path.filename() == "package.xml"){
            auto parentPath = path.parent_path().parent_path();
            if(std::find(packageParentPaths.begin(), packageParentPaths.end(), parentPath)
               == packageParentPaths.end()){
                packageParentPaths.push_back(parentPath);
            }
        }
    }
    if(packageParentPaths.size() >= 2){
        mout()->putWarningln(
            formatR(_("The files on which {0} depends belong to the ROS packages located in "
                      "different directory trees. The \"package://\" references between those "
                      "packages may not be resolved when the archived project is loaded "
                      "in another environment."),
                    item->displayName()));
    }
}


Item* LogPlaybackArchivePacker::getPackingItem(Item* item)
{
    auto it = dispositionMap.find(item);
    if(it != dispositionMap.end()){
        if(it->second == Remove){
            return nullptr;
        }
        if(it->second == ReplaceWithFolder){
            auto folderItem = new FolderItem;
            folderItem->setName(item->displayName());
            return folderItem;
        }
    }

    auto relocateFilePath = [this](const std::string& path){ return getRelocatedFilePath(path); };

    if(auto bodyItem = dynamic_cast<BodyItem*>(item)){
        bodyItem->relocateDependentFiles(relocateFilePath);
    } else if(auto sceneItem = dynamic_cast<SceneItem*>(item)){
        sceneItem->relocateDependentFiles(relocateFilePath);
    } else if(item == logFileItem){
        auto relocatedLogFile = getRelocatedFilePath(logFile);
        if(relocatedLogFile.empty()){
            mout()->putErrorln(
                formatR(_("Log file \"{0}\" of {1} cannot be relocated to be a file path in the archive."),
                        logFile, item->displayName()));
            return nullptr;
        }
        /*
          The file information is directly updated here instead of using the
          setLogFile function. The setLogFile function opens the relocated log
          file to read its header and keeps the file open, which prevents the
          removal of the temporary packing directory for a zip archive on
          Windows. Just updating the file information is sufficient because it
          is what the project archiving stores, and the state of this item does
          not matter here because the original project is restored by reloading
          it after the packing.
        */
        logFileItem->updateFileInformation(relocatedLogFile, "CNOID-WORLD-LOG");
        logFileItem->setTimeStampSuffixEnabled(false);
        return item;
    }

    return ProjectPacker::getPackingItem(item);
}

}


WorldLogFileEngine* WorldLogFileItem::Impl::getOrCreateLogEngine()
{
    if(!logEngine){
        logEngine = new WorldLogFileEngine(self);
    }
    return logEngine;
}


bool WorldLogFileItem::setLivePlaybackReadInterval(int interval)
{
    return impl->setLivePlaybackReadInterval(interval);
}


bool WorldLogFileItem::Impl::setLivePlaybackReadInterval(int interval)
{
    if(interval >= 1){
        if(interval != livePlaybackReadInterval){
            livePlaybackReadInterval = interval;
            if(livePlaybackTimer){
                bool isActive = livePlaybackTimer->isActive();
                if(isActive){
                    livePlaybackTimer->stop();
                }
                livePlaybackTimer->setInterval(interval);
                if(isActive){
                    livePlaybackTimer->start();
                }
            }
        }
        return true;
    }
    return false;
}


bool WorldLogFileItem::setLivePlaybackReadTimeout(double timeout)
{
    if(timeout >= 0.0){
        impl->livePlaybackReadTimeout = timeout;
        return true;
    }
    return false;
}


void WorldLogFileItem::startLivePlayback()
{
    impl->startLivePlayback();
}


void WorldLogFileItem::Impl::startLivePlayback()
{
    livePlaybackLastFramePos = 0;
    seekToLivePlaybackLastFrame();
    double time = currentReadFrameTime;
    if(time < 0.0){
        time = 0.0;
    }

    getOrCreateLogEngine()->startOngoingTimeUpdate(time);

    if(!livePlaybackTimer){
        livePlaybackTimer = new Timer;
        livePlaybackTimer->sigTimeout().connect(
            [this]{ onLivePlaybackTimerTimeOut(); });
        livePlaybackTimer->setInterval(10);
    }

    doCheckLivePlaybackReadTimeout = false;
    
    livePlaybackTimer->start();
}


void WorldLogFileItem::stopLivePlayback()
{
    impl->stopLivePlayback();
}


void WorldLogFileItem::Impl::stopLivePlayback()
{
    if(livePlaybackTimer){
        livePlaybackTimer->stop();
    }
    if(logEngine){
        logEngine->stopOngoingTimeUpdate();
    }
}


void WorldLogFileItem::Impl::onLivePlaybackTimerTimeOut()
{
    if(seekToLivePlaybackLastFrame()){
        logEngine->updateOngoingTime(currentReadFrameTime);

        if(livePlaybackReadTimeout > 0.0){
            doCheckLivePlaybackReadTimeout = true;
            livePlaybackReadTimeoutTimer.start();
        }
    } else {
        if(doCheckLivePlaybackReadTimeout){
            if(livePlaybackReadTimeoutTimer.elapsed() >= livePlaybackReadTimeout * 1000.0){
                stopLivePlayback();
            }
        }
    }
}


//! \return true if a new frame is found
bool WorldLogFileItem::Impl::seekToLivePlaybackLastFrame()
{
    if(currentReadFramePos == 0){
        if(!readTopHeader()){
            return false;
        }
    }
    size_t framePos = currentReadFramePos;
    while(true){
        if(!readFrameHeader(framePos)){
            break;
        }
        framePos += frameHeaderSize + currentReadFrameDataSize;
    }

    bool hasNewFrames = currentReadFramePos > livePlaybackLastFramePos;
    livePlaybackLastFramePos = currentReadFramePos;
    
    std::error_code ec;
    std::uintmax_t prevFileSize = livePlaybackLogFileSize;
    livePlaybackLogFileSize = filesystem::file_size(readFilePath, ec);
    
    if(!hasNewFrames){
        if(livePlaybackLogFileSize < prevFileSize || prevFileSize < 0){
            readTopHeader();
            return seekToLivePlaybackLastFrame();
        }
    }
    
    return hasNewFrames;
}


WorldLogFileEngine::WorldLogFileEngine(WorldLogFileItem* item)
    : TimeSyncItemEngine(item)
{
    logItem = item;
}


bool WorldLogFileEngine::onTimeChanged(double time)
{
    return logItem->recallStateAtTime(time);
}


double WorldLogFileEngine::onPlaybackStopped(double time, bool /* isStoppedManually */)
{
    if(isUpdatingOngoingTime()){
        logItem->stopLivePlayback();
    }
    return time;
}
