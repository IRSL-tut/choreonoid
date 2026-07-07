#ifndef CNOID_UTIL_VALUE_TREE_H
#define CNOID_UTIL_VALUE_TREE_H

#include "ClonableReferenced.h"
#include "CloneMap.h"
#include <map>
#include <vector>
#include <string>
#include <initializer_list>
#include "exportdecl.h"

namespace cnoid {

class YAMLReaderImpl;
class ValueNode;
class ScalarNode;
class Mapping;
class Listing;

#ifndef CNOID_BACKWARD_COMPATIBILITY
enum StringStyle { PLAIN_STRING, SINGLE_QUOTED, DOUBLE_QUOTED, LITERAL_STRING, FOLDED_STRING };
#else
enum StringStyle { PLAIN_STRING, YAML_PLAIN_STRING = PLAIN_STRING,
                   SINGLE_QUOTED, YAML_SINGLE_QUOTED = SINGLE_QUOTED,
                   DOUBLE_QUOTED, YAML_DOUBLE_QUOTED = DOUBLE_QUOTED,
                   LITERAL_STRING, YAML_LITERAL = LITERAL_STRING,
                   FOLDED_STRING, YAML_FOLDED = FOLDED_STRING
};
#endif

/*
   API conventions on how the value tree node classes return nodes and values.
   The return type tells whether the caller has to check the result:

   - Reference (Mapping::get, Mapping::operator[], Listing::operator[], etc.):
     The existence and the type of the target are guaranteed, and no check is
     needed on the caller side; an exception is thrown when the guarantee is
     broken. This form is intended for reading values in a concise form.

   - ValueNodeRef (the find functions of Mapping):
     The target may not exist. The returned object works as the boolean value
     corresponding to the validity in boolean contexts, and the node pointer
     stored in it is never nullptr; a special invalid node object is set when
     the target is not found so that the node access can also be written in a
     flow style. See the comment of the ValueNodeRef class for details.

   - Raw pointer (toScalar, toMapping, toListing, Listing::at, etc.):
     The returned pointer is never nullptr; an exception is thrown when the
     requested access is not possible. This form is intended for handling nodes
     as objects, such as storing them in ref_ptr and chaining the node access.
     A nullable raw pointer is not used in this API; whether a check is needed
     is expressed by ValueNodeRef.

   - Boolean return value with an output argument (the read functions):
     The target is optional; the value is stored in the output argument and
     true is returned only when the target exists and has the expected type.

   New functions should also follow these conventions.
*/

class CNOID_EXPORT ValueNode : public ClonableReferenced
{
    struct Initializer {
        Initializer();
    };
    static Initializer initializer;
        
public:
    /**
     * Clone methods providing convenient API with reference parameter.
     * These delegate to doClone() which contains the actual implementation.
     */
    ValueNode* clone() const {
        return doClone(nullptr);
    }
    ValueNode* clone(CloneMap& cloneMap) const {
        return doClone(&cloneMap);
    }
    
    // Using covariant return type - returns ValueNode* instead of Referenced*
    virtual ValueNode* doClone(CloneMap* cloneMap) const override;

    enum TypeBit {
        INVALID_NODE = 0,
        SCALAR = 1,
        MAPPING = 2,
        LISTING = 4,
        INSERT_LF = 8,
        APPEND_LF = 16,
        FORCED_RADIAN_MODE = 32
    };

    bool isValid() const { return typeBits; }
    explicit operator bool() const { return isValid(); }
    
    TypeBit LFType() const { return (TypeBit)(typeBits & (INSERT_LF | APPEND_LF)); }
    TypeBit nodeType() const { return (TypeBit)(typeBits & 7); }

    int toInt() const;
    double toDouble() const;
    float toFloat() const;
    bool toBool() const;
    [[deprecated("Check isForcedRadianMode() of the top node.")]]
    double toAngle() const;

    bool isScalar() const { return typeBits & SCALAR; }
    bool isString() const { return typeBits & SCALAR; }
    bool isCollection() const { return typeBits & (MAPPING | LISTING); }

    const std::string& toString() const;
    
    operator const std::string& () const {
        return toString();
    }

    /**
       \note If this is true for a top node given to a particular process, all the angle elements
       contained in the sub tree are foreced to radians. Note that the mode of the nodes except for
       the top node given to the process do not affect the angle unit.
    */
    bool isForcedRadianMode() const { return typeBits & FORCED_RADIAN_MODE; }
    void setForcedRadianMode(bool on = true) { typeBits |= FORCED_RADIAN_MODE; }

    /**
       When this is set for a node used as a value of a mapping entry, a blank line is
       inserted after the entry when the mapping is written, as long as another entry
       follows it.
    */
    bool isBlankLineAppended() const { return typeBits & APPEND_LF; }
    void setBlankLineAppended(bool on = true) {
        if(on){ typeBits |= APPEND_LF; } else { typeBits &= ~APPEND_LF; }
    }
    [[deprecated("Use 'isForcedRadianMode' to determine the angle unit.")]]
    bool isDegreeMode() const { return !isForcedRadianMode(); }
    [[deprecated("Use ''setForcedRadianMode' to specify the angle unit.")]]
    void setDegreeMode() { setForcedRadianMode(false); }

    template<typename T> T to() const;

    /**
       The following conversion functions return the pointer to this node as the
       specified node type. The returned pointer is never nullptr; an exception
       is thrown when this node is not of the specified type.
    */
    const ScalarNode* toScalar() const;
    ScalarNode* toScalar();
    
    bool isMapping() const { return typeBits & MAPPING; }
    const Mapping* toMapping() const;
    Mapping* toMapping();

    bool isListing() const { return typeBits & LISTING; }
    const Listing* toListing() const;
    Listing* toListing();
        
#ifdef CNOID_BACKWARD_COMPATIBILITY
    bool isSequence() const { return typeBits & LISTING; }
    const Listing* toSequence() const { return toListing(); }
    Listing* toSequence() { return toListing(); }
#endif

    bool read(int &out_value) const;
    bool read(double &out_value) const;
    bool read(float &out_value) const;
    bool read(bool &out_value) const;
    bool read(std::string &out_value) const;

    bool hasLineInfo() const { return (line_ >= 0); }
    int line() const { return line_ + 1; }
    int column() const { return column_ + 1; }

    void throwException(const std::string& message) const;

    class CNOID_EXPORT Exception {
    public:
        Exception();
        virtual ~Exception();
        int line() const { return line_; }
        int column() const { return column_; }
        std::string message() const;
        void setPosition(int line, int column) {
            line_ = line;
            column_ = column;
        }
        void setMessage(const std::string& m){
            message_ = m;
        }
    private:
        int line_;
        int column_;
        std::string message_;
    };
        
    class KeyNotFoundException : public Exception {
    public:
        const std::string& key() { return key_; }
        void setKey(const std::string& key) { key_ = key; }
    private:
        std::string key_;
    };

    class EmptyKeyException : public Exception {
    };
        
    class NotScalarException : public Exception {
    };
        
    class ScalarTypeMismatchException : public Exception {
    };

    class NotMappingException : public Exception {
    };

    class NotListingException : public Exception {
    };

    class SyntaxException : public Exception {
    };

    class DocumentNotFoundException : public Exception {
    };

    class FileException : public Exception {
    };

    class UnknownNodeTypeException : public Exception {
    };

    // This function is only used by YAMLWriter
    int indexInMapping() const { return indexInMapping_; }
    void setAsHeaderInMapping(int priority = 1) { indexInMapping_ = -priority; }

public:
    // Made public so that nanobind, which owns and frees Referenced-derived
    // objects through its intrusive ownership mechanism, can destroy a bare
    // ValueNode wrapper (e.g. a scalar element obtained via Listing/Mapping
    // indexing). Mapping and Listing already expose public destructors.
    virtual ~ValueNode() { }

protected:

    ValueNode() { }
    ValueNode(TypeBit type) : typeBits(type), line_(-1), column_(-1) { }

    void throwNotScalarException() const;
    void throwNotMappingException() const;
    void throwNotListingException() const;

    int typeBits;

private:
    ValueNode(const ValueNode& org);
    ValueNode& operator=(const ValueNode&);

    int line_;
    int column_;

    //! \todo Move this information to a value of the map defined in Mapping
    int indexInMapping_;

    friend class YAMLReaderImpl;
    friend class ScalarNode;
    friend class Mapping;
    friend class Listing;
};

template<> inline double ValueNode::to<double>() const { return toDouble(); }
template<> inline float ValueNode::to<float>() const { return toFloat(); }
template<> inline int ValueNode::to<int>() const { return toInt(); }
template<> inline std::string ValueNode::to<std::string>() const { return toString(); }
    
typedef ref_ptr<ValueNode> ValueNodePtr;

    
class CNOID_EXPORT ScalarNode : public ValueNode
{
public:
    ScalarNode(const std::string& value, StringStyle stringStyle = PLAIN_STRING);
    ScalarNode(int value);
    ScalarNode(double value, const char* floatingNumberFormat_ = nullptr);
    
    // Type-safe clone methods
    ScalarNode* clone() const {
        return doClone(nullptr);
    }
    ScalarNode* clone(CloneMap& cloneMap) const {
        return doClone(&cloneMap);
    }
    
    // Using covariant return type - returns ScalarNode* instead of ValueNode*
    virtual ScalarNode* doClone(CloneMap* cloneMap) const override;

    const std::string& stringValue() const { return stringValue_; }
    StringStyle stringStyle() const { return stringStyle_; }
    
private:
    ScalarNode(const char* text, size_t length);
    ScalarNode(const char* text, size_t length, StringStyle stringStyle);
    ScalarNode(const ScalarNode& org);

    std::string stringValue_;
    StringStyle stringStyle_;

    friend class YAMLReaderImpl;
    friend class ValueNode;
    friend class Mapping;
    friend class Listing;
};

typedef ref_ptr<ScalarNode> ScalarNodePtr;


inline const std::string& ValueNode::toString() const
{
    if(!isScalar()){
        throwNotScalarException();
    }
    return static_cast<const ScalarNode* const>(this)->stringValue_;
}


/**
   This class is used as the return type of the find functions of Mapping.
   The stored node pointer is never nullptr; when the node is not found, a pointer
   to a special invalid node object is stored so that the node access can be written
   in a flow style without inserting the validity check of each pointer. In that
   case, the check can be done by isValid() at any position of the flow. In addition
   to this original convention, this class works as the boolean value corresponding
   to isValid() in boolean contexts, so that the following idiom is also available:

   \code
   if(auto mapping = parent->findMapping(key)){ ... }
   \endcode

   Note that this class is a smart pointer derived from ref_ptr, and it is
   implicitly converted to the raw node pointer or ref_ptr. The conversion
   never results in nullptr.

   \note (Future plan) Deriving from ref_ptr adds a pair of reference count
   operations (about 7.5 ns) to every find call. This design is adopted for now
   because it keeps the full source compatibility with the existing code,
   including the copy initialization of a smart pointer variable
   (MappingPtr p = mapping->findMapping(key);), which requires two user-defined
   conversions and is only allowed by the derived-to-base conversion. Once such
   copy initializations are rewritten with auto, a raw pointer variable, or the
   direct initialization (MappingPtr p(mapping->findMapping(key));), the base
   class of this class can be replaced with a non-owning lightweight wrapper
   that just holds a raw node pointer. That will make the overhead of this class
   completely zero while keeping all the other usages valid, including the
   isValid() checks and the flow-style accesses. The remaining copy
   initializations will then be detected as compile errors, so the replacement
   does not cause any silent runtime breakage. On the other hand, replacing the
   return type of the find functions with a nullable raw pointer is NOT planned;
   it would give no additional performance gain over the non-owning wrapper, and
   the existing flow-style code would compile fine but crash at runtime.
*/
template<class NodeType = ValueNode>
class ValueNodeRef : public ref_ptr<NodeType>
{
public:
    ValueNodeRef(NodeType* node) : ref_ptr<NodeType>(node) { }

    //! This boolean conversion returns the validity of the found node instead of the null check
    explicit operator bool() const { return (*this)->isValid(); }
};

typedef ValueNodeRef<Mapping> MappingRef;
typedef ValueNodeRef<Listing> ListingRef;


class CNOID_EXPORT Mapping : public ValueNode
{
    typedef std::map<std::string, ValueNodePtr> Container;
        
public:

    typedef Container::iterator iterator;
    typedef Container::const_iterator const_iterator;

    Mapping();
    Mapping(int line, int column);
    virtual ~Mapping();

    // Type-safe clone methods
    Mapping* clone() const {
        return doClone(nullptr);
    }
    Mapping* clone(CloneMap& cloneMap) const {
        return doClone(&cloneMap);
    }
    
    // Using covariant return type - returns Mapping* instead of ValueNode*
    virtual Mapping* doClone(CloneMap* cloneMap) const override;
    
    // Deep clone without node sharing (all nodes are duplicated)
    Mapping* deepClone() const;
    
    [[deprecated("Use clone() instead")]]
    Mapping* cloneMapping() const {
        return clone();
    }
    
    bool empty() const { return values.empty(); }
    int size() const { return static_cast<int>(values.size()); }
    void clear();

    void setFlowStyle(bool isFlowStyle = true) { isFlowStyle_ = isFlowStyle; }
    bool isFlowStyle() const { return isFlowStyle_; }

    void setFloatingNumberFormat(const char* format);
    const char* floatingNumberFormat() { return floatingNumberFormat_; }

    [[deprecated("Use Mapping::setFloatingNumberFormat")]]
    void setDoubleFormat(const char* format) { setFloatingNumberFormat(format); }
    [[deprecated("Use Mapping::floatingNumberFormat")]]
    const char* doubleFormat() { return floatingNumberFormat(); }
        
    void setKeyQuoteStyle(StringStyle style);

    ValueNodeRef<> find(const std::string& key) const;
    ValueNodeRef<> find(std::initializer_list<const char*> keys) const;
    MappingRef findMapping(const std::string& key) const;
    MappingRef findMapping(std::initializer_list<const char*> keys) const;
    ListingRef findListing(const std::string& key) const;
    ListingRef findListing(std::initializer_list<const char*> keys) const;

    ValueNodePtr extract(const std::string& key);
    ValueNodePtr extract(std::initializer_list<const char*> keys);

    bool extract(const std::string& key, double& out_value);
    bool extract(const std::string& key, std::string& out_value);

    ValueNode& get(const std::string& key) const;
    ValueNode& get(std::initializer_list<const char*> keys) const;
    
    ValueNode& operator[](const std::string& key) const {
        return get(key);
    }

    void insert(const std::string& key, ValueNode* node);

    void insert(const Mapping* other, bool doArrangeElementIndices = true);

    Mapping* openMapping(const std::string& key) {
        return openMapping_(key, false);
    }
        
    Mapping* openFlowStyleMapping(const std::string& key) {
        return openFlowStyleMapping_(key, false);
    }

    Mapping* createMapping(const std::string& key) {
        return openMapping_(key, true);
    }
        
    Mapping* createFlowStyleMapping(const std::string& key) {
        return openFlowStyleMapping_(key, true);
    }

    Listing* openListing(const std::string& key) {
        return openListing_(key, false);
    }
        
    Listing* openFlowStyleListing(const std::string& key){
        return openFlowStyleListing_(key, false);
    }

    Listing* createListing(const std::string& key){
        return openListing_(key, true);
    }
        
    Listing* createFlowStyleListing(const std::string& key){
        return openFlowStyleListing_(key, true);
    }

    bool remove(const std::string& key);

    bool read(const std::string& key, std::string& out_value) const;
    bool read(const std::string& key, bool& out_value) const;
    bool read(const std::string& key, int& out_value) const;
    bool read(const std::string& key, double& out_value) const;
    bool read(const std::string& key, float& out_value) const;

    template<class T>
    bool read(std::initializer_list<const char*> keys, T& out_value) const {
        for(auto& key : keys){
            if(this->read(key, out_value)){
                return true;
            }
        }
        return false;
    }

    bool readAngle(const std::string& key, double& out_angle, const ValueNode* unitAttrNode = nullptr) const;
    bool readAngle(const std::string& key, float& out_angle, const ValueNode* unitAttrNode = nullptr) const;
    bool readAngle(std::initializer_list<const char*> keys, double& out_angle, const ValueNode* unitAttrNode = nullptr) const;
    bool readAngle(std::initializer_list<const char*> keys, float& out_angle, const ValueNode* unitAttrNode = nullptr) const;
    
    template <class T> T get(const std::string& key) const {
        T value;
        if(!read(key, value)){
            throwKeyNotFoundException(key);
        }            
        return value;
    }

    template <class T> T get(std::initializer_list<const char*> keys) const {
        T value;
        if(!read(keys, value)){
            throwKeyNotFoundException(*keys.begin());
        }
        return value;
    }
    
    template <class T>
    T get(const std::string& key, const T& defaultValue) const {
        T value;
        if(read(key, value)){
            return value;
        } else {
            return defaultValue;
        }
    }

    std::string get(const std::string& key, const char* defaultValue) const {
        std::string value;
        if(read(key, value)){
            return value;
        } else {
            return defaultValue;
        }
    }

    template<class T>
        T get(std::initializer_list<const char*> keys, const T& defaultValue) const {
        T value;
        if(read(keys, value)){
            return value;
        } else {
            return defaultValue;
        }
    }

    void write(const std::string& key, const std::string& value, StringStyle stringStyle = PLAIN_STRING);
    void write(const std::string& key, const char* value, StringStyle stringStyle = PLAIN_STRING){
        write(key, std::string(value), stringStyle);
    }

    void write(const std::string& key, bool value);
    void write(const std::string& key, int value);
    void write(const std::string& key, double value);
    void writePath(const std::string &key, const std::string& value);
    void write(const std::string& key, ValueNode* node) { insert(key, node); }

    template<class ArrayType> void writeAsListing(const std::string& key, const ArrayType& container);

    typedef enum { READ_MODE, WRITE_MODE } AssignMode;

    void setAssignMode(AssignMode mode) { this->mode = mode; }

    template <class T>
        void assign(const std::string& key, T& io_value, const T& defaultValue){
        switch(mode){
        case READ_MODE:
            if(!read(key, io_value)){
                io_value = defaultValue;
            }
            break;
        case WRITE_MODE:
            write(key, io_value);
            break;
        }
    }

    iterator begin() { return values.begin(); }
    iterator end() { return values.end(); }
    const_iterator begin() const { return values.begin(); }
    const_iterator end() const { return values.end(); }

    void throwKeyNotFoundException(const std::string& key) const;

    StringStyle keyStringStyle() const { return keyStringStyle_; }

    size_t getContentHash() const;

#ifdef CNOID_BACKWARD_COMPATIBILITY
    Listing* findSequence(const std::string& key) const { return findListing(key); }
    Listing* openSequence(const std::string& key) { return openListing(key); }
    Listing* openFlowStyleSequence(const std::string& key){ return openFlowStyleListing(key); }
    Listing* createSequence(const std::string& key){ return createListing(key); }
    Listing* createFlowStyleSequence(const std::string& key){ return createFlowStyleListing(key); }
#endif
        
private:

    Mapping(const Mapping& org);
    Mapping(const Mapping& org, CloneMap* cloneMap);
    Mapping& operator=(const Mapping&);

    Mapping* openMapping_(const std::string& key, bool doOverwrite);
    Mapping* openFlowStyleMapping_(const std::string& key, bool doOverwrite);
    Listing* openListing_(const std::string& key, bool doOverwrite);
    Listing* openFlowStyleListing_(const std::string& key, bool doOverwrite);

    inline void insertSub(const std::string& key, ValueNode* node);

    void writeSub(const std::string &key, const char* text, size_t length, StringStyle stringStyle);

    Container values;
    AssignMode mode;
    mutable int indexCounter;
    const char* floatingNumberFormat_;
    bool isFlowStyle_;
    StringStyle keyStringStyle_;

    friend class Listing;
    friend class YAMLReaderImpl;
};

typedef ref_ptr<Mapping> MappingPtr;


/**
   @todo add 'openMapping' and 'openListing' methods
   @note The name "Sequence" should not be used for this class
   because it confilcts with the name defined in the boost's concept check library.
*/
class CNOID_EXPORT Listing : public ValueNode
{
    typedef std::vector<ValueNodePtr> Container;

public:

    Listing();
    Listing(int size);
    ~Listing();
        
    // Type-safe clone methods
    Listing* clone() const {
        return doClone(nullptr);
    }
    Listing* clone(CloneMap& cloneMap) const {
        return doClone(&cloneMap);
    }
    
    // Using covariant return type - returns Listing* instead of ValueNode*
    virtual Listing* doClone(CloneMap* cloneMap) const override;
    
    // Deep clone without node sharing (all nodes are duplicated)
    Listing* deepClone() const;

    typedef Container::iterator iterator;
    typedef Container::const_iterator const_iterator;

    bool empty() const { return values.empty(); }
    int size() const { return static_cast<int>(values.size()); }
    void clear();
    void reserve(int size);

    void setFlowStyle(bool isFlowStyle = true) { isFlowStyle_ = isFlowStyle; }
    bool isFlowStyle() const { return isFlowStyle_; }

    void setFloatingNumberFormat(const char* format);
    const char* floatingNumberFormat() { return floatingNumberFormat_; }

    [[deprecated("Use Mapping::setFloatingNumberFormat")]]
    void setDoubleFormat(const char* format) { setFloatingNumberFormat(format); }
    [[deprecated("Use Mapping::floatingNumberFormat")]]
    const char* doubleFormat() { return floatingNumberFormat(); }

    ValueNode* front() const {
        return values.front();
    }

    ValueNode* back() const {
        return values.back();
    }

    /**
       The element access for handling the element as a node object, such as
       storing it in ref_ptr and chaining the node access. The returned pointer
       is never nullptr, and the index must be within the valid range.
    */
    ValueNode* at(int i) const {
        return values[i];
    }

    /**
       deprecated
    */
    ValueNode& get(int i) const {
        return *values[i];
    }

    void write(int i, int value);
    void write(int i, const std::string& value, StringStyle stringStyle = PLAIN_STRING);

    /**
       The element access for reading the element value in a concise form.
       Use the at function instead to handle the element as a node object.
       The index must be within the valid range.
    */
    ValueNode& operator[](int i) const {
        return *values[i];
    }

    /// \todo implement the following funcion (ticket #35)
    //MappingPtr extractMapping(const std::string& key) const;

    Mapping* newMapping();

    void append(ValueNode* node) {
        values.push_back(node);
    }

    void append(ValueNode* node, int maxColumns, int numValues = 0) {
        insertLF(maxColumns, numValues);
        values.push_back(node);
    }

    void insert(int index, ValueNode* node);
        
    void append(int value);

    /**
       @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
       @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
       This feature is disabled if numValues = 0.
    */
    void append(int value, int maxColumns, int numValues = 0) {
        insertLF(maxColumns, numValues);
        append(value);
    }

    void append(size_t value);
    void append(double value);

    /**
       @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
       @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
       This feature is disabled if numValues = 0.
    */
    void append(double value, int maxColumns, int numValues = 0) {
        insertLF(maxColumns, numValues);
        append(value);
    }

    void append(const std::string& value, StringStyle stringStyle = PLAIN_STRING);

    /**
       @param maxColumns LF is automatically inserted when the column pos is over maxColumsn
       @param numValues If numValues is not greater than maxColumns, the initial LF is skipped.
       This feature is disabled if numValues = 0.
    */
    void append(const std::string& value, int maxColumns, int numValues = 0, StringStyle stringStyle = PLAIN_STRING){
        insertLF(maxColumns, numValues);
        append(value, stringStyle);
    }

    void appendLF();

    iterator begin() { return values.begin(); }
    iterator end() { return values.end(); }
    const_iterator begin() const { return values.begin(); }
    const_iterator end() const { return values.end(); };

    size_t getContentHash() const;

private:

    Listing(int line, int column);
    Listing(int line, int column, int reservedSize);
        
    Listing(const Listing& org);
    Listing(const Listing& org, CloneMap* cloneMap);
    Listing& operator=(const Listing&);

    void insertLF(int maxColumns, int numValues);
        
    Container values;
    const char* floatingNumberFormat_;
    bool isFlowStyle_;
    bool doInsertLFBeforeNextElement;

    friend class Mapping;
    friend class YAMLReaderImpl;
};


template<class ArrayType>
void Mapping::writeAsListing(const std::string& key, const ArrayType& container)
{
    auto listing = createFlowStyleListing(key);
    for(auto& value : container){
        listing->append(value);
    }
}


typedef ref_ptr<Listing> ListingPtr;

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef ValueNode YamlNode;
typedef ValueNodePtr YamlNodePtr;
typedef ScalarNode YamlScalar;
typedef Mapping YamlMapping;
typedef MappingPtr YamlMappingPtr;
typedef Listing YamlSequence;
typedef ListingPtr YamlSequencePtr;
typedef Listing Sequence;
typedef ListingPtr SequencePtr;
#endif
}

#endif
