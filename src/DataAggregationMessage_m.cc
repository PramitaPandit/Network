//
// Generated file, do not edit! Created by opp_msgtool 6.0 from DataAggregationMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include <memory>
#include <type_traits>
#include "DataAggregationMessage_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp

Register_Class(DataAggregationMessage)

DataAggregationMessage::DataAggregationMessage(const char *name, short kind) : ::omnetpp::cMessage(name, kind)
{
}

DataAggregationMessage::DataAggregationMessage(const DataAggregationMessage& other) : ::omnetpp::cMessage(other)
{
    copy(other);
}

DataAggregationMessage::~DataAggregationMessage()
{
}

DataAggregationMessage& DataAggregationMessage::operator=(const DataAggregationMessage& other)
{
    if (this == &other) return *this;
    ::omnetpp::cMessage::operator=(other);
    copy(other);
    return *this;
}

void DataAggregationMessage::copy(const DataAggregationMessage& other)
{
    this->sourceNodeId = other.sourceNodeId;
    this->aggregatedValueNode11 = other.aggregatedValueNode11;
    this->aggregatedValueNode12 = other.aggregatedValueNode12;
    this->aggregatedValueNode21 = other.aggregatedValueNode21;
    this->aggregatedValueNode22 = other.aggregatedValueNode22;
    this->aggregatedValueNode31 = other.aggregatedValueNode31;
    this->aggregatedValueNode32 = other.aggregatedValueNode32;
}

void DataAggregationMessage::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cMessage::parsimPack(b);
    doParsimPacking(b,this->sourceNodeId);
    doParsimPacking(b,this->aggregatedValueNode11);
    doParsimPacking(b,this->aggregatedValueNode12);
    doParsimPacking(b,this->aggregatedValueNode21);
    doParsimPacking(b,this->aggregatedValueNode22);
    doParsimPacking(b,this->aggregatedValueNode31);
    doParsimPacking(b,this->aggregatedValueNode32);
}

void DataAggregationMessage::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cMessage::parsimUnpack(b);
    doParsimUnpacking(b,this->sourceNodeId);
    doParsimUnpacking(b,this->aggregatedValueNode11);
    doParsimUnpacking(b,this->aggregatedValueNode12);
    doParsimUnpacking(b,this->aggregatedValueNode21);
    doParsimUnpacking(b,this->aggregatedValueNode22);
    doParsimUnpacking(b,this->aggregatedValueNode31);
    doParsimUnpacking(b,this->aggregatedValueNode32);
}

int DataAggregationMessage::getSourceNodeId() const
{
    return this->sourceNodeId;
}

void DataAggregationMessage::setSourceNodeId(int sourceNodeId)
{
    this->sourceNodeId = sourceNodeId;
}

double DataAggregationMessage::getAggregatedValueNode11() const
{
    return this->aggregatedValueNode11;
}

void DataAggregationMessage::setAggregatedValueNode11(double aggregatedValueNode11)
{
    this->aggregatedValueNode11 = aggregatedValueNode11;
}

double DataAggregationMessage::getAggregatedValueNode12() const
{
    return this->aggregatedValueNode12;
}

void DataAggregationMessage::setAggregatedValueNode12(double aggregatedValueNode12)
{
    this->aggregatedValueNode12 = aggregatedValueNode12;
}

double DataAggregationMessage::getAggregatedValueNode21() const
{
    return this->aggregatedValueNode21;
}

void DataAggregationMessage::setAggregatedValueNode21(double aggregatedValueNode21)
{
    this->aggregatedValueNode21 = aggregatedValueNode21;
}

double DataAggregationMessage::getAggregatedValueNode22() const
{
    return this->aggregatedValueNode22;
}

void DataAggregationMessage::setAggregatedValueNode22(double aggregatedValueNode22)
{
    this->aggregatedValueNode22 = aggregatedValueNode22;
}

double DataAggregationMessage::getAggregatedValueNode31() const
{
    return this->aggregatedValueNode31;
}

void DataAggregationMessage::setAggregatedValueNode31(double aggregatedValueNode31)
{
    this->aggregatedValueNode31 = aggregatedValueNode31;
}

double DataAggregationMessage::getAggregatedValueNode32() const
{
    return this->aggregatedValueNode32;
}

void DataAggregationMessage::setAggregatedValueNode32(double aggregatedValueNode32)
{
    this->aggregatedValueNode32 = aggregatedValueNode32;
}

class DataAggregationMessageDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertyNames;
    enum FieldConstants {
        FIELD_sourceNodeId,
        FIELD_aggregatedValueNode11,
        FIELD_aggregatedValueNode12,
        FIELD_aggregatedValueNode21,
        FIELD_aggregatedValueNode22,
        FIELD_aggregatedValueNode31,
        FIELD_aggregatedValueNode32,
    };
  public:
    DataAggregationMessageDescriptor();
    virtual ~DataAggregationMessageDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyName) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyName) const override;
    virtual int getFieldArraySize(omnetpp::any_ptr object, int field) const override;
    virtual void setFieldArraySize(omnetpp::any_ptr object, int field, int size) const override;

    virtual const char *getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const override;
    virtual std::string getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const override;
    virtual omnetpp::cValue getFieldValue(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual omnetpp::any_ptr getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const override;
};

Register_ClassDescriptor(DataAggregationMessageDescriptor)

DataAggregationMessageDescriptor::DataAggregationMessageDescriptor() : omnetpp::cClassDescriptor(omnetpp::opp_typename(typeid(DataAggregationMessage)), "omnetpp::cMessage")
{
    propertyNames = nullptr;
}

DataAggregationMessageDescriptor::~DataAggregationMessageDescriptor()
{
    delete[] propertyNames;
}

bool DataAggregationMessageDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<DataAggregationMessage *>(obj)!=nullptr;
}

const char **DataAggregationMessageDescriptor::getPropertyNames() const
{
    if (!propertyNames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
        const char **baseNames = base ? base->getPropertyNames() : nullptr;
        propertyNames = mergeLists(baseNames, names);
    }
    return propertyNames;
}

const char *DataAggregationMessageDescriptor::getProperty(const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? base->getProperty(propertyName) : nullptr;
}

int DataAggregationMessageDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? 7+base->getFieldCount() : 7;
}

unsigned int DataAggregationMessageDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeFlags(field);
        field -= base->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,    // FIELD_sourceNodeId
        FD_ISEDITABLE,    // FIELD_aggregatedValueNode11
        FD_ISEDITABLE,    // FIELD_aggregatedValueNode12
        FD_ISEDITABLE,    // FIELD_aggregatedValueNode21
        FD_ISEDITABLE,    // FIELD_aggregatedValueNode22
        FD_ISEDITABLE,    // FIELD_aggregatedValueNode31
        FD_ISEDITABLE,    // FIELD_aggregatedValueNode32
    };
    return (field >= 0 && field < 7) ? fieldTypeFlags[field] : 0;
}

const char *DataAggregationMessageDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldName(field);
        field -= base->getFieldCount();
    }
    static const char *fieldNames[] = {
        "sourceNodeId",
        "aggregatedValueNode11",
        "aggregatedValueNode12",
        "aggregatedValueNode21",
        "aggregatedValueNode22",
        "aggregatedValueNode31",
        "aggregatedValueNode32",
    };
    return (field >= 0 && field < 7) ? fieldNames[field] : nullptr;
}

int DataAggregationMessageDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    int baseIndex = base ? base->getFieldCount() : 0;
    if (strcmp(fieldName, "sourceNodeId") == 0) return baseIndex + 0;
    if (strcmp(fieldName, "aggregatedValueNode11") == 0) return baseIndex + 1;
    if (strcmp(fieldName, "aggregatedValueNode12") == 0) return baseIndex + 2;
    if (strcmp(fieldName, "aggregatedValueNode21") == 0) return baseIndex + 3;
    if (strcmp(fieldName, "aggregatedValueNode22") == 0) return baseIndex + 4;
    if (strcmp(fieldName, "aggregatedValueNode31") == 0) return baseIndex + 5;
    if (strcmp(fieldName, "aggregatedValueNode32") == 0) return baseIndex + 6;
    return base ? base->findField(fieldName) : -1;
}

const char *DataAggregationMessageDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeString(field);
        field -= base->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "int",    // FIELD_sourceNodeId
        "double",    // FIELD_aggregatedValueNode11
        "double",    // FIELD_aggregatedValueNode12
        "double",    // FIELD_aggregatedValueNode21
        "double",    // FIELD_aggregatedValueNode22
        "double",    // FIELD_aggregatedValueNode31
        "double",    // FIELD_aggregatedValueNode32
    };
    return (field >= 0 && field < 7) ? fieldTypeStrings[field] : nullptr;
}

const char **DataAggregationMessageDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldPropertyNames(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *DataAggregationMessageDescriptor::getFieldProperty(int field, const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldProperty(field, propertyName);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int DataAggregationMessageDescriptor::getFieldArraySize(omnetpp::any_ptr object, int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldArraySize(object, field);
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        default: return 0;
    }
}

void DataAggregationMessageDescriptor::setFieldArraySize(omnetpp::any_ptr object, int field, int size) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldArraySize(object, field, size);
            return;
        }
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set array size of field %d of class 'DataAggregationMessage'", field);
    }
}

const char *DataAggregationMessageDescriptor::getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldDynamicTypeString(object,field,i);
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string DataAggregationMessageDescriptor::getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValueAsString(object,field,i);
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        case FIELD_sourceNodeId: return long2string(pp->getSourceNodeId());
        case FIELD_aggregatedValueNode11: return double2string(pp->getAggregatedValueNode11());
        case FIELD_aggregatedValueNode12: return double2string(pp->getAggregatedValueNode12());
        case FIELD_aggregatedValueNode21: return double2string(pp->getAggregatedValueNode21());
        case FIELD_aggregatedValueNode22: return double2string(pp->getAggregatedValueNode22());
        case FIELD_aggregatedValueNode31: return double2string(pp->getAggregatedValueNode31());
        case FIELD_aggregatedValueNode32: return double2string(pp->getAggregatedValueNode32());
        default: return "";
    }
}

void DataAggregationMessageDescriptor::setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValueAsString(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        case FIELD_sourceNodeId: pp->setSourceNodeId(string2long(value)); break;
        case FIELD_aggregatedValueNode11: pp->setAggregatedValueNode11(string2double(value)); break;
        case FIELD_aggregatedValueNode12: pp->setAggregatedValueNode12(string2double(value)); break;
        case FIELD_aggregatedValueNode21: pp->setAggregatedValueNode21(string2double(value)); break;
        case FIELD_aggregatedValueNode22: pp->setAggregatedValueNode22(string2double(value)); break;
        case FIELD_aggregatedValueNode31: pp->setAggregatedValueNode31(string2double(value)); break;
        case FIELD_aggregatedValueNode32: pp->setAggregatedValueNode32(string2double(value)); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'DataAggregationMessage'", field);
    }
}

omnetpp::cValue DataAggregationMessageDescriptor::getFieldValue(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValue(object,field,i);
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        case FIELD_sourceNodeId: return pp->getSourceNodeId();
        case FIELD_aggregatedValueNode11: return pp->getAggregatedValueNode11();
        case FIELD_aggregatedValueNode12: return pp->getAggregatedValueNode12();
        case FIELD_aggregatedValueNode21: return pp->getAggregatedValueNode21();
        case FIELD_aggregatedValueNode22: return pp->getAggregatedValueNode22();
        case FIELD_aggregatedValueNode31: return pp->getAggregatedValueNode31();
        case FIELD_aggregatedValueNode32: return pp->getAggregatedValueNode32();
        default: throw omnetpp::cRuntimeError("Cannot return field %d of class 'DataAggregationMessage' as cValue -- field index out of range?", field);
    }
}

void DataAggregationMessageDescriptor::setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValue(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        case FIELD_sourceNodeId: pp->setSourceNodeId(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_aggregatedValueNode11: pp->setAggregatedValueNode11(value.doubleValue()); break;
        case FIELD_aggregatedValueNode12: pp->setAggregatedValueNode12(value.doubleValue()); break;
        case FIELD_aggregatedValueNode21: pp->setAggregatedValueNode21(value.doubleValue()); break;
        case FIELD_aggregatedValueNode22: pp->setAggregatedValueNode22(value.doubleValue()); break;
        case FIELD_aggregatedValueNode31: pp->setAggregatedValueNode31(value.doubleValue()); break;
        case FIELD_aggregatedValueNode32: pp->setAggregatedValueNode32(value.doubleValue()); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'DataAggregationMessage'", field);
    }
}

const char *DataAggregationMessageDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructName(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

omnetpp::any_ptr DataAggregationMessageDescriptor::getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructValuePointer(object, field, i);
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        default: return omnetpp::any_ptr(nullptr);
    }
}

void DataAggregationMessageDescriptor::setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldStructValuePointer(object, field, i, ptr);
            return;
        }
        field -= base->getFieldCount();
    }
    DataAggregationMessage *pp = omnetpp::fromAnyPtr<DataAggregationMessage>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'DataAggregationMessage'", field);
    }
}

namespace omnetpp {

}  // namespace omnetpp

