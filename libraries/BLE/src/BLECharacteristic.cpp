
//#include "internal/ble_client.h"

#include "BLECharacteristic.h"
#include "BLEProfileManager.h"
#include "BLEUtils.h"

#include "BLECharacteristicImp.h"

BLECharacteristic::BLECharacteristic():
    _bledev(), _internal(NULL), _properties(0), 
    _value_size(0), _value(NULL),_event_handlers(NULL)
{
    memset(_uuid_cstr, 0, sizeof(_uuid_cstr));
}

BLECharacteristic::BLECharacteristic(const char* uuid, 
                                     unsigned char properties, 
                                     unsigned short valueSize):
    _bledev(), _internal(NULL), _properties(properties), _value(NULL),
    _event_handlers(NULL)
{
    bt_uuid_128 bt_uuid_tmp;
    _value_size = valueSize > BLE_MAX_ATTR_LONGDATA_LEN ? BLE_MAX_ATTR_LONGDATA_LEN : valueSize;
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t *)&bt_uuid_tmp);
    BLEUtils::uuidBT2String((const bt_uuid_t *)&bt_uuid_tmp, _uuid_cstr);
    _bledev.setAddress(*BLEUtils::bleGetLoalAddress());
}

BLECharacteristic::BLECharacteristic(const char* uuid, 
                                     unsigned char properties, 
                                     const char* value):
    BLECharacteristic(uuid, properties, strlen(value))
{
    _setValue((const uint8_t*)value, strlen(value));
}

BLECharacteristic::BLECharacteristic(BLECharacteristicImp *characteristicImp,
                                     const BLEDevice *bleDev):
    _bledev(bleDev), _internal(characteristicImp), 
    _value(NULL),_event_handlers(NULL)
{
    BLEUtils::uuidBT2String(characteristicImp->bt_uuid(), _uuid_cstr);
    _properties = characteristicImp->properties();
    _value_size = characteristicImp->valueSize();
}

BLECharacteristic::~BLECharacteristic()
{
    if (_value) 
    {
        bfree(_value);
        _value = NULL;
    }
    
    if (_event_handlers != NULL)
    {
        bfree(_event_handlers);
        _event_handlers = NULL;
    }
}

const char* BLECharacteristic::uuid() const
{
    return _uuid_cstr;
}

unsigned char BLECharacteristic::properties()
{
    unsigned char property = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (NULL != characteristicImp)
    {
        property = characteristicImp->properties();
    }
    return property;
}

int BLECharacteristic::valueSize() //const
{
    int valuesize = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (NULL != characteristicImp)
    {
        valuesize = characteristicImp->valueSize();
    }
    return valuesize;
}

const byte* BLECharacteristic::value() //const
{
    const byte* value_temp = NULL;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (NULL != characteristicImp)
    {
        value_temp = characteristicImp->value();
    }
    return value_temp;
}

int BLECharacteristic::valueLength() //const
{
    int valueLength = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (NULL != characteristicImp)
    {
        valueLength = characteristicImp->valueLength();
    }
    return valueLength;
}

BLECharacteristic::operator bool() const
{
    return (strlen(_uuid_cstr) > 3);
}


byte BLECharacteristic::operator[] (int offset) //const
{
    byte data = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        data = (*characteristicImp)[offset];
    }
    return data;
}

bool BLECharacteristic::writeValue(const byte value[], int length)
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        characteristicImp->writeValue(value, length);
        retVar = true;
    }
    return retVar;
}

bool BLECharacteristic::writeValue(const byte value[], int length, int offset)
{
    // TODO: Not support it now.
    // Will add this feature.
    return false;
}

bool BLECharacteristic::writeValue(const char* value)
{
    return writeValue((const byte*)value, strlen(value));
}

bool BLECharacteristic::broadcast()
{
    // TODO: Need more information
    return false;
}

bool BLECharacteristic::written()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->written();
    }
    return retVar;
}

bool BLECharacteristic::subscribed()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->subscribed();
    }
    return retVar;
}

bool BLECharacteristic::canNotify()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->canNotify();
    }
    return retVar;
}

bool BLECharacteristic::canIndicate()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->canIndicate();
    }
    return retVar;
}

bool BLECharacteristic::canRead()
{
    // TODO: Need more confirmation
    return false;
}
bool BLECharacteristic::canWrite()
{
    // TODO: Need more confirmation
    return false;
}
bool BLECharacteristic::canSubscribe()
{
    // TODO: Need more confirmation
    return false;
}
bool BLECharacteristic::canUnsubscribe()
{
    // TODO: Need more confirmation
    return false;
}

bool BLECharacteristic::read()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->read();
    }
    return retVar;
}

bool BLECharacteristic::write(const unsigned char* value, int length)
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->write(value, (uint16_t)length);
    }
    return retVar;
}

bool BLECharacteristic::subscribe()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->subscribe();
    }
    return retVar;
}

bool BLECharacteristic::unsubscribe()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->unsubscribe();
    }
    return retVar;
}

bool BLECharacteristic::valueUpdated()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->valueUpdated();
    }
    return retVar;
}

int BLECharacteristic::addDescriptor(BLEDescriptor& descriptor)
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->addDescriptor(descriptor);
    }
    return retVar;
}

int BLECharacteristic::descriptorCount() //const
{
    int count = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        count = characteristicImp->descriptorCount();
    }
    return count;
}

bool BLECharacteristic::hasDescriptor(const char* uuid) const
{
    // TODO: Not support now
    return false;
}
bool BLECharacteristic::hasDescriptor(const char* uuid, int index) const
{
    // TODO: Not support now
    return false;
}
BLEDescriptor BLECharacteristic::descriptor(int index) const
{
    // TODO: Not support now
    return BLEDescriptor();
}
BLEDescriptor BLECharacteristic::descriptor(const char * uuid) const
{
    // TODO: Not support now
    return BLEDescriptor();
}
BLEDescriptor BLECharacteristic::descriptor(const char * uuid, int index) const
{
    // TODO: Not support now
    return BLEDescriptor();
}

void BLECharacteristic::setEventHandler(BLECharacteristicEvent event, 
                                        BLECharacteristicEventHandler eventHandler)
{
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (event >= BLECharacteristicEventLast)
    {
        return;
    }
    
    if (NULL != characteristicImp)
    {
        characteristicImp->setEventHandler(event, eventHandler);
    }
    else
    {
        if (_event_handlers == NULL)
        {
            _event_handlers = (BLECharacteristicEventHandler*)balloc(sizeof(BLECharacteristicEventHandler) * BLECharacteristicEventLast, NULL);
        }
        
        if (_event_handlers != NULL)
        {
            _event_handlers[event] = eventHandler;
        }
    }
}


void
BLECharacteristic::_setValue(const uint8_t value[], uint16_t length)
{
    if (length > _value_size) {
        length = _value_size;
    }
    
    if (NULL == _value)
    {
        // Allocate the buffer for characteristic
        _value = (unsigned char*)balloc(_value_size, NULL);//malloc(_value_size)
    }
    if (NULL == _value)
    {
        return;
    }
    memcpy(_value, value, length);
}

BLECharacteristicImp* BLECharacteristic::getImplementation()
{
    if (NULL == _internal)
    {
        _internal = BLEProfileManager::instance()->characteristic(_bledev, (const char*)_uuid_cstr);
    }
    return _internal;
}

void BLECharacteristic::setBLECharacteristicImp(BLECharacteristicImp *characteristicImp)
{
    _internal = characteristicImp;
}


