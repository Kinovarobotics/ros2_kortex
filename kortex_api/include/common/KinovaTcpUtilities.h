#ifndef CPPAPI_KINOVATCPUTILITIES_H
#define CPPAPI_KINOVATCPUTILITIES_H

#include <string>

class KinovaTcpUtilities {

public:
    KinovaTcpUtilities();
    ~KinovaTcpUtilities();

    uint32_t ParseBufferHeader(uint8_t* buff);
    void PrependHeader(uint8_t *in_buf, uint32_t payload_length);
    void PrintBuffer(uint8_t* in_buf, uint32_t len);
    char* ResizeBuffer(char *buf, size_t data_size_to_copy, size_t new_size, uint32_t& buffer_size);
    uint8_t* ResizeBuffer(uint8_t *buf, size_t data_size_to_copy, size_t new_size, uint32_t& buffer_size);

    const std::string KINOVA_MAGIC_STRING = { "\07xEtRoK\07" };
    const size_t KINOVA_HEADER_SIZE { (KINOVA_MAGIC_STRING.length() + sizeof(uint32_t) )};
};


#endif //CPPAPI_KINOVATCPUTILITIES_H
