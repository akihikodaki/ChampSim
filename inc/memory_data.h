#ifndef MEMORY_DATA_H
#define MEMORY_DATA_H

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <unordered_map>

class MEMORY_DATA {
protected:
    std::unordered_map<uint64_t, char[16318]> table;
    std::string init_fname;
    bool compressed = false;

public:
    void set_init_fname(const std::string fname, bool is_compressed);
    void init();

    int64_t read(uint64_t addr, uint8_t size, bool sign);
    void write(uint64_t addr, uint64_t wdata, uint8_t zize);
};

#endif
