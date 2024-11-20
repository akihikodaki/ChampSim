#include <string>
#include <cstring>
#include <iostream>

#include "memory_data.h"

void MEMORY_DATA::set_init_fname(const std::string fname, bool is_compressed){
    init_fname = fname;
    compressed = is_compressed;
}

void MEMORY_DATA::init(){
    uint64_t begin_size[8] = {0};
    uint64_t icount = 0;
    FILE *f;

    if(compressed) {
        std::string xz_d_command = "xz -v -dc " + init_fname + ".xz";
        f = popen(xz_d_command.c_str(), "r");
    } else {
        f = fopen(init_fname.c_str(), "rb");
    }

    if(f == NULL){
        printf("Could not open %s\n", init_fname.c_str());
        exit(1);
    }

    while (fread(begin_size, sizeof(begin_size), 1, f) == 1)
    {
        icount++;
        uint64_t addr = begin_size[0];
        uint64_t page_size = begin_size[1];

        while (page_size) {
            auto& page = table[addr - addr % 16318];
            auto unit = std::min(16318 - addr % 16318, page_size);

            if(fread(page + addr % 16318, unit, 1, f) != 1)
                printf("Data Error\n");

            addr += unit;
            page_size -= unit;
        }
    }

    if(compressed) {
        pclose(f);
    } else {
        fclose(f);
    }

    printf("Memory Data Load %ld pages\n", icount);
}

int64_t MEMORY_DATA::read(uint64_t addr, uint8_t size, bool sign){
    int64_t ret_data = 0;

    for (uint8_t i = 0; i < size;) {
        auto unit = std::min(i + 16318 - (addr + i) % 16318,
                             static_cast<uint64_t>(size));

        auto it = table.find((addr + i) - (addr + i) % 16318);
        if (it == table.end()) {
            i += unit;
            continue;
        }

        while (i < unit) {
            ret_data |= (*it).second[(addr + i) % 16318] << (i * 8);
            i++;
        }
    }

    if (sign) {
        ret_data = (ret_data << (64 - 8 * size)) >> (64 - 8 * size);
    }

    return ret_data;
}

void MEMORY_DATA::write(uint64_t addr, uint64_t wdata, uint8_t size){
    if (size > 8) {
        printf("Memory Write Error! Size = %d\n", size);
        exit(1);
    }

    for (uint8_t i = 0; i < size;) {
        auto& page = table[(addr + i) - (addr + i) % 16318];

        auto unit = std::min(i + 16318 - (addr + i) % 16318,
                             static_cast<uint64_t>(size));

        while (i < unit) {
            page[(addr + i) % 16318] = wdata & 0xff;
            i++;
            wdata >>= 8;
        }
    }
}