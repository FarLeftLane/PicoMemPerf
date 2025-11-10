/*
MIT License

Copyright (c) 2025 Michael Neil, Far Left Lane

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "hardware/clocks.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <malloc.h>


//  PSRAM setup routines from Waveshare Core2350B demo code

/*
    //2350 memory map details

    0x00000000 - 0x1FFFFFFF Code
    0x20000000 - 0x3FFFFFFF SRAM
    0x40000000 - 0x5FFFFFFF Peripheral
    0x60000000 - 0x9FFFFFFF External RAM
    0xA0000000 - 0xDFFFFFFF External device
    
    #define PSRAM_LOCATION _u(0x11000000)

    XIP
    0x11000000
    0x10000000 0x13ffffff XIP, Cached
    0x14000000 0x17ffffff XIP, Uncached
    0x18000000 0x1bffffff XIP, Cache Maintenance
    0x1c000000 0x1fffffff XIP, Uncached + Untranslated


    The 26-bit XIP address space is mirrored multiple times in the RP2350 address space, decoded
    on bits 27:26 of the system bus address:
    • 0x10… : Cached XIP access
    • 0x14… : Uncached XIP access
    • 0x18… : Cache maintenance writes
    • 0x1c… : Uncached, untranslated XIP access — bypass QMI address translation

*/

#define RP2350_XIP_CSI_PIN  47
#define PSRAM_CMD_QUAD_END 0xF5
#define PSRAM_CMD_QUAD_ENABLE 0x35
#define PSRAM_CMD_READ_ID 0x9F
#define PSRAM_CMD_RSTEN 0x66
#define PSRAM_CMD_RST 0x99
#define PSRAM_CMD_QUAD_READ 0xEB
#define PSRAM_CMD_QUAD_WRITE 0x38
#define PSRAM_CMD_NOOP 0xFF
#define PSRAM_CMD_LINEAR_TOGGLE 0xC0

#define PSRAM_ID 0x5D

// max select pulse width = 8us
#define PSRAM_MAX_SELECT 0.000008f

// min deselect pulse width = 50ns
#define PSRAM_MIN_DESELECT 0.000000050f

// from psram datasheet - max Freq at 3.3v
#define PSRAM_MAX_SCK_HZ 109000000.f

// Location /address where PSRAM starts
#define PSRAM_LOCATION          _u(0x11000000)      //  0x11000000
#define PSRAM_LOCATION_NOCAHE   _u(0x14000000)      //  0x14000000
                                   
static size_t _psram_size = 0;


static size_t __no_inline_not_in_flash_func(setup_psram)(uint psram_cs_pin)
{
    gpio_set_function(psram_cs_pin, GPIO_FUNC_XIP_CS1);

    size_t psram_size = 0;

    const int max_psram_freq = 133000000;
    const int clock_hz = clock_get_hz(clk_sys);
    int clockDivider = (clock_hz + max_psram_freq - 1) / max_psram_freq;
    if (clockDivider == 1 && clock_hz > 100000000) {
        clockDivider = 2;
    }
    int rxdelay = clockDivider;
    if (clock_hz / clockDivider > 100000000) {
        rxdelay += 1;
    }

    // - Max select must be <= 8us.  The value is given in multiples of 64 system clocks.
    // - Min deselect must be >= 18ns.  The value is given in system clock cycles - ceil(divisor / 2).
    const int clock_period_fs = 1000000000000000ll / clock_hz;
    const int maxSelect = (125 * 1000000) / clock_period_fs;  // 125 = 8000ns / 64
    const int minDeselect = (18 * 1000000 + (clock_period_fs - 1)) / clock_period_fs - (clockDivider + 1) / 2;

    stdio_printf("Max Select: %d, Min Deselect: %d, clock divider: %d\n", maxSelect, minDeselect, clockDivider);

    uint32_t intr_stash = save_and_disable_interrupts();

    // Try and read the PSRAM ID via direct_csr.
    qmi_hw->direct_csr = 30 << QMI_DIRECT_CSR_CLKDIV_LSB | QMI_DIRECT_CSR_EN_BITS;

    // direct-mode operation
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
    {
    }

    // Exit out of QMI in case we've inited already
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    // Turn off quad.
    qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS | (QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB) | PSRAM_CMD_QUAD_END;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
    {
    }
    (void)qmi_hw->direct_rx;
    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS);

    // Read the id
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    uint8_t kgd = 0;
    uint8_t eid = 0;
    for (size_t i = 0; i < 7; i++)
    {
        if (i == 0)
        {
            qmi_hw->direct_tx = PSRAM_CMD_READ_ID;
        }
        else
        {
            qmi_hw->direct_tx = PSRAM_CMD_NOOP;
        }
        while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) == 0)
        {
        }
        while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
        {
        }
        if (i == 5)
        {
            kgd = qmi_hw->direct_rx;
        }
        else if (i == 6)
        {
            eid = qmi_hw->direct_rx;
        }
        else
        {
            (void)qmi_hw->direct_rx;
        }
    }
    
    // Disable direct csr.
    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);
    restore_interrupts(intr_stash);
    if (kgd != PSRAM_ID)
    {
        printf("Invalid PSRAM ID: %x\n", kgd);
        return psram_size;
    }
    printf("Valid PSRAM ID: %x\n", kgd);
    intr_stash = save_and_disable_interrupts();

    // Enable quad mode.
    qmi_hw->direct_csr = (30 << QMI_DIRECT_CSR_CLKDIV_LSB) | QMI_DIRECT_CSR_EN_BITS;
    
    // direct-mode operation
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
    {
    }

    // RESETEN, RESET and quad enable
    for (uint8_t i = 0; i < 4; i++)
    {
        qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        if (i == 0)
        {
            qmi_hw->direct_tx = PSRAM_CMD_RSTEN;
        }
        else if (i == 1)
        {
            qmi_hw->direct_tx = PSRAM_CMD_RST;
        }
        else if (i == 2)
        {
            qmi_hw->direct_tx = PSRAM_CMD_QUAD_ENABLE;
        }
        else 
        {
            qmi_hw->direct_tx = PSRAM_CMD_LINEAR_TOGGLE;
        }
        while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0)
        {
        }
        qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS);
        for (size_t j = 0; j < 20; j++)
        {
            asm("nop");
        }
        (void)qmi_hw->direct_rx;
    }
    // Disable direct csr.
    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);

    qmi_hw->m[1].timing =
        (QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB) | // Break between pages.
        (1 << QMI_M1_TIMING_COOLDOWN_LSB) | (rxdelay << QMI_M1_TIMING_RXDELAY_LSB) |
        (maxSelect << QMI_M1_TIMING_MAX_SELECT_LSB) |  // In units of 64 system clock cycles. PSRAM says 8us max. 8 / 0.00752 /64
                                              // = 16.62
        (minDeselect << QMI_M1_TIMING_MIN_DESELECT_LSB) | // In units of system clock cycles. PSRAM says 50ns.50 / 7.52 = 6.64
        (clockDivider << QMI_M1_TIMING_CLKDIV_LSB);
    
    qmi_hw->m[1].rfmt = (QMI_M1_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_PREFIX_WIDTH_LSB) |
                         (QMI_M1_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M1_RFMT_ADDR_WIDTH_LSB) |
                         (QMI_M1_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_SUFFIX_WIDTH_LSB) |
                         (QMI_M1_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M1_RFMT_DUMMY_WIDTH_LSB) |
                         (QMI_M1_RFMT_DUMMY_LEN_VALUE_24 << QMI_M1_RFMT_DUMMY_LEN_LSB) |
                         (QMI_M1_RFMT_DATA_WIDTH_VALUE_Q << QMI_M1_RFMT_DATA_WIDTH_LSB) |
                         (QMI_M1_RFMT_PREFIX_LEN_VALUE_8 << QMI_M1_RFMT_PREFIX_LEN_LSB) |
                         (QMI_M1_RFMT_SUFFIX_LEN_VALUE_NONE << QMI_M1_RFMT_SUFFIX_LEN_LSB);
    qmi_hw->m[1].rcmd = (PSRAM_CMD_QUAD_READ);
    qmi_hw->m[1].wfmt = (QMI_M1_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_PREFIX_WIDTH_LSB) |
                         (QMI_M1_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M1_WFMT_ADDR_WIDTH_LSB) |
                         (QMI_M1_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_SUFFIX_WIDTH_LSB) |
                         (QMI_M1_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M1_WFMT_DUMMY_WIDTH_LSB) |
                         (QMI_M1_WFMT_DUMMY_LEN_VALUE_NONE << QMI_M1_WFMT_DUMMY_LEN_LSB) |
                         (QMI_M1_WFMT_DATA_WIDTH_VALUE_Q << QMI_M1_WFMT_DATA_WIDTH_LSB) |
                         (QMI_M1_WFMT_PREFIX_LEN_VALUE_8 << QMI_M1_WFMT_PREFIX_LEN_LSB) |
                         (QMI_M1_WFMT_SUFFIX_LEN_VALUE_NONE << QMI_M1_WFMT_SUFFIX_LEN_LSB);
    qmi_hw->m[1].wcmd = (PSRAM_CMD_QUAD_WRITE);

    psram_size = 1024 * 1024; // 1 MiB
    uint8_t size_id = eid >> 5;
    if (eid == 0x26 || size_id == 2)
    {
        psram_size *= 8;//8
    }
    else if (size_id == 0)
    {
        psram_size *= 1;
    }
    else if (size_id == 1)
    {
        psram_size *= 4;
    }

    // Mark that we can write to PSRAM.
    xip_ctrl_hw->ctrl |= XIP_CTRL_WRITABLE_M1_BITS;
    restore_interrupts(intr_stash);
    printf("PSRAM ID: %x %x\n", kgd, eid);
    return psram_size;
}


//  Heap functions

// total size of heap
uint32_t getTotalHeap(void)
{
   extern char __StackLimit, __bss_end__;
   return &__StackLimit  - &__bss_end__;
}

// available heap size
uint32_t getFreeHeap(void)
{
   struct mallinfo m = mallinfo();
   return getTotalHeap() - m.uordblks;
}



//  Test structures

#define TEST_SIZE (16 * 1024)       //  16 * 4 = 64K
#define LOOP_SCALE (200)            //  LOOP_SCALE * 100

uint32_t s_test_memory[TEST_SIZE];      
const uint32_t s_testROM[TEST_SIZE];


typedef struct 
{
    uint32_t *buffer;
    uint32_t buffer_size;
    int loop_scale;
    bool read;
    bool random;
    char * test_name;
    uint64_t result;
} memory_test_config;

memory_test_config s_memory_test_config[] = 
{
    //  Sequential Read
    { s_test_memory, TEST_SIZE, LOOP_SCALE, true, false, "SEQ SRAM READ", 0 },
    { (uint32_t *)s_testROM, TEST_SIZE, LOOP_SCALE, true, false, "SEQ ROM READ", 0 },
    { (uint32_t *)PSRAM_LOCATION, TEST_SIZE, LOOP_SCALE, true, false, "SEQ PSRAM READ", 0 },
    { (uint32_t *)PSRAM_LOCATION_NOCAHE, TEST_SIZE, LOOP_SCALE, true, false, "SEQ PSRAM NOCACHE READ", 0 },

    //  Random Read
    { s_test_memory, TEST_SIZE, LOOP_SCALE, true, true, "RND SRAM READ", 0 },
    { (uint32_t *)s_testROM, TEST_SIZE, LOOP_SCALE, true, true, "RND ROM READ", 0 },
    { (uint32_t *)PSRAM_LOCATION, TEST_SIZE, LOOP_SCALE, true, true, "RND PSRAM READ", 0 },
    { (uint32_t *)PSRAM_LOCATION_NOCAHE, TEST_SIZE, LOOP_SCALE, true, true, "RND PSRAM NOCACHE READ", 0 },


    { s_test_memory, TEST_SIZE, LOOP_SCALE, false, false, "SEQ SRAM WRITE", 0 },
    { (uint32_t *)PSRAM_LOCATION, TEST_SIZE, LOOP_SCALE, false, false, "SEQ PSRAM WRITE", 0 },
    { (uint32_t *)PSRAM_LOCATION_NOCAHE, TEST_SIZE, LOOP_SCALE, false, false, "SEQ PSRAM NOCACHE WRITE", 0 },

    { s_test_memory, TEST_SIZE, LOOP_SCALE, false, true, "RND SRAM WRITE", 0 },
    { (uint32_t *)PSRAM_LOCATION, TEST_SIZE, LOOP_SCALE, false, true, "RND PSRAM WRITE", 0 },
    { (uint32_t *)PSRAM_LOCATION_NOCAHE, TEST_SIZE, LOOP_SCALE, false, true, "RND PSRAM NOCACHE WRITE", 0 }

};

uint32_t s_value = 0;

uint64_t __time_critical_func(memory_test)(uint32_t *buffer, uint32_t buffer_size, int loop_scale, bool read, bool rnd)
{
    uint64_t start = time_us_64();
    int loop_count = 100 * loop_scale;
    uint32_t value = 0;

    if (rnd)
    {
        uint32_t seed_value = 0xDEADBEEF;

        //  Random
        if (read)
        {
            //  Read
            for (int loop = 0; loop < loop_count; loop++)
            {
                for (int i = 0; i < buffer_size; i++)
                {
                    seed_value = (seed_value * 1103515245U + 12345U);
                    value += buffer[seed_value & (buffer_size - 1)];
                }
            }
        }
        else
        {
            //  Write
            for (int loop = 0; loop < loop_count; loop++)
            {
                for (int i = 0; i < buffer_size; i++)
                {
                    seed_value = (seed_value * 1103515245U + 12345U);
                    buffer[seed_value & (buffer_size - 1)] = value++;
                }
            }
        }
    }
    else
    {
        //  Seqential
        if (read)
        {
            //  Read
            for (int loop = 0; loop < loop_count; loop++)
            {
                for (int i = 0; i < buffer_size; i++)
                {
                    value += buffer[i];
                }
            }
        }
        else
        {
            //  Write
            for (int loop = 0; loop < loop_count; loop++)
            {
                for (int i = 0; i < buffer_size; i++)
                {
                    buffer[i] = value++;
                }
            }
        }
    }

    uint64_t delta = time_us_64() - start;

    s_value = value;

    return delta;
}

void __time_critical_func(run_tests)(void)
{
    for (int i = 0; i < (sizeof(s_memory_test_config) / sizeof(memory_test_config)); i++)
    {
        s_memory_test_config[i].result = memory_test(s_memory_test_config[i].buffer, s_memory_test_config[i].buffer_size, s_memory_test_config[i].loop_scale, s_memory_test_config[i].read, s_memory_test_config[i].random);

        printf("Test, %s, 0x%08lX, %d, %d\n", s_memory_test_config[i].test_name, (long unsigned int)s_memory_test_config[i].buffer, (int)(s_memory_test_config[i].buffer_size), (int)(s_memory_test_config[i].result));
    }
}

void __time_critical_func(test_mem)(void)
{
    for (int i = 0; i < (sizeof(s_memory_test_config) / sizeof(memory_test_config)); i++)
    {
        //  Skip tests in Flash memory space
        if ((s_memory_test_config[i].buffer < (uint32_t *)0x10000000) || (s_memory_test_config[i].buffer >= (uint32_t *)0x10000000))
        {
            uint32_t value = 0xDEADBEEF;
            uint32_t buffer_size = s_memory_test_config[i].buffer_size;
            bool result = true;

            for (int x = 0; x < buffer_size; x++)
            {
                s_memory_test_config[i].buffer[x] = value;
                if (s_memory_test_config[i].buffer[x] != value)
                {
                    result = false;
                    break;
                }
            }

            if (result == true)
            {
                printf("Passed Mem Test, %s\n", s_memory_test_config[i].test_name);
            }
            else
            {
                printf("Failed Mem Test, %s\n", s_memory_test_config[i].test_name);
            }
        }
        else
        {
            printf("Skipped Mem Test, %s\n", s_memory_test_config[i].test_name);
        }
    }
}

#define VREG_VSEL         VREG_VOLTAGE_1_20

int __time_critical_func(main)()
{
    stdio_init_all();
    printf("stdio_init_all\n");

#if 0
    //  If we want to try different Sys Clocks

    printf("Sys Clock\n");
    // slightly rise the core voltage, preparation for overclocking
    // vreg_set_voltage(VREG_VSEL);

    // wait a bit, until the raised core VCC has settled
    sleep_ms(2);
    // shift into higher gears...
    set_sys_clock_khz(200000, true);
    sleep_ms(1000);
    printf("Sys Clock changed!\n");
#endif

    //  Wait a bit so we can connect to serial monitor
    sleep_ms(4000);
    printf("Starting!\n");
    sleep_ms(1000);

    //  Get the basic system info
    int clock_hz = clock_get_hz(clk_sys);
    _psram_size = setup_psram(RP2350_XIP_CSI_PIN);

    size_t free_heap = getFreeHeap();
    //  If we want to test malloc'd memory
 //   uint32_t *test_memory = malloc(TEST_SIZE * sizeof(uint32_t));
    size_t free_heap_after = getFreeHeap();

    printf("_psram_size, %d, clock_hz, %d, free_heap, %d, free_heap_after, %d\n", _psram_size, clock_hz, free_heap, free_heap_after);

    //  Check memory
    test_mem();

    //  Run the tests
    run_tests();

    //  Loop
    while (true)
    {
        printf("Loop!\n");
        sleep_ms(10000);
    }
}
